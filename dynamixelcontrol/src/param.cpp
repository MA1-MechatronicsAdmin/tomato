#include <ros/ros.h>
#include "std_msgs/String.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Joy.h>
#include "DynamixelControl/DynamixelControl.h"
#include <yolo_detection/BoundingBoxArray.h>
#include <yolo_detection/BoundingBox.h>

#include <cmath>
#include <memory>
#include <string>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdexcept>

#define NODE_FREQUENCY        200

#define f1(x, y) -(cos(x) / cos(y))
#define f2(x, y) (sin(x) / sin(y))
#define rad(x) (x * M_PI/180)

// tomato states
double tomato_x = 9;
double tomato_y = 9;
double tomato_z = 0;
double tomato_size = 2.8; //cm
double t = 0;

// arm states
float vel_ax_x = 0.0;
float vel_ax_y = 0.0;
bool arrived = false;
bool paused = false;
bool backed = false;
bool fixed_y = true;
bool goback = false;
double closed = rad(-40);
double opened = rad(10);
double l = 9; // cm

//joy states
bool opening = true;
bool autonomous = false;
int prev_start_button = 0;
int prev_back_button = 0;
float lt = 0;

// camera states
double cam_angle = rad(-30);
double armdim_x = 8;
double armdim_y = 9;

// velocity variables
int16_t vel_mx_write= 0; // -285 ~ 285
float scale_ax = 0.05;
float vel_ax_1 = 0.0;
float vel_ax_2 = 0.0;
double speed_x = 0.02;
double speed_y = 0.02;
float scale_mx = 30.0; // 3/s? that's 30/ds

// create permanent double
class MappedDouble{
  double *ptr;
  int fd;
  std::string filename;

  public:
    MappedDouble(const std::string &file) : filename(file){
      //open/create file
      fd = open(filename.c_str(), O_CREAT | O_RDWR, 0666);
      if(fd == -1) throw std::runtime_error("Cannot open  file");

      //Ensure file is large enough
      struct stat st;
      fstat(fd, &st);
      if(st.st_size < sizeof(double)){
        lseek(fd, sizeof(double) - 1, SEEK_SET);
        write(fd, "", 1); //extend file
      }

      //map file to memory
      ptr = (double *)mmap(nullptr, sizeof(double), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

      if(ptr == MAP_FAILED){
        close(fd);
        throw std::runtime_error("mmap failed");
      }
    }

    ~MappedDouble(){
      if(ptr != MAP_FAILED) munmap(ptr, sizeof(double));
      if( fd != -1)  close(fd);
    }

    double &operator *(){ return *ptr;}
    double *operator ->(){ return ptr;}
    double &value() { return *ptr;}
};
MappedDouble mx_pos("mx_pos.mmap");


// example range
// y(250, 220) minus 150-> (100, 70)?? (because the range is 0,300 not 0,360)
// x(90, 200) -> (-60, 50)??
// (130, 150 (rest)) -> (-20, 0) try 180

// arrived (x, y)
bool is_arrived(std::vector<double> &cur_val, std::vector<double> &target_val){
  for(int i = 0; i <3; i++){
    if(i == 2 || i== 3) continue;
    if(abs(cur_val[i] - target_val[i]) < 0.08) continue;
    else return false;
  }
  return true;
}


// reset arm position
void go_back(std::vector<double> &cur_val, std::vector<double> &target_val){
  target_val[0] = rad(-75);
  target_val[1] = rad(105);
  target_val[4] = rad(75);
  std::vector<double> init_state = {rad(-75), rad(105), opened, 0, rad(75)};
  backed = is_arrived(cur_val, init_state);
  if(backed){
    ROS_ERROR("YESSSSSSSSSS");
    arrived = false;
    paused = false;
    opening = true;
    goback = false;
    t = 0;
  } 
}

void joyCallback(const sensor_msgs::Joy& msg)
{
  // cross 
  if(msg.buttons[1]==1)
    vel_ax_1 = - msg.buttons[1]*scale_ax;
  else if(msg.buttons[2]==1)
    vel_ax_1 = msg.buttons[2]*scale_ax;
  else
    vel_ax_1 = 0;

  // arm x, y
  vel_ax_x = msg.axes[4];
  vel_ax_y = msg.axes[7];
  opening = msg.buttons[5] == 0;

  // mx
  if(msg.buttons[3]==1)
    vel_mx_write= msg.buttons[3]*scale_mx;    
  else if(msg.buttons[0]==1)
    vel_mx_write= -msg.buttons[0]*scale_mx;
  else
    vel_mx_write=0;

  // fixed y or not
  fixed_y = (msg.axes[2] > 0.8)?  false: true;

  int current_start_button = msg.buttons[7];
  // Detect button pressed (start)
  if (current_start_button == 1 && prev_start_button == 0) autonomous = !autonomous;

  int current_back_button = msg.buttons[6];
  // Detect button pressed (back)
  if (current_back_button == 1 && prev_back_button == 0) goback = true;
  
  // Update previous button state
  prev_start_button = current_start_button;
  prev_back_button = current_back_button;

}


void bboxCallback(const yolo_detection::BoundingBoxArray::ConstPtr& msg)
{
  if(paused) return;
  double min_distance = 10000;


  // Process each bounding box
  for (size_t i = 0; i < msg->bounding_boxes.size(); ++i)
  {
    const yolo_detection::BoundingBox& bbox = msg->bounding_boxes[i];
    double distance = tomato_size /((bbox.x_max - bbox.x_min) + (bbox.y_max - bbox.y_min))/2.0 *3111;
    if(bbox.class_name != "tomato" || distance > min_distance){
      continue;
    }else if(abs(bbox.x_max - bbox.x_min - (bbox.y_max-bbox.y_min)) > 0.1*std::max(bbox.x_max - bbox.x_min, bbox.y_max-bbox.y_min)){
      continue;
    }
    double center_x = (bbox.x_max + bbox.x_min)/2.0;
    double center_y = (bbox.y_max + bbox.y_min)/2.0;
    //tomato_x within (7, 21)
    tomato_x = distance * cos(cam_angle) - (240 - center_y )/480 * 11.25 / 17 * distance * sin(cam_angle)- 3; 
    // tomato_y has to be within (0, 14)
    tomato_y = (240 - center_y )/480 * 11.25 / 17 * distance * cos(cam_angle)\
    + distance * sin(cam_angle) + (41.5 - *mx_pos);
    //tomato_z within (-5.5, 5.5)
    tomato_z = (320 - center_x )/640 * 15 / 17 * distance + 1;
  }
}

// square
std::vector<std::pair<double, double>> square{
  {5, 5}, {5, 9}, {9, 9}, {9, 5} 
};
std::vector<std::pair<double, double>> farstraight{
  {9, -10}, {9, 10}// {9, -10 fails because of the arm geomtry}
};
int point = 0;

void limitcheck(std::vector<double> &target_val){
  // upper and lower rotational limits
  std::vector<std::pair<double, double>> limits = {
    {rad(-70), rad(80)},
    {rad(-target_val[0]+10), rad(-target_val[0]+140)},
    {rad(-150), rad(150)},
    {-100, 100},
    {rad(20), rad(120)}
  };

  // gatekeep
  for(int i = 0; i < target_val.size(); i++){
    if(target_val[i] < limits[i].first){
       target_val[i] = limits[i].first;
    }else if (target_val[i] > limits[i].second){
      target_val[i] = limits[i].second;
    }
  }
}

// //Runge-Kutta
// double solveRK4(double &x, double &y, double &h, std::string mode ){
//   if(mode == "dtheta1"){
//     double k1 = h * f1(x, y);
//     double k2 = h * f1(x + 0.5*h, y + 0.5*k1);
//     double k3 = h * f1(x + 0.5*h, y + 0.5*k2);
//     double k4 = h * f1(x + h, y + k3);
//     return 1/6*(k1 + 2*k2 + 2*k3 + k4);
//   }
//   if(mode == "dtheta2"){
//     double k1 = h * f2(x, y);
//     double k2 = h * f2(x + 0.5*h, y + 0.5*k1);
//     double k3 = h * f2(x + 0.5*h, y + 0.5*k2);
//     double k4 = h * f2(x + h, y + k3);
//     return 1/6*(k1 + 2*k2 + 2*k3 + k4);
//   }
// }

bool is_ok(double theta1, double theta2){
  double _x = l*(sin(theta1) + sin(theta2));
  double _y = l*(cos(theta1) - cos(theta2));
  // put limits here
  if(sqrt(_x*_x+_y*_y) > 2*l || _x > 15 || _y < -2 ) {
    ROS_ERROR("invalid square values");
    return false;
  }
  return true;
}

// insert coordinates, arm goes there
void go_to(double _x, double _y, double _z, std::vector<double> &target_val){
  std::vector<double> goal(3);
  if(!is_ok(_x, _y)) return;
  //convert to angle (I got them flipped the entire time)
  goal[1] = asin(sqrt((_x*_x+_y*_y))/2/l) + atan(_y/_x);
  goal[0] = asin(sqrt((_x*_x+_y*_y))/2/l) - atan(_y/_x);
  goal[2] = _z/2.5;
  target_val[0] = goal[0];
  target_val[1] = goal[1];
  target_val[4] = goal[2];
  // if(target_val[0] < goal[0]) target_val[0] += speed_x;
  // else if(target_val[0] > goal[0]) target_val[0] -= speed_x;
  // if(target_val[1] < goal[1]) target_val[1] += speed_y;
  // else if(target_val[1] > goal[1]) target_val[1] -= speed_y;
  // if(target_val[4] < goal[2]) target_val[4] += scale_ax;
  // else if(target_val[4] > goal[2]) target_val[4] -= scale_ax;
}

void make_move(std::vector<double> &target_val, std::vector<double> &theta){
  double signx = vel_ax_x >= 0? 1:-1;
  double signy = vel_ax_y >= 0? 1:-1;
  double x = l*(sin(theta[0]) + sin(theta[1]));
  double y = l*(cos(theta[0]) - cos(theta[1]));
  double z = -theta[4] * 2.5;
  
  double dtheta1 = signx * speed_x;
  double dtheta2 = signy * speed_y;

  // joy stick control
  // TODO use runge-kutta. (p.n never mind runge-kutta is actually worse)
  if(abs(vel_ax_x) > 0.8 && is_ok(target_val[0] + dtheta1, target_val[1] + dtheta1 * sin(theta[0])/sin(theta[1]))){
    target_val[0] += dtheta1 ; //radians
    target_val[1] += dtheta1 * sin(theta[0])/sin(theta[1]);
    //solveRK4(theta[0], theta[1], dtheta1, "dtheta2");
    t = 0;
  }
  else if(abs(vel_ax_y) > 0.8 && !fixed_y && is_ok(target_val[1] + dtheta2, target_val[0] - dtheta2 * cos(theta[1])/cos(theta[0]))){ 
    target_val[1] += dtheta2;
    target_val[0] += -dtheta2 * cos(theta[1])/cos(theta[0]);
    // solveRK4(theta[1], theta[0], dtheta2, "dtheta1");
    t = 0;
  }
  // autonomous
  else if(autonomous && !arrived){
    go_to(tomato_x - armdim_x, tomato_y, tomato_z, target_val);
    paused = true;
    if(t > 5) paused = false; 
    if(t > 7) go_back(theta, target_val);
    arrived = is_arrived(theta, target_val);
    if(arrived){
      opening = false;
      t = 0;
    }
  }

  // states for going back 
  if((arrived && t > 1.5) || goback){
    go_back(theta, target_val);
  }

  target_val[2] = opening ? opened: closed;
  target_val[3] = vel_mx_write;
  double tmp = *mx_pos;
  *mx_pos = tmp + (double)vel_mx_write / 500;
  target_val[4] += vel_ax_1; 
  
  if(abs(t - round(t)) < 0.03){
    ROS_INFO("tomato_xyz: [%f, %f, %f]", tomato_x, tomato_y, tomato_z);
    ROS_INFO("COORDS: [%f, %f, %f]", x, y, z);
    ROS_INFO("t: %f", t);
    ROS_INFO("%f", *mx_pos);

    if(tomato_x > 17) ROS_ERROR("TOO FAR !! move closer");
  }

  // time update
  t += 0.04;

  limitcheck(target_val);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "param");

  *mx_pos = 23;

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Rate cycle_rate(500);
  ros::Subscriber subscriber = nh.subscribe("joy", 1, joyCallback);
  ros::Subscriber sub = nh.subscribe<yolo_detection::BoundingBoxArray>("tomato_detections", 1, bboxCallback);

  std::string dev_name;
  pnh.param<std::string>("dev", dev_name, "/dev/ttyUSB0");
  DynamixelControl dynamixelcontrol(dev_name);

  // add motors
  dynamixelcontrol.addMotor("AX", 5);
  dynamixelcontrol.addMotor("AX", 4);
  dynamixelcontrol.addMotor("AX", 3);
  dynamixelcontrol.addMotor("MX", 10);
  dynamixelcontrol.addMotor("AX", 1);   //stage R/L
  
  dynamixelcontrol.torque_on();

  std::vector<double> target_positions{rad(-75), rad(105), opened, 0, rad(75)};
  std::vector<double> current_values(5);

  while(ros::ok())
  {
    ros::spinOnce();

    make_move(target_positions, current_values);
    dynamixelcontrol.setTarget(target_positions); 
    dynamixelcontrol.write();
    dynamixelcontrol.read();  
    dynamixelcontrol.getCurrentvalues(current_values);
    
    cycle_rate.sleep();
  }
  
  dynamixelcontrol.torque_off();

  return 0;
}