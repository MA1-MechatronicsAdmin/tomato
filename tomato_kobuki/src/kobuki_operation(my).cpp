#include "../include/kobuki_operation/kobuki_operation.hpp"
#include<sensor_msgs/Joy.h>
using namespace std;

KobukiOperation::KobukiOperation(double freq) :
    _nh(),
    _freq(freq),
    _update_rate(freq),
    _control{0},
    _speed_acc(0.02),
    _turn_acc(0.01)
{
    _joy_sub = _nh.subscribe("joy",10,&KobukiOperation::joy_callback,this);

    _kobuki_pub = _nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);   
}

void KobukiOperation::joy_callback(const sensor_msgs::Joy &joy_msg)
{
    double _g_speed = 0.35;
    double _g_turn = 1;
    double motion_v_tmp = joy_msg.axes[1] * _g_speed;
    double rotation_v_tmp = joy_msg.axes[0] * _g_turn;

    if( (abs(joy_msg.axes[1])<=0.1)&(abs(joy_msg.axes[0])<=0.1) ){
        cout << "Stop" << endl;
        kobukiStop();
    }else{
        cout << "Move" << endl;
        kobukiMove(motion_v_tmp, rotation_v_tmp);
    }
}


void KobukiOperation::spin()
{
    ///< awaken timer
    ros::Duration(0.1).sleep();

    ///< change terminal setting
    struct termios old_terminal, new_terminal;
    int old_fcntl;
    tcgetattr(STDIN_FILENO, &old_terminal);
    new_terminal = old_terminal;
    new_terminal.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_terminal);
    old_fcntl = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, old_fcntl | O_NONBLOCK);

    while (ros::ok())
    {
        ros::spinOnce();
        normalOperation();
    }
    kobukiStop();

    ///< restore terminal setting
    tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal);
    fcntl(STDIN_FILENO, F_SETFL, old_fcntl);
}


void KobukiOperation::kobukiMove(double speed, double turn)
{
    _control.target_speed = speed;
    _control.target_turn = turn;
    kobukiInterpolate();
}

void KobukiOperation::kobukiStop()
{
    _control.target_speed = 0.0;
    _control.target_turn = 0.0;
    kobukiInterpolate();
}

void KobukiOperation::normalOperation()
{
    kobukiInterpolate();
    _update_rate.sleep();
  
}

void KobukiOperation::kobukiInterpolate()
{
    if(_control.control_speed < _control.target_speed){
        _control.control_speed += _speed_acc;
    }
    else if(_control.target_speed < _control.control_speed){
        _control.control_speed -= _speed_acc;
    }
    else{
        _control.control_speed = _control.target_speed;
    }  
    //std::cout << _control.control_speed << std::endl;

    _control.control_turn = _control.target_turn;

    geometry_msgs::Twist command;
    command.linear.x = _control.control_speed;
    command.angular.z = _control.control_turn;
    _kobuki_pub.publish(command);
}