#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include<sensor_msgs/Joy.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>


class KobukiOperation
{
public:
    KobukiOperation(double freq = 10);
    void spin();
private:
    void joy_callback(const sensor_msgs::Joy &joy_msg);
    void kobukiMove(double speed, double turn);
    void kobukiKeep(double duration, bool exit_w_interrupt = true);

    void kobukiStop();
    void kobukiInterpolate();
    void normalOperation();

    ros::NodeHandle _nh;
    ros::Subscriber _joy_sub;
    ros::Publisher  _kobuki_pub;

    const double _freq;
    ros::Rate _update_rate;

    struct control_t {
        double target_speed;
        double control_speed;
        double target_turn;
        double control_turn;
    } _control;
    double _speed, _turn;
    const double _speed_acc, _turn_acc;

    int _print_status;
    int _stop_count;
    bool _exit_program;
};
