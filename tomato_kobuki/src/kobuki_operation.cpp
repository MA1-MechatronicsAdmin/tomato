#include "../include/kobuki_operation/kobuki_operation.hpp"
#include<sensor_msgs/Joy.h>

KobukiOperation::KobukiOperation(double freq) :
    _nh(),
    _freq(freq),
    _update_rate(freq),
    _control{0},
    _speed_acc(0.12),
    _turn_acc(1.0)

{
    _joy_sub = _nh.subscribe("joy",10,&KobukiOperation::joy_callback,this);
    _kobuki_pub = _nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);   
}

void KobukiOperation::joy_callback(const sensor_msgs::Joy &joy_msg)
{
    double _g_speed = 0.2;
    double motion_v_tmp = joy_msg.axes[1] * _g_speed;
 
    double _g_turn = 1;
    double rotation_v_tmp = joy_msg.axes[0] * _g_turn;

    if( (abs(joy_msg.axes[1])<=0.1) & (abs(joy_msg.axes[0])<=0.1) ){
        std::cout << "Stop" << std::endl;
        kobukiStop();
    }else{
        std::cout << "Move" << std::endl;
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
    double speed_diff = _control.control_speed - _control.target_speed;
    double max_speed_acc=_speed_acc/_freq;

    if (abs(speed_diff)<= max_speed_acc){
        _control.control_speed = _control.target_speed;
    } else{
        _control.control_speed +=(speed_diff>0?-1:1)*max_speed_acc;
    }

    double turn_diff = _control.control_turn - _control.target_turn;
    double max_turn_acc=_turn_acc/_freq;

    if (abs(turn_diff)<= max_turn_acc){
        _control.control_turn = _control.target_turn;
    } else{
        _control.control_turn +=(turn_diff>0?-1:1)*max_turn_acc;
    }


    std::cout <<  _control.control_turn  << std::endl;

    geometry_msgs::Twist command;
    command.linear.x = _control.control_speed;
    command.angular.z = _control.control_turn;
    _kobuki_pub.publish(command);
}