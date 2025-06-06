#ifndef MOTOR_H
#define MOTOR_H

#include "dynamixel_sdk/dynamixel_sdk.h"

#define BAUDRATE              1000000


class Motor
{
protected:
    dynamixel::PortHandler* porthandler;
    dynamixel::PacketHandler* packethandler;
    double goal_value;          //[rad] or [rad/s] or [mA]?
    bool protocol_version_check();

public:
    Motor(int id, float version, dynamixel::PortHandler* porthandler, dynamixel::PacketHandler* packethandler);
    ~Motor();

    const float protocol_version;
    const int id;

    double get_goal_value();
    virtual double get_current_value() = 0;
    
    virtual bool torque_on() = 0;
    virtual bool torque_off() = 0;

    virtual bool goalset(double goal /*[rad]*/) = 0;
    virtual bool read() = 0;
};

#endif