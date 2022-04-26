/*
底层驱动层
与CAN总线对接
*/

#ifndef _MOTORDRIVER_H_
#define _MOTORDRIVER_H_

#include<iostream>
#include"Common.h"
#include<unistd.h>
#include"Communicator.h"

using namespace std;

struct MotorParameter{
    double  gear_rate;               //减速比
    double  rad_code_rate;
    short   left_sign;
    short   right_sign;
    unsigned int    left_motor_id;
    unsigned int    right_motor_id;
};



class MotorDriver{
protected:
    double          gear_rate;               //减速比
    double          rad_code_rate;
    short           side_sign[2];
    double          obstacle_current_limit;
    //double          obstacle_velocity_limit;
    unsigned int    slave_id[2];
    Communicator    communicator;

protected:
    byte                transmit_buffer[2][8];      // 发送消息缓冲区
    const unsigned int  RECEIVE_BUFFER_SIZE;        // 接收消息存储区大小
    byte                (*receive_buffer)[8];       // 接收消息存储区
    unsigned int        *receive_id_buffer;         // 接收消息ID存储区
    unsigned int        receive_message_number;     // 接收消息数

protected:
    unsigned int    lost_communicate_time;

public:
    //double  present_position[2];
    double  present_velocity[2];
    double  present_current[2];
    //int     present_position[2];

public:
    MotorDriver(MotorParameter);
    ~MotorDriver();

protected:
    Status  _transmitMessage(unsigned int);
    Status  _receiveCmd();
    // void    _rotateSpeedToCode(double[2], byte[2][2]);
    // void    _constructMessage(byte[2][2], byte[2][8]);
    void    _constructMessage(double[2]);
    Status  _receiveMessage();
    Status  _analysisMessage();
    Status  _obstacleCheck();


    void    _onDealFeedback(unsigned short&, byte[8]);
    void    _codeToRotateSpeed(unsigned short&, byte[2]);

public:
    double*     getPresentVelocity();
    double*     getPresentCurrent();
    //int*        getPresentPosition();

    Status  velocityControl(double[2]);
    Status  deactive();
    Status  enableMotor();                  //锁定
    Status  disableMotor();                 //解锁
    Status  clearError();                   //故障清除

    Status  loopMonitor();                  //循环监控

};




#endif
