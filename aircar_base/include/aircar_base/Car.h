/*
功能应用层
*/

#ifndef _CAR_H_
#define _CAR_H_

#include<iostream>
#include"Common.h"

#include"CarModel.h"
#include"MotorDriver.h"

#include"Communicator.h"

using namespace std;

enum CarState {_IDLE, _ONCONTROL, _ERROR};

class Car{
protected:
    CarModel        car_model;
    MotorDriver     motor_driver;
    //Communicator    communicator;


protected:
    CarState    car_state;


public:
    Car(CarModelParameter, MotorParameter);
    ~Car();


public:
    Status  velControl(double[2]);      //移动底盘速度控制
    Status  deactive();                 //移动底盘常用制动
    Status  toControlMode();            //锁定移动底盘
    Status  toIdleMode();               //解锁移动底盘
    Status  loopUpdate();               //循环更新状态参数

    Status  getOrderOdom(double[5]);    //获取指令里程计
    Status  getWheelOdom(double[5]);    //获取轮式里程计
    void    initOdom();                 //重置里程计(位置)
    void    resetOdom(double[3]);       //重设里程计(位置)


    double*   getCurrent();
    double*   getVelocity();
    CarState  getState();
};




#endif