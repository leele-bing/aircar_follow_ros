/*
底盘运动数学模型
差速模型解算、里程计计算
*/


#ifndef _CARMODEL_H_
#define _CARMODEL_H_

#include<iostream>
#include"Common.h"

#include<cmath>
#include<chrono>


using namespace std;


struct CarModelParameter{
    double  wheel_radiu;        // 车轮半径(m)
    double  tread_length;        // 轮间距(m) 
    double  linear_velocity_up_limit;        // 线速度上限(m/s)
    double  rotate_velocity_up_limit;        // 角速度上限(m/s)
    double  linear_velocity_low_limit;        // 线速度上限(m/s)
    double  rotate_velocity_low_limit;        // 角速度上限(m/s)
};


class CarModel{
/*
机器人底盘运动模型
*/
protected:
    double  r_wheel;        // 车轮半径(m)
    double  l_tread;        // 轮间距(m) 
    double  v_ulimit;       // 线速度上限(m/s)
    double  w_ulimit;       // 角速度上限(m/s)
    double  v_llimit;       // 线速度上限(m/s)
    double  w_llimit;       // 角速度上限(m/s)

protected:
    double  odomX_last;     // (m)
    double  odomY_last;     // (m)
    double  odomPhi_last;   // (rad)
    double  v_current;      // 当前线速度(m/s)
    double  w_current;      // 当前角速度(rad/s)
    double  w_left;         // 当前左轮转动速度(rad/s)
    double  w_right;        // 当前右轮转动速度(rad/s)

protected:
    double  odomX_wheel;        // (m)
    double  odomY_wheel;        // (m)
    double  odomPhi_wheel;      // (rad)
    double  v_wheel_current;      // 当前线速度(m/s)
    double  w_wheel_current;      // 当前角速度(rad/s)

protected:
    int64_t t_last;         // 指令里程计最后更新时间
    double  t_step;         // 轮式里程计刷新时间间隔

protected:
    void    _updateOdom();

public:
    CarModel(CarModelParameter);
    ~CarModel();

public:
    Status  checkLimit(double[2]);       //速度限制检测
    void    carVtoMotorW(double[2], double[2]);     //车速到轮速转算
    void    motorWtoCarV(double[2], double[2]);     //轮速到车速转算
    void    updateVelocity(double[2]);   //更新速度（同时会刷新里程计）
    void    initOdom();
    void    resetOdom(double[3]);
    void    getOrderOdom(double[5]);


    void    getWheelOdom(double[5]);
    void    updateWheelOdom(double[2]);


};





#endif