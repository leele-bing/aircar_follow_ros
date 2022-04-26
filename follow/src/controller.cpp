#include "controller.h"
#include <cmath>
#include <ros/ros.h>

// config func (pid para)
Controller::Controller(double p, double d)
{
    kp = p;
    // this.ki = ki;
    kd = d;
    // initial difference is 0
    e_d = 0;
}

// calculate output
double Controller::get_u(double r, double x, double u_up, bool back)
{
    double e = x - r;
    e_d = e - e_d;
    // pid controll
    double u = kp * e + kd * e_d;
    // save last error
    e_d = e;

    // max vel limitation
    if (u > u_up)
    {
        u =  u_up;
    }
    else if (u < -1 * u_up)
    {
        u = -u_up;
    }
    else{}

    // forbid move back
    if (!back && u < 0)
    {
        u = 0;
    }
    else{}

    return u;
}



Controller::~Controller()
{
    ROS_INFO("PID deleted");
}
