/*
底盘运动ROS程序

*/

#include<iostream>
#include<ros/ros.h>
#include<cmath>
#include"geometry_msgs/Twist.h"
#include"std_msgs/Int8.h"
#include"nav_msgs/Odometry.h"
#include"aircar_base/MotorCurrent.h"
#include"aircar_base/MotorVelocity.h"
#include"Car.h"
#include"AircarParameters.hpp"

#define CAR_STATE_IDLE 0
#define CAR_STATE_CTRL 1


class AircarController{
protected:
    ros::NodeHandle nh;
    ros::Subscriber vel_ctrl_sub;
    ros::Subscriber stop_sub;
    ros::Subscriber change_mode_sub;
    ros::Publisher  order_odom_pub;
    ros::Subscriber joy_sub;

    ros::Publisher  wheel_odom_pub;
    ros::Publisher  current_pub;
    ros::Publisher  velocity_pub;

protected:
    Car         aircar;
    CarState    car_former_state;

public:
    AircarController(CarModelParameter, MotorParameter);

public:
    void velCtrlCallback(const geometry_msgs::Twist::ConstPtr&);
    void stopCallback(const std_msgs::Int8::ConstPtr&);
    void changeModeCallback(const std_msgs::Int8::ConstPtr&);

    void orderOdomPublisher(unsigned int);
    void wheelOdomPublisher(unsigned int);
    void currentPublisher(unsigned int);
    void velocityPublisher(unsigned int);
    void carLoopUpdate();
};


AircarController::AircarController(CarModelParameter carModelParameter,
                                   MotorParameter motorParameter): 
                                   aircar(carModelParameter, motorParameter){

    //订阅话题定义
    //joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &AircarController::velCtrlCallback, this);
    //vel_ctrl_sub = nh.subscribe<std_msgs::Int8>("")
    vel_ctrl_sub = nh.subscribe<geometry_msgs::Twist>("/aircar/cmd_vel", 10, &AircarController::velCtrlCallback, this);
    stop_sub = nh.subscribe<std_msgs::Int8>("/aircar/stop", 10, &AircarController::stopCallback, this);
    change_mode_sub = nh.subscribe<std_msgs::Int8>("/aircar/change_mode", 10, &AircarController::changeModeCallback, this);

    //消息发布器定义
    order_odom_pub = nh.advertise<nav_msgs::Odometry>("aircar/order_odometery", 10);
    wheel_odom_pub = nh.advertise<nav_msgs::Odometry>("aircar/wheel_odometery", 10);
    current_pub = nh.advertise<aircar_base::MotorCurrent>("aircar/monitor/current", 10);
    velocity_pub = nh.advertise<aircar_base::MotorVelocity>("aircar/monitor/velocity", 10);

    car_former_state = aircar.getState();
}

void AircarController::changeModeCallback(const std_msgs::Int8::ConstPtr& change_mod_msg){
    //转变控制模式
    Status  status_code = CAR_SUCCESS;
    if(change_mod_msg->data == 1){
        status_code = aircar.toIdleMode();
        if(status_code == CAR_SUCCESS){
            ROS_INFO("Change to IDLE mode.");
        } else{
            ROS_WARN("Change to IDLE mode failed! Error code is 0x%08x", status_code);
        }
    } else{
        status_code = aircar.toControlMode();
        if(status_code == CAR_SUCCESS){
            ROS_INFO("Change to COMMAND CONTROL mode.");
        } else{
            ROS_WARN("Change to COMMAND CONTROL mode failed! Error code is 0x%08x", status_code);
        }
    }
    car_former_state = aircar.getState();
}

void AircarController::stopCallback(const std_msgs::Int8::ConstPtr& stop_msg){
    //制动
    Status  status_code = CAR_SUCCESS;
    if(stop_msg->data == 1){
        status_code = aircar.deactive();
        if(status_code == CAR_SUCCESS){
            ROS_INFO("Stop the car.");
        } else{
            ROS_WARN("Stop the car failed! Error code is 0x%08x", status_code);
        }
    }
}


void AircarController::velCtrlCallback(const geometry_msgs::Twist::ConstPtr& vel_ctrl_msg){
    //速控
    Status  status_code = CAR_SUCCESS;
    double vw_car[2] = {0.0};
    vw_car[0] = vel_ctrl_msg->linear.x;
    vw_car[1] = -vel_ctrl_msg->angular.z;
    status_code = aircar.velControl(vw_car);
    if(status_code == CAR_SUCCESS){
        ROS_INFO("Velocity control: v = %lfm/s, w = %lfrad/s.", vw_car[0], vw_car[1]);
    } else{
        ROS_WARN("Velocity control failed! Error code is 0x%08x", status_code);
    }
}


void AircarController::orderOdomPublisher(unsigned int seq){
    /*
    指令里程计发布器
    通过指令计算里程计
    */
    nav_msgs::Odometry order_odom_msg;
    double order_odom[5] = {0.0};

    Status  status_code = CAR_SUCCESS;
    status_code = aircar.getOrderOdom(order_odom);

    order_odom_msg.header.seq       = seq;
    order_odom_msg.header.stamp     = ros::Time::now();
    if(status_code == CAR_SUCCESS){
        order_odom_msg.header.frame_id  = "order_odom";
    } else{
        order_odom_msg.header.frame_id  = "invalid";
    }

    order_odom_msg.pose.pose.position.x     = order_odom[0];
    order_odom_msg.pose.pose.position.y     = order_odom[1];
    order_odom_msg.pose.pose.orientation.z  = sin(order_odom[2] / 2.0);
    order_odom_msg.pose.pose.orientation.w  = cos(order_odom[2] / 2.0);

    order_odom_msg.twist.twist.linear.x     = order_odom[3];
    order_odom_msg.twist.twist.linear.y     = 0;
    order_odom_msg.twist.twist.angular.z    = order_odom[4];

    order_odom_pub.publish(order_odom_msg);
}



void AircarController::wheelOdomPublisher(unsigned int seq){
    /*
    轮式里程计发布器
    通过轮速反馈计算得到的里程计
    */
    nav_msgs::Odometry wheel_odom_msg;
    double wheel_odom[5] = {0.0};

    Status  status_code = CAR_SUCCESS;
    status_code = aircar.getWheelOdom(wheel_odom);

    wheel_odom_msg.header.seq       = seq;
    wheel_odom_msg.header.stamp     = ros::Time::now();
    if(status_code == CAR_SUCCESS){
        wheel_odom_msg.header.frame_id  = "wheel_odom";
    } else{
        wheel_odom_msg.header.frame_id  = "invalid";
    }

    wheel_odom_msg.pose.pose.position.x     = wheel_odom[0];
    wheel_odom_msg.pose.pose.position.y     = wheel_odom[1];
    wheel_odom_msg.pose.pose.orientation.z  = sin(wheel_odom[2] / 2.0);
    wheel_odom_msg.pose.pose.orientation.w  = cos(wheel_odom[2] / 2.0);

    wheel_odom_msg.twist.twist.linear.x     = wheel_odom[3];
    wheel_odom_msg.twist.twist.linear.y     = 0;
    wheel_odom_msg.twist.twist.angular.z    = wheel_odom[4];

    wheel_odom_pub.publish(wheel_odom_msg);
}



void AircarController::currentPublisher(unsigned int seq){
    /*
    电流数据发布器
    发布电机电流数据    
    */
    aircar_base::MotorCurrent current_msg;

    double* current;
    
    current = aircar.getCurrent();
    current_msg.header.seq       = seq;
    current_msg.header.stamp     = ros::Time::now();
    current_msg.header.frame_id  = "";

    current_msg.left_motor_current = current[0];
    current_msg.right_motor_current = current[1];
    current_pub.publish(current_msg);
}

void AircarController::velocityPublisher(unsigned int seq){
    /*
    电流数据发布器
    发布电机电流数据    
    */
    aircar_base::MotorVelocity velocity_msg;
    double* velocity;
    
    velocity = aircar.getVelocity();
    velocity_msg.header.seq       = seq;
    velocity_msg.header.stamp     = ros::Time::now();
    velocity_msg.header.frame_id  = "";

    velocity_msg.left_motor_velocity = velocity[0];
    velocity_msg.right_motor_velocity = velocity[1];
    velocity_pub.publish(velocity_msg);
}


void AircarController::carLoopUpdate(){
    Status  status_code = CAR_SUCCESS;
    status_code = aircar.loopUpdate();
    if(car_former_state != _ERROR && status_code != CAR_SUCCESS){
        ROS_WARN("Error occurred! Error code is 0x%08x", status_code);
    }
    car_former_state = aircar.getState();
}


int main(int argc, char** argv){
    ros::init(argc, argv, "aircarROS");
    AircarController aircarController(carModelParameter, motorParameter);

    unsigned int seq = 0;

    ros::Rate loop_rate(100);
    while(ros::ok()){
        aircarController.carLoopUpdate();
        aircarController.orderOdomPublisher(seq);
        aircarController.wheelOdomPublisher(seq);
        aircarController.currentPublisher(seq);
        aircarController.velocityPublisher(seq);

        seq += 1;
        ros::spinOnce();
        loop_rate.sleep();
    }
    

    return 0;
}
