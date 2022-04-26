/*
手柄节点ROS程序
将手柄按钮数据转换为动作消息
*/

#include<iostream>
#include<ros/ros.h>
#include"sensor_msgs/Joy.h"
#include"geometry_msgs/Twist.h"
#include"std_msgs/Int8.h"


class JoyMsgTransformer{
protected: 
    double          v_max;
    double          w_max;
    ros::NodeHandle nh;
    ros::Subscriber joy_sub;
    ros::Publisher  vel_ctrl_pub;
    ros::Publisher  stop_pub;
    ros::Publisher  change_mode_pub;

public:
    JoyMsgTransformer();

public:
    void joyCallback(const sensor_msgs::Joy::ConstPtr&);
};

JoyMsgTransformer::JoyMsgTransformer(){
    // 软限速参数设置
    ros::NodeHandle n("~");
    n.param<double>("linearVelocityMax", v_max, 0.5);
    n.param<double>("rotateVelocityMax", w_max, 0.78539816);
    if(v_max < 0) v_max = -v_max;
    if(w_max < 0) w_max = -w_max;
    ROS_INFO("The maximum linear velocity is %lf m/s.", v_max);
    ROS_INFO("The maximum rotate velocity is %lf rad/s.", w_max);

    //订阅话题定义
    joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &JoyMsgTransformer::joyCallback, this);

    //消息发布器定义
    vel_ctrl_pub = nh.advertise<geometry_msgs::Twist>("aircar/cmd_vel", 10);
    stop_pub = nh.advertise<std_msgs::Int8>("aircar/stop", 10);
    change_mode_pub = nh.advertise<std_msgs::Int8>("aircar/change_mode", 10);

}


void JoyMsgTransformer::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg){
    //0键按下，发送切换到IDLE模式消息
    if(joy_msg->buttons.at(0) != 0){
        std_msgs::Int8 change_mode_msg;
        change_mode_msg.data = 1;
        change_mode_pub.publish(change_mode_msg);
        //ROS_INFO("Publish change_mode_msg");
        return;
    }

    //1键按下，发送切换到速控模式消息
    if(joy_msg->buttons.at(1) != 0){
        std_msgs::Int8 change_mode_msg;
        change_mode_msg.data = 2;
        change_mode_pub.publish(change_mode_msg);
        //ROS_INFO("Publish change_mode_msg");
        return;
    }


    //3键按下，发送急停消息
    if(joy_msg->buttons.at(3) != 0){
        std_msgs::Int8 stop_msg;
        stop_msg.data = 1;
        stop_pub.publish(stop_msg);
        //ROS_INFO("Publish stop_msg");
        return;
    }

    //4键按下，传递拨盘的速控信息
    if(joy_msg->buttons.at(4) != 0){
        double v_car = joy_msg->axes.at(1) * v_max;
        double w_car = joy_msg->axes.at(3) * w_max;

        geometry_msgs::Twist vel_ctrl_msg;
        vel_ctrl_msg.linear.x   = v_car;
        vel_ctrl_msg.angular.z  = w_car;
        vel_ctrl_pub.publish(vel_ctrl_msg);
        //ROS_INFO("Publish vel_ctrl_msg");
        return;
    }
    geometry_msgs::Twist vel_ctrl_msg;
    vel_ctrl_msg.linear.x   = 0.0;
    vel_ctrl_msg.angular.z  = 0.0;
    vel_ctrl_pub.publish(vel_ctrl_msg);
}



int main(int argc, char** argv){
    ros::init(argc, argv, "joyROS");
    JoyMsgTransformer joyMsgTransformer;
    ros::spin();

    return 0;
}
