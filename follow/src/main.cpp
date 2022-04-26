#include <ros/ros.h>
#include <cmath>
#include "utils.h"
#include "observer.h"
#include "controller.h"
#include <geometry_msgs/Twist.h>

#define FOLLOW_D 1.2
#define LIM_V 2.0
#define LIM_W 1.0

/**
*@brief ros node name follower
*/

int main(int argc, char **argv)
{
    // initialize node and ns
    ros::init(argc, argv, "f");
    ros::NodeHandle n;
    
    // 3 observer
    ros::Subscriber tag_sub = n.subscribe("/ar_pose_marker", 10, get_ar_pose);
    ros::Subscriber uwb0_sub = n.subscribe("uwb_aoa0_msg", 10, get_uwb0_pose);
    ros::Subscriber us_sub = n.subscribe("/ultra_msg", 10, get_us_dis);
    
    // declare 2 pid
    Controller linear(1.0, 0.3);
    Controller angle(-1, 0.1);

    geometry_msgs::Twist velocity;
    
    ros::Publisher v_pub = n.advertise<geometry_msgs::Twist>("/aircar/cmd_vel", 10);

    ros::Rate loop_rate(20);
    while (ros::ok()) {
        // get pose
        ros::spinOnce();

        // choose ar if exist, else if uwb is reliabel, else stop
        if (ar_id == 0)
        {   
            velocity.angular.z = angle.get_u(0, ar_pose[1], LIM_W, true);
            velocity.linear.x = linear.get_u(FOLLOW_D, ar_pose[0], LIM_V - abs(velocity.angular.z), false);
            ROS_INFO("follow ar");
        }
        else if (uwb_id == 0)
        {
            velocity.angular.z = angle.get_u(0, uwb0_pose[1], LIM_W, true);
            velocity.linear.x = linear.get_u(FOLLOW_D, uwb0_pose[0], LIM_V - abs(velocity.angular.z), false);
            ROS_INFO("follow uwb");
        }
        else
        {
            velocity.linear.x = 0;
            velocity.angular.z = 0;
            ROS_ERROR("NO TARGET");
        }
	    // ROS_INFO("%c", us_dis?'T':'F');

        // when us detect is true, pub <0, 0>
	    if(!us_dis){
            v_pub.publish(velocity);
            ROS_WARN("pub v: l %f, w %f", velocity.linear.x, velocity.angular.z);
	    }   
        else
        {
            velocity.linear.x = 0;
            velocity.angular.z = 0; 
            v_pub.publish(velocity);
            ROS_ERROR("US STOP");
        }
        
        loop_rate.sleep();
    };

    return 0;
}   
