#include <ros/ros.h>
#include <cmath>
#include "observer.h"
#include "utils.h"
#include "uwb_nooploop_aoa_serial/UwbAOA0.h"
#include "ultra_serial/ultra.h"
#include "visualization_msgs/Marker.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#define FOLLOW_D 1.2
#define US_D 0.6

// initialize global
double ar_pose[2] = {FOLLOW_D, 0};
bool ar_dis = false;
double uwb0_pose[2] = {FOLLOW_D, 0};
int uwb_id = -1;
bool us_dis = false;


/**
 * record uwb pose
**/
void get_uwb0_pose(uwb_nooploop_aoa_serial::UwbAOA0 req)
{   
    uwb0_pose[1] = ((double)req.angle) / 180 * M_PI;
    uwb0_pose[0] = (double)req.distance;
    ROS_INFO("uwb0 dis: %f m, angle: %f ' ", req.distance, req.angle);
    if ((abs(uwb0_pose[1]) < 5* M_PI / 6) && !req.time_out)
    {
        uwb_id = 0;
        pose2tf(uwb0_pose[0] * cos(uwb0_pose[1]), uwb0_pose[0] * sin(uwb0_pose[1]), uwb0_pose[1], "/uwb0_link", "/tag0_link");
    }
    else
    {   
        ROS_ERROR("uwb0 timeout");
        uwb_id = -1;
        uwb0_pose[0] = FOLLOW_D;
        uwb0_pose[1] =  0;
    }
}

/**
 * record us topic and decide whether the obstacle is in danger
**/

// initialize here
bool us_t0[4] = {false, false, false, false};
bool us_t1[4] = {false, false, false, false};

void get_us_dis(ultra_serial::ultra req)
{   
    int num = 0;
    us_dis = false;
    // ROS_INFO("spinus %c", us_dis?'T':'F');
    for (float dis : req.distance_array)
    {          
	    // time out
        if (req.time_out)
        {
            ROS_ERROR("TIME OUT");
            us_dis = true;
            break;
        }
        else if (dis < US_D && dis != 0)
        {
            // ROS_INFO("us %d find %f", num, dis);
            bool his = us_filter(true, num);
	        // ROS_INFO("his %c", us_dis?'T':'F');
            us_dis = true && his;
        }
        else
        {
	        // ROS_INFO("us ok %d", num);
            us_filter(false, num);
        }
        num++;
    }
}

/**
 * drop out the noise
**/
bool us_filter(bool now, int index)
{   
    if (us_t0[index] || us_t1[index])
    {
        return true;
    }
    us_t0[index] = us_t1[index];
    us_t1[index] = now;
    return false;
}

/**
 * record ar tag pose
**/
void get_ar_pose(visualization_msgs::Marker req)
{
    // int marker_id = -1;
    // double marker_distance = FOLLOW_D;
    // double marker_angle = 0;
    // ROS_INFO("sub marker");
    // if (!req.empty())
    // {
    // calculate maker 111 pose

    double x = req.pose.position.x;
    double y = req.pose.position.y;
    ar_pose[0] = sqrt(x*x + y*y);
    ar_pose[1] = atan2(y,x);
    ROS_INFO("find marker 111: x %f, y: %f, dis %f m, angle %f '", 
                x, y, ar_pose[0], ar_pose[1] /M_PI * 180);
    ar_dis = true;
}

/**
 * pub uwb tf
**/
void pose2tf(double x, double y, double theta, std::string parent_frame, std::string child_frame)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
  
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = parent_frame;
    transformStamped.child_frame_id = child_frame;
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
}
