#pragma once
#include "visualization_msgs/Marker.h"
#include "uwb_nooploop_aoa_serial/UwbAOA0.h"
#include "ultra_serial/ultra.h"

void get_uwb0_pose(uwb_nooploop_aoa_serial::UwbAOA0 req);

void get_us_dis(ultra_serial::ultra req);

bool us_filter(bool now, int index);

void get_ar_pose(visualization_msgs::Marker req);

void pose2tf(double x, double y, double theta, std::string parent_frame, std::string child_frame);
