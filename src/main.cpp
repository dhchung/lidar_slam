#include <iostream>
#include <ros/ros.h>
#include <core_msgs/string_w_header.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#include "gps_module.h"

GPSModule gpsmodule;

void cb_gps(core_msgs::string_w_header::ConstPtr msg) {
    core_msgs::string_w_header data = *msg;
    bool gps_data = gpsmodule.cb_gps(data);
    if(gps_data) {
        std::cout<<"FUCKMAN"<<std::endl;
    }

}


int main(int argc, char ** argv) {
	ros::init(argc, argv, "lidar_slam_node");
	ros::NodeHandle nh;

    ROS_INFO("SLAM based navigation module");
	ros::Subscriber sub_gps = nh.subscribe<core_msgs::string_w_header>("/gps_bypass", 100, cb_gps);

    ros::spin();

    return 0;
}