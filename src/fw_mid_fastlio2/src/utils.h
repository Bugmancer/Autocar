#pragma once
#include <ros/time.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <std_msgs/Header.h>
#include <livox_ros_driver2/CustomMsg.h>

class Utils
{
public:
    static double getSec(const std_msgs::Header &header);
    static double getLivoxSec(const livox_ros_driver2::CustomMsg &msg);
    static pcl::PointCloud<pcl::PointXYZINormal>::Ptr livox2PCL(const livox_ros_driver2::CustomMsg::ConstPtr &msg, int filter_num, double min_range = 0.5, double max_range = 20.0);
    static ros::Time getTime(double sec);
};
