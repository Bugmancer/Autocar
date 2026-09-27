#include "utils.h"
#include <algorithm>
#include <cmath>
#include <ros/time.h>

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::livox2PCL(const livox_ros_driver2::CustomMsg::ConstPtr &msg, int filter_num, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    if (!msg || min_range < 0.0 || max_range <= min_range)
        return cloud;
    const std::size_t point_num = std::min<std::size_t>(msg->point_num, msg->points.size());
    filter_num = std::max(1, filter_num);
    cloud->reserve(point_num / filter_num + 1);
    for (std::size_t i = 0; i < point_num; i += static_cast<std::size_t>(filter_num))
    {
        if ((msg->points[i].line < 4) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {

            const float x = msg->points[i].x;
            const float y = msg->points[i].y;
            const float z = msg->points[i].z;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;
            const double squared_range = static_cast<double>(x) * x + static_cast<double>(y) * y +
                                         static_cast<double>(z) * z;
            if (squared_range < min_range * min_range || squared_range > max_range * max_range)
                continue;
            pcl::PointXYZINormal p{};
            p.x = x;
            p.y = y;
            p.z = z;
            p.intensity = msg->points[i].reflectivity;
            p.curvature = msg->points[i].offset_time / 1000000.0f;
            cloud->push_back(p);
        }
    }
    return cloud;
}

double Utils::getSec(const std_msgs::Header &header)
{
    return header.stamp.toSec();
}

double Utils::getLivoxSec(const livox_ros_driver2::CustomMsg &msg)
{
    const double header_time = getSec(msg.header);
    if (header_time > 0.0 && std::isfinite(header_time))
        return header_time;
    return static_cast<double>(msg.timebase) * 1e-9;
}

ros::Time Utils::getTime(double sec)
{
    if (sec < 0.0 || !std::isfinite(sec))
        return ros::Time(0);
    return ros::Time(sec);
}
