#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>

#include "utils.h"
#include "map_builder/commons.h"
#include "map_builder/map_builder.h"

struct NodeConfig
{
    std::string imu_topic = "/livox/imu";
    std::string lidar_topic = "/livox/lidar";
    std::string body_frame = "body";
    std::string world_frame = "lidar";
    bool print_time_cost = false;
    double imu_acc_scale = 9.80665;
    double max_scan_duration = 0.5;
    std::size_t max_imu_buffer_size = 4000;
    std::size_t max_lidar_buffer_size = 200;
    std::size_t max_path_poses = 10000;
};

struct StateData
{
    bool lidar_pushed = false;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    double last_lidar_time = -1.0;
    double last_imu_time = -1.0;
    std::deque<IMUData> imu_buffer;
    std::deque<std::pair<double, CloudType::Ptr>> lidar_buffer;
    nav_msgs::Path path;
};

class LIONode
{
public:
    LIONode() : nh_(), pnh_("~")
    {
        ROS_INFO("FAST-LIO2 ROS1 node starting");
        loadParameters();
        imu_sub_ = nh_.subscribe(node_config_.imu_topic, 100, &LIONode::imuCB, this);
        lidar_sub_ = nh_.subscribe(node_config_.lidar_topic, 100, &LIONode::lidarCB, this);
        body_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("body_cloud", 10);
        world_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("world_cloud", 10);
        path_pub_ = nh_.advertise<nav_msgs::Path>("lio_path", 10);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("lio_odom", 10);
        state_data_.path.header.frame_id = node_config_.world_frame;
        kf_ = std::make_shared<IESKF>();
        builder_ = std::make_shared<MapBuilder>(builder_config_, kf_);
        timer_ = nh_.createTimer(ros::Duration(0.020), &LIONode::timerCB, this);
    }

private:
    void loadParameters()
    {
        std::string config_path;
        pnh_.param<std::string>("config_path", config_path, std::string());
        if (!config_path.empty())
        {
          try
          {
            const YAML::Node config = YAML::LoadFile(config_path);
            if (config["imu_topic"]) node_config_.imu_topic = config["imu_topic"].as<std::string>();
            if (config["lidar_topic"]) node_config_.lidar_topic = config["lidar_topic"].as<std::string>();
            if (config["body_frame"]) node_config_.body_frame = config["body_frame"].as<std::string>();
            if (config["world_frame"]) node_config_.world_frame = config["world_frame"].as<std::string>();
            if (config["print_time_cost"]) node_config_.print_time_cost = config["print_time_cost"].as<bool>();
            if (config["imu_acc_scale"]) node_config_.imu_acc_scale = config["imu_acc_scale"].as<double>();
            if (config["max_scan_duration"]) node_config_.max_scan_duration = config["max_scan_duration"].as<double>();
            if (config["max_imu_buffer_size"])
            {
                const long long value = config["max_imu_buffer_size"].as<long long>();
                if (value <= 0) throw std::invalid_argument("max_imu_buffer_size must be positive");
                node_config_.max_imu_buffer_size = static_cast<std::size_t>(value);
            }
            if (config["max_lidar_buffer_size"])
            {
                const long long value = config["max_lidar_buffer_size"].as<long long>();
                if (value <= 0) throw std::invalid_argument("max_lidar_buffer_size must be positive");
                node_config_.max_lidar_buffer_size = static_cast<std::size_t>(value);
            }
            if (config["max_path_poses"])
            {
                const long long value = config["max_path_poses"].as<long long>();
                if (value <= 0) throw std::invalid_argument("max_path_poses must be positive");
                node_config_.max_path_poses = static_cast<std::size_t>(value);
            }
            if (config["lidar_filter_num"]) builder_config_.lidar_filter_num = config["lidar_filter_num"].as<int>();
            if (config["lidar_min_range"]) builder_config_.lidar_min_range = config["lidar_min_range"].as<double>();
            if (config["lidar_max_range"]) builder_config_.lidar_max_range = config["lidar_max_range"].as<double>();
            if (config["scan_resolution"]) builder_config_.scan_resolution = config["scan_resolution"].as<double>();
            if (config["map_resolution"]) builder_config_.map_resolution = config["map_resolution"].as<double>();
            if (config["cube_len"]) builder_config_.cube_len = config["cube_len"].as<double>();
            if (config["det_range"]) builder_config_.det_range = config["det_range"].as<double>();
            if (config["move_thresh"]) builder_config_.move_thresh = config["move_thresh"].as<double>();
            if (config["na"]) builder_config_.na = config["na"].as<double>();
            if (config["ng"]) builder_config_.ng = config["ng"].as<double>();
            if (config["nba"]) builder_config_.nba = config["nba"].as<double>();
            if (config["nbg"]) builder_config_.nbg = config["nbg"].as<double>();
            if (config["imu_init_num"]) builder_config_.imu_init_num = config["imu_init_num"].as<int>();
            if (config["near_search_num"]) builder_config_.near_search_num = config["near_search_num"].as<int>();
            if (config["ieskf_max_iter"]) builder_config_.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
            if (config["gravity_align"]) builder_config_.gravity_align = config["gravity_align"].as<bool>();
            if (config["esti_il"]) builder_config_.esti_il = config["esti_il"].as<bool>();
            if (config["lidar_cov_inv"]) builder_config_.lidar_cov_inv = config["lidar_cov_inv"].as<double>();
            if (config["t_il"])
            {
                const std::vector<double> values = config["t_il"].as<std::vector<double>>();
                if (values.size() != 3) throw std::invalid_argument("t_il must contain exactly 3 values");
                builder_config_.t_il << values[0], values[1], values[2];
            }
            if (config["r_il"])
            {
                const std::vector<double> values = config["r_il"].as<std::vector<double>>();
                if (values.size() != 9) throw std::invalid_argument("r_il must contain exactly 9 values");
                builder_config_.r_il << values[0], values[1], values[2], values[3], values[4], values[5],
                    values[6], values[7], values[8];
            }
            ROS_INFO("Loaded FAST-LIO2 config: %s", config_path.c_str());
          }
          catch (const std::exception &error)
          {
            throw std::runtime_error("failed to load FAST-LIO2 config '" + config_path + "': " + error.what());
          }
        }
        else
        {
            ROS_WARN("~config_path is empty; using built-in FAST-LIO defaults");
        }
        // A launch/private parameter takes precedence over the YAML value.
        double parameter_scale = 0.0;
        if (pnh_.getParam("imu_acc_scale", parameter_scale))
            node_config_.imu_acc_scale = parameter_scale;
        validateParameters();
    }

    void validateParameters() const
    {
        const auto finite_positive = [](double value) { return std::isfinite(value) && value > 0.0; };
        if (node_config_.imu_topic.empty() || node_config_.lidar_topic.empty())
            throw std::invalid_argument("imu_topic and lidar_topic must not be empty");
        if (node_config_.world_frame.empty() || node_config_.body_frame.empty() ||
            node_config_.world_frame == node_config_.body_frame)
            throw std::invalid_argument("world_frame and body_frame must be non-empty and different");
        if (!finite_positive(node_config_.imu_acc_scale))
            throw std::invalid_argument("imu_acc_scale must be finite and greater than zero");
        if (!finite_positive(node_config_.max_scan_duration) || node_config_.max_imu_buffer_size == 0 ||
            node_config_.max_lidar_buffer_size == 0 || node_config_.max_path_poses == 0)
            throw std::invalid_argument("scan duration and buffer/path limits must be greater than zero");
        if (builder_config_.lidar_filter_num < 1 || !std::isfinite(builder_config_.lidar_min_range) ||
            builder_config_.lidar_min_range < 0.0 ||
            !finite_positive(builder_config_.lidar_max_range) ||
            builder_config_.lidar_min_range >= builder_config_.lidar_max_range)
            throw std::invalid_argument("invalid LiDAR filter or range parameters");
        if (!std::isfinite(builder_config_.scan_resolution) || builder_config_.scan_resolution < 0.0 ||
            !finite_positive(builder_config_.map_resolution) ||
            !finite_positive(builder_config_.cube_len) || !finite_positive(builder_config_.det_range) ||
            !finite_positive(builder_config_.move_thresh))
            throw std::invalid_argument("invalid map resolution or local-map parameters");
        if (!std::isfinite(builder_config_.na) || builder_config_.na < 0.0 ||
            !std::isfinite(builder_config_.ng) || builder_config_.ng < 0.0 ||
            !std::isfinite(builder_config_.nba) || builder_config_.nba < 0.0 ||
            !std::isfinite(builder_config_.nbg) || builder_config_.nbg < 0.0)
            throw std::invalid_argument("IMU noise covariance parameters must be finite and non-negative");
        if (builder_config_.imu_init_num < 2 || builder_config_.near_search_num < 3 ||
            builder_config_.ieskf_max_iter < 1 || !finite_positive(builder_config_.lidar_cov_inv))
            throw std::invalid_argument("invalid estimator iteration/search parameters");
        if (!builder_config_.r_il.allFinite() || !builder_config_.t_il.allFinite() ||
            !(builder_config_.r_il.transpose() * builder_config_.r_il).isApprox(M3D::Identity(), 1e-4) ||
            std::abs(builder_config_.r_il.determinant() - 1.0) > 1e-4)
            throw std::invalid_argument("r_il must be a finite proper rotation matrix and t_il must be finite");
    }

    void imuCB(const sensor_msgs::ImuConstPtr &msg)
    {
        const double timestamp = Utils::getSec(msg->header);
        if (!std::isfinite(timestamp) || timestamp <= 0.0)
        {
            ROS_WARN_THROTTLE(2.0, "Dropping IMU message with invalid timestamp");
            return;
        }
        const V3D raw_acceleration(msg->linear_acceleration.x, msg->linear_acceleration.y,
                                   msg->linear_acceleration.z);
        const V3D gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        if (!raw_acceleration.allFinite() || !gyro.allFinite())
        {
            ROS_WARN_THROTTLE(2.0, "Dropping IMU message containing non-finite values");
            return;
        }
        const V3D acceleration = raw_acceleration * node_config_.imu_acc_scale;
        const double acceleration_norm = acceleration.norm();
        if (acceleration_norm < 1.0 || acceleration_norm > 30.0)
            ROS_WARN_THROTTLE(5.0, "Scaled IMU acceleration magnitude %.3f m/s^2 is implausible; check imu_acc_scale",
                              acceleration_norm);
        std::lock_guard<std::mutex> lock(state_data_.imu_mutex);
        if (timestamp <= state_data_.last_imu_time)
        {
            ROS_WARN_THROTTLE(2.0, "Dropping duplicate/out-of-order IMU message (restart after a clock reset)");
            return;
        }
        state_data_.imu_buffer.emplace_back(acceleration, gyro, timestamp);
        while (state_data_.imu_buffer.size() > node_config_.max_imu_buffer_size)
        {
            state_data_.imu_buffer.pop_front();
            ROS_WARN_THROTTLE(5.0, "IMU buffer limit reached; dropping oldest samples");
        }
        state_data_.last_imu_time = timestamp;
    }

    void lidarCB(const livox_ros_driver2::CustomMsgConstPtr &msg)
    {
        if (msg->point_num != msg->points.size())
            ROS_WARN_THROTTLE(5.0, "Livox point_num (%u) differs from points array size (%zu); using the smaller value",
                              msg->point_num, msg->points.size());
        CloudType::Ptr cloud = Utils::livox2PCL(msg, builder_config_.lidar_filter_num,
                                                builder_config_.lidar_min_range,
                                                builder_config_.lidar_max_range);
        if (cloud->empty())
        {
            ROS_WARN_THROTTLE(2.0, "Dropping Livox message with no valid points after filtering");
            return;
        }
        const double timestamp = Utils::getLivoxSec(*msg);
        if (!std::isfinite(timestamp) || timestamp <= 0.0)
        {
            ROS_WARN_THROTTLE(2.0, "Dropping Livox message with invalid header/timebase timestamp");
            return;
        }
        std::lock_guard<std::mutex> lock(state_data_.lidar_mutex);
        if (timestamp <= state_data_.last_lidar_time)
        {
            ROS_WARN_THROTTLE(2.0, "Dropping duplicate/out-of-order LiDAR message (restart after a clock reset)");
            return;
        }
        state_data_.lidar_buffer.emplace_back(timestamp, cloud);
        while (state_data_.lidar_buffer.size() > node_config_.max_lidar_buffer_size)
        {
            if (state_data_.lidar_pushed && state_data_.lidar_buffer.size() > 1)
                state_data_.lidar_buffer.erase(state_data_.lidar_buffer.begin() + 1);
            else
                state_data_.lidar_buffer.pop_front();
            ROS_WARN_THROTTLE(5.0, "LiDAR buffer limit reached; dropping queued scans");
        }
        state_data_.last_lidar_time = timestamp;
    }

    bool syncPackage()
    {
        std::lock_guard<std::mutex> imu_lock(state_data_.imu_mutex);
        std::lock_guard<std::mutex> lidar_lock(state_data_.lidar_mutex);
        if (state_data_.imu_buffer.empty() || state_data_.lidar_buffer.empty()) return false;
        while (!state_data_.lidar_buffer.empty())
        {
            if (!state_data_.lidar_pushed)
            {
                m_package_.cloud = state_data_.lidar_buffer.front().second;
                if (!m_package_.cloud || m_package_.cloud->empty())
                {
                    state_data_.lidar_buffer.pop_front();
                    continue;
                }
                std::sort(m_package_.cloud->points.begin(), m_package_.cloud->points.end(),
                          [](const PointType &a, const PointType &b) { return a.curvature < b.curvature; });
                m_package_.cloud_start_time = state_data_.lidar_buffer.front().first;
                const double scan_duration = m_package_.cloud->points.back().curvature / 1000.0;
                if (!std::isfinite(scan_duration) || scan_duration < 0.0 ||
                    scan_duration > node_config_.max_scan_duration)
                {
                    ROS_WARN_THROTTLE(2.0, "Dropping LiDAR scan with invalid duration %.6f seconds", scan_duration);
                    state_data_.lidar_buffer.pop_front();
                    continue;
                }
                m_package_.cloud_end_time = m_package_.cloud_start_time + scan_duration;
                state_data_.lidar_pushed = true;
            }
            if (state_data_.imu_buffer.front().time > m_package_.cloud_end_time)
            {
                ROS_WARN_THROTTLE(2.0, "Dropping LiDAR scan older than the available IMU history");
                state_data_.lidar_buffer.pop_front();
                state_data_.lidar_pushed = false;
                continue;
            }
            if (state_data_.last_imu_time < m_package_.cloud_end_time) return false;
            m_package_.imus.clear();
            while (!state_data_.imu_buffer.empty() &&
                   state_data_.imu_buffer.front().time <= m_package_.cloud_end_time)
            {
                m_package_.imus.push_back(state_data_.imu_buffer.front());
                state_data_.imu_buffer.pop_front();
            }
            state_data_.lidar_buffer.pop_front();
            state_data_.lidar_pushed = false;
            return !m_package_.imus.empty();
        }
        return false;
    }

    void publishCloud(const ros::Publisher &publisher, const CloudType::Ptr &cloud,
                      const std::string &frame_id, double time)
    {
        if (!cloud || publisher.getNumSubscribers() == 0) return;
        sensor_msgs::PointCloud2 message;
        pcl::toROSMsg(*cloud, message);
        message.header.frame_id = frame_id;
        message.header.stamp = Utils::getTime(time);
        publisher.publish(message);
    }

    void publishOdometry(double time)
    {
        // r_wi/t_wi 给出 IMU/body 在连续 world_frame 中的位姿；这里尚未包含地图定位校正。
        if (odom_pub_.getNumSubscribers() == 0) return;
        nav_msgs::Odometry message;
        message.header.frame_id = node_config_.world_frame;
        message.header.stamp = Utils::getTime(time);
        message.child_frame_id = node_config_.body_frame;
        message.pose.pose.position.x = kf_->x().t_wi.x();
        message.pose.pose.position.y = kf_->x().t_wi.y();
        message.pose.pose.position.z = kf_->x().t_wi.z();
        Eigen::Quaterniond q(kf_->x().r_wi);
        q.normalize();
        message.pose.pose.orientation.x = q.x(); message.pose.pose.orientation.y = q.y();
        message.pose.pose.orientation.z = q.z(); message.pose.pose.orientation.w = q.w();
        // ROS Odometry 的 twist 位于 child_frame，因此把世界系速度逆旋转到 body。
        const V3D velocity = kf_->x().r_wi.transpose() * kf_->x().v;
        message.twist.twist.linear.x = velocity.x(); message.twist.twist.linear.y = velocity.y();
        message.twist.twist.linear.z = velocity.z();
        odom_pub_.publish(message);
    }

    void publishPath(double time)
    {
        if (path_pub_.getNumSubscribers() == 0) return;
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = node_config_.world_frame;
        pose.header.stamp = Utils::getTime(time);
        pose.pose.position.x = kf_->x().t_wi.x(); pose.pose.position.y = kf_->x().t_wi.y();
        pose.pose.position.z = kf_->x().t_wi.z();
        Eigen::Quaterniond q(kf_->x().r_wi);
        q.normalize();
        pose.pose.orientation.x = q.x(); pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z(); pose.pose.orientation.w = q.w();
        state_data_.path.header.stamp = pose.header.stamp;
        if (state_data_.path.poses.size() >= node_config_.max_path_poses)
        {
            const std::size_t trim_count = std::max<std::size_t>(1, node_config_.max_path_poses / 10);
            state_data_.path.poses.erase(state_data_.path.poses.begin(),
                                         state_data_.path.poses.begin() + trim_count);
        }
        state_data_.path.poses.push_back(pose);
        path_pub_.publish(state_data_.path);
    }

    void broadcastTF(double time)
    {
        // 只广播连续的 world -> body；导航所需 map -> world 由独立定位节点补齐。
        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = node_config_.world_frame;
        transform.child_frame_id = node_config_.body_frame;
        transform.header.stamp = Utils::getTime(time);
        Eigen::Quaterniond q(kf_->x().r_wi);
        q.normalize();
        transform.transform.translation.x = kf_->x().t_wi.x();
        transform.transform.translation.y = kf_->x().t_wi.y();
        transform.transform.translation.z = kf_->x().t_wi.z();
        transform.transform.rotation.x = q.x(); transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z(); transform.transform.rotation.w = q.w();
        tf_broadcaster_.sendTransform(transform);
    }

    void timerCB(const ros::TimerEvent &)
    {
        if (!syncPackage()) return;
        const auto start = std::chrono::steady_clock::now();
        builder_->process(m_package_);
        const auto end = std::chrono::steady_clock::now();
        if (node_config_.print_time_cost)
            ROS_WARN("FAST-LIO2 processing time: %.2f ms",
                     std::chrono::duration<double, std::milli>(end - start).count());
        if (builder_->status() != MAPPING) return;
        const State &state = kf_->x();
        if (!state.r_wi.allFinite() || !state.t_wi.allFinite() || !state.v.allFinite())
        {
            ROS_ERROR_THROTTLE(1.0, "FAST-LIO2 state contains non-finite values; suppressing output");
            return;
        }
        // TF、里程计和两种点云共用扫描结束时间，便于定位节点按同一时刻同步。
        const double time = m_package_.cloud_end_time;
        broadcastTF(time); publishOdometry(time);
        // r_il/t_il 是 LiDAR -> IMU/body 外参；body_cloud 供 ICP 和近场障碍处理。
        publishCloud(body_cloud_pub_, builder_->lidar_processor()->transformCloud(m_package_.cloud,
                     kf_->x().r_il, kf_->x().t_il), node_config_.body_frame, time);
        // world_cloud 进一步应用 LIO 姿态，处于连续局部世界系，而非静态地图 map。
        publishCloud(world_cloud_pub_, builder_->lidar_processor()->transformCloud(m_package_.cloud,
                     builder_->lidar_processor()->r_wl(), builder_->lidar_processor()->t_wl()),
                     node_config_.world_frame, time);
        publishPath(time);
    }

    ros::NodeHandle nh_, pnh_;
    ros::Subscriber imu_sub_, lidar_sub_;
    ros::Publisher body_cloud_pub_, world_cloud_pub_, path_pub_, odom_pub_;
    ros::Timer timer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    StateData state_data_;
    SyncPackage m_package_;
    NodeConfig node_config_;
    Config builder_config_;
    std::shared_ptr<IESKF> kf_;
    std::shared_ptr<MapBuilder> builder_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lio_node");
    try
    {
        LIONode node;
        ros::spin();
        return 0;
    }
    catch (const std::exception &error)
    {
        ROS_FATAL("FAST-LIO2 startup failed: %s", error.what());
        return 1;
    }
}
