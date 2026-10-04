#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/bind/bind.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

#include "fw_mid_localizer/IsValid.h"
#include "fw_mid_localizer/Relocalize.h"
#include "localizers/commons.h"
#include "localizers/icp_localizer.h"

namespace
{

template <typename T>
void readRequired(const YAML::Node &config, const char *key, T *value)
{
    const YAML::Node entry = config[key];
    if (!entry)
    {
        throw std::runtime_error(std::string("missing required key: ") + key);
    }
    *value = entry.as<T>();
}

std::string normalizedFrame(std::string frame)
{
    while (!frame.empty() && frame.front() == '/')
    {
        frame.erase(frame.begin());
    }
    return frame;
}

bool finitePoseRequest(const fw_mid_localizer::Relocalize::Request &request)
{
    return std::isfinite(request.x) && std::isfinite(request.y) &&
           std::isfinite(request.z) && std::isfinite(request.yaw) &&
           std::isfinite(request.pitch) && std::isfinite(request.roll);
}

}  // 匿名命名空间

struct NodeConfig
{
    std::string cloud_topic = "/fastlio2/body_cloud";
    std::string odom_topic = "/fastlio2/lio_odom";
    std::string map_frame = "map";
    std::string local_frame = "lidar";
    double update_hz = 1.0;
    double input_timeout = 0.5;
    double alignment_timeout = 3.0;
    double max_sync_dt = 0.05;
    bool print_pose = false;
};

struct NodeState
{
    std::mutex mutex;
    bool message_received = false;
    bool request_pending = false;
    bool localized = false;
    std::uint64_t request_id = 0;
    ros::Time last_message_time;
    ros::WallTime last_received;
    ros::WallTime last_alignment;
    CloudType::Ptr last_cloud{new CloudType};
    // last_local_* 表示 body 到连续 LIO 世界的变换，map_local_* 表示该世界到地图的校正。
    M3D last_local_rotation = M3D::Identity();
    V3D last_local_translation = V3D::Zero();
    M3D map_local_rotation = M3D::Identity();
    V3D map_local_translation = V3D::Zero();
    M4F initial_guess = M4F::Identity();
    std::string local_frame = "lidar";
    std::string body_frame;
};

// ROS 定位集成层：同步 body 点云与 LIO 里程计，以 ICP 校正 map -> local，
// 并向导航发布定位有效性。连续的 local -> body 仍由 LIO 节点提供。
class LocalizerNode
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LocalizerNode()
        : nh_(), pnh_("~"), last_update_time_(ros::WallTime::now())
    {
        loadParameters();
        state_.local_frame = config_.local_frame;
        localizer_.reset(new ICPLocalizer(icp_config_));

        cloud_sub_.subscribe(nh_, config_.cloud_topic, 10);
        odom_sub_.subscribe(nh_, config_.odom_topic, 10);
        sync_.reset(new Synchronizer(SyncPolicy(10), cloud_sub_, odom_sub_));
        sync_->setAgePenalty(0.1);
        sync_->setMaxIntervalDuration(ros::Duration(config_.max_sync_dt));
        sync_->registerCallback(boost::bind(&LocalizerNode::syncCallback, this,
                                            boost::placeholders::_1,
                                            boost::placeholders::_2));

        map_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map_cloud", 1, true);
        valid_pub_ = nh_.advertise<std_msgs::Bool>("localization_valid", 1, true);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 10);
        icp_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("icp_pose", 1);
        aligned_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("aligned_cloud", 1);
        fitness_pub_ = nh_.advertise<std_msgs::Float64>("fitness_score", 1);
        relocalize_server_ = nh_.advertiseService(
            "relocalize", &LocalizerNode::relocalizeCallback, this);
        status_server_ = nh_.advertiseService(
            "relocalize_check", &LocalizerNode::statusCallback, this);
        timer_ = nh_.createWallTimer(ros::WallDuration(0.01),
                                     &LocalizerNode::timerCallback, this);
        status_timer_ = nh_.createWallTimer(ros::WallDuration(0.1),
            [this](const ros::WallTimerEvent &) { publishStatus(); });
        publishStatus();

        std::string map_pcd;
        pnh_.param<std::string>("map_pcd", map_pcd, "");
        if (!map_pcd.empty())
        {
            std::vector<double> pose;
            if (!pnh_.getParam("initial_pose", pose) || pose.size() != 6)
                throw std::runtime_error("initial_pose must contain [x, y, z, yaw, pitch, roll] for map -> body");
            fw_mid_localizer::Relocalize::Request request;
            fw_mid_localizer::Relocalize::Response response;
            request.pcd_path = map_pcd;
            request.x = pose[0]; request.y = pose[1]; request.z = pose[2];
            request.yaw = pose[3]; request.pitch = pose[4]; request.roll = pose[5];
            relocalizeCallback(request, response);
            if (!response.success)
                throw std::runtime_error("startup relocalization rejected: " + response.message);
        }

        ROS_INFO_STREAM("fw_mid_localizer started: cloud=" << config_.cloud_topic
                        << ", odom=" << config_.odom_topic
                        << ", TF=" << config_.map_frame << " -> "
                        << config_.local_frame);
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::PointCloud2, nav_msgs::Odometry>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

    void loadParameters()
    {
        std::string config_path;
        if (!pnh_.getParam("config_path", config_path) || config_path.empty())
        {
            throw std::runtime_error("private parameter '~config_path' is required");
        }

        YAML::Node yaml;
        try
        {
            yaml = YAML::LoadFile(config_path);
            readRequired(yaml, "cloud_topic", &config_.cloud_topic);
            readRequired(yaml, "odom_topic", &config_.odom_topic);
            readRequired(yaml, "map_frame", &config_.map_frame);
            readRequired(yaml, "local_frame", &config_.local_frame);
            readRequired(yaml, "update_hz", &config_.update_hz);

            readRequired(yaml, "rough_scan_resolution", &icp_config_.rough_scan_resolution);
            readRequired(yaml, "rough_map_resolution", &icp_config_.rough_map_resolution);
            readRequired(yaml, "rough_max_iteration", &icp_config_.rough_max_iteration);
            readRequired(yaml, "rough_score_thresh", &icp_config_.rough_score_thresh);
            readRequired(yaml, "refine_scan_resolution", &icp_config_.refine_scan_resolution);
            readRequired(yaml, "refine_map_resolution", &icp_config_.refine_map_resolution);
            readRequired(yaml, "refine_max_iteration", &icp_config_.refine_max_iteration);
            readRequired(yaml, "refine_score_thresh", &icp_config_.refine_score_thresh);
        }
        catch (const YAML::Exception &error)
        {
            throw std::runtime_error("failed to load localizer config '" + config_path +
                                     "': " + error.what());
        }

        // 私有参数覆盖 YAML，允许 launch 单独调整话题、坐标系和时效门限。
        pnh_.param("cloud_topic", config_.cloud_topic, config_.cloud_topic);
        pnh_.param("odom_topic", config_.odom_topic, config_.odom_topic);
        pnh_.param("map_frame", config_.map_frame, config_.map_frame);
        pnh_.param("local_frame", config_.local_frame, config_.local_frame);
        pnh_.param("update_hz", config_.update_hz, config_.update_hz);
        pnh_.param("input_timeout", config_.input_timeout, config_.input_timeout);
        pnh_.param("alignment_timeout", config_.alignment_timeout, config_.alignment_timeout);
        pnh_.param("max_sync_dt", config_.max_sync_dt, config_.max_sync_dt);
        pnh_.param("print_pose", config_.print_pose, config_.print_pose);

        config_.map_frame = normalizedFrame(config_.map_frame);
        config_.local_frame = normalizedFrame(config_.local_frame);
        if (config_.cloud_topic.empty() || config_.odom_topic.empty())
        {
            throw std::runtime_error("cloud_topic and odom_topic must not be empty");
        }
        if (config_.map_frame.empty() || config_.local_frame.empty())
        {
            throw std::runtime_error("map_frame and local_frame must not be empty");
        }
        if (!std::isfinite(config_.update_hz) || config_.update_hz <= 0.0)
        {
            throw std::runtime_error("update_hz must be greater than zero");
        }
        if (!std::isfinite(config_.input_timeout) || config_.input_timeout <= 0.0 ||
            !std::isfinite(config_.alignment_timeout) || config_.alignment_timeout <= 0.0 ||
            !std::isfinite(config_.max_sync_dt) || config_.max_sync_dt <= 0.0)
            throw std::runtime_error("localization timeouts must be finite and positive");
        if (!std::isfinite(icp_config_.rough_scan_resolution) ||
            !std::isfinite(icp_config_.rough_map_resolution) ||
            !std::isfinite(icp_config_.refine_scan_resolution) ||
            !std::isfinite(icp_config_.refine_map_resolution) ||
            !std::isfinite(icp_config_.rough_score_thresh) ||
            !std::isfinite(icp_config_.refine_score_thresh) ||
            icp_config_.rough_scan_resolution < 0.0 ||
            icp_config_.rough_map_resolution < 0.0 ||
            icp_config_.refine_scan_resolution < 0.0 ||
            icp_config_.refine_map_resolution < 0.0 ||
            icp_config_.rough_score_thresh < 0.0 ||
            icp_config_.refine_score_thresh < 0.0 ||
            icp_config_.rough_max_iteration <= 0 ||
            icp_config_.refine_max_iteration <= 0)
        {
            throw std::runtime_error("ICP resolutions/scores must be non-negative and iterations positive");
        }

        ROS_INFO_STREAM("loaded localizer config: " << config_path);
    }

    void syncCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_message,
                      const nav_msgs::OdometryConstPtr &odom_message)
    {
        // 点云必须位于里程计的 child frame；只有帧名、采样时间均匹配才能组合变换。
        const std::string odom_frame = normalizedFrame(odom_message->header.frame_id);
        const std::string body_frame = normalizedFrame(odom_message->child_frame_id);
        if (odom_frame.empty() || body_frame.empty() ||
            normalizedFrame(cloud_message->header.frame_id) != body_frame ||
            odom_frame == body_frame || odom_frame == config_.map_frame ||
            body_frame == config_.map_frame)
        {
            ROS_WARN_THROTTLE(2.0, "discarding input: cloud must be in the odometry child frame, with distinct map/local/body frames");
            return;
        }
        // 既限制两路采样时间差，也限制点云相对 ROS 时钟的年龄及允许的未来偏差。
        if (cloud_message->header.stamp.isZero() || odom_message->header.stamp.isZero() ||
            std::abs((cloud_message->header.stamp - odom_message->header.stamp).toSec()) > config_.max_sync_dt ||
            (ros::Time::now() - cloud_message->header.stamp).toSec() > config_.input_timeout ||
            (cloud_message->header.stamp - ros::Time::now()).toSec() > config_.max_sync_dt)
        {
            ROS_WARN_THROTTLE(2.0, "discarding stale or unsynchronized cloud/odometry timestamps");
            return;
        }
        Eigen::Quaterniond orientation(
            odom_message->pose.pose.orientation.w,
            odom_message->pose.pose.orientation.x,
            odom_message->pose.pose.orientation.y,
            odom_message->pose.pose.orientation.z);
        const double norm = orientation.norm();
        const auto &position = odom_message->pose.pose.position;
        if (!std::isfinite(norm) || norm < 1e-9 ||
            !std::isfinite(position.x) || !std::isfinite(position.y) ||
            !std::isfinite(position.z))
        {
            ROS_WARN_THROTTLE(2.0, "discarding synchronized input with invalid odometry pose");
            return;
        }
        orientation.normalize();

        CloudType::Ptr cloud(new CloudType);
        pcl::fromROSMsg(*cloud_message, *cloud);
        cloud->is_dense = false;
        std::vector<int> finite_indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, finite_indices);
        if (cloud->empty())
        {
            ROS_WARN_THROTTLE(2.0, "discarding empty body cloud");
            return;
        }

        std::lock_guard<std::mutex> lock(state_.mutex);
        if (state_.message_received && cloud_message->header.stamp <= state_.last_message_time)
        {
            ROS_WARN_THROTTLE(2.0, "discarding repeated/backward cloud timestamp; restart LIO/localizer after a clock reset");
            return;
        }
        if (!state_.message_received)
        {
            state_.local_frame = odom_frame;
            state_.body_frame = body_frame;
            if (state_.local_frame != config_.local_frame)
            {
                ROS_WARN_STREAM("using odometry frame '" << state_.local_frame
                                << "' for map TF instead of configured frame '"
                                << config_.local_frame << "'");
            }
        }
        else if (odom_frame != state_.local_frame || body_frame != state_.body_frame)
        {
            ROS_WARN_THROTTLE(2.0, "odometry frame changed; synchronized sample ignored");
            return;
        }

        state_.last_cloud = cloud;
        state_.last_local_rotation = orientation.toRotationMatrix();
        state_.last_local_translation = V3D(position.x, position.y, position.z);
        state_.last_message_time = cloud_message->header.stamp;
        state_.last_received = ros::WallTime::now();
        state_.message_received = true;
    }

    void timerCallback(const ros::WallTimerEvent &)
    {
        ros::Time message_time;
        M3D existing_rotation = M3D::Identity();
        V3D existing_translation = V3D::Zero();
        M3D latest_local_rotation = M3D::Identity();
        V3D latest_local_translation = V3D::Zero();
        std::string local_frame;
        bool publish_existing_tf = false;
        {
            std::lock_guard<std::mutex> lock(state_.mutex);
            if (!state_.message_received)
            {
                return;
            }
            message_time = state_.last_message_time;
            existing_rotation = state_.map_local_rotation;
            existing_translation = state_.map_local_translation;
            latest_local_rotation = state_.last_local_rotation;
            latest_local_translation = state_.last_local_translation;
            local_frame = state_.local_frame;
            publish_existing_tf = validLocked();
        }

        if (publish_existing_tf)
        {
            // 两次 ICP 之间保留 map -> local 校正，并用最新 LIO 位姿更新 map -> body。
            broadcastTransform(ros::Time::now(), local_frame, existing_rotation,
                               existing_translation);
            if (message_time != last_pose_stamp_)
            {
                const M3D map_body_rotation =
                    existing_rotation * latest_local_rotation;
                const V3D map_body_translation =
                    existing_rotation * latest_local_translation + existing_translation;
                publishPose(message_time, map_body_rotation, map_body_translation);
                last_pose_stamp_ = message_time;
            }
        }

        // TF/位姿随输入更新，昂贵的 ICP 另按墙钟限频；同一扫描最多尝试一次。
        const ros::WallTime now = ros::WallTime::now();
        if ((now - last_update_time_).toSec() < (1.0 / config_.update_hz))
        {
            return;
        }
        last_update_time_ = now;

        CloudType::Ptr cloud(new CloudType);
        M3D current_local_rotation;
        V3D current_local_translation;
        M4F initial_guess = M4F::Identity();
        bool request_pending = false;
        std::uint64_t request_id = 0;
        {
            std::lock_guard<std::mutex> lock(state_.mutex);
            if (!inputFreshLocked() || message_time == last_attempt_stamp_)
            {
                return;
            }
            if (state_.request_pending)
            {
                initial_guess = state_.initial_guess;
                request_pending = true;
            }
            else if (state_.localized)
            {
                initial_guess.block<3, 3>(0, 0) =
                    (state_.map_local_rotation * state_.last_local_rotation).cast<float>();
                initial_guess.block<3, 1>(0, 3) =
                    (state_.map_local_rotation * state_.last_local_translation +
                     state_.map_local_translation).cast<float>();
            }
            else
            {
                return;
            }

            pcl::copyPointCloud(*state_.last_cloud, *cloud);
            current_local_rotation = state_.last_local_rotation;
            current_local_translation = state_.last_local_translation;
            message_time = state_.last_message_time;
            local_frame = state_.local_frame;
            request_id = state_.request_id;
            last_attempt_stamp_ = message_time;
        }

        // ICP 使用同一份点云/里程计快照，计算期间释放状态锁。
        // 当前仍是单线程回调，耗时配准会同时延迟输入回调与状态心跳。
        bool converged = false;
        double rough_score = 0.0;
        double refine_score = 0.0;
        {
            std::lock_guard<std::mutex> lock(localizer_mutex_);
            localizer_->setInput(cloud);
            converged = localizer_->align(initial_guess);
            rough_score = localizer_->roughScore();
            refine_score = localizer_->refineScore();
        }

        if (!converged)
        {
            if (request_pending)
            {
                ROS_WARN_STREAM_THROTTLE(2.0,
                    "relocalization has not converged (rough score=" << rough_score
                    << ", refine score=" << refine_score << ")");
            }
            return;
        }

        const M3D map_body_rotation =
            initial_guess.block<3, 3>(0, 0).cast<double>();
        const V3D map_body_translation =
            initial_guess.block<3, 1>(0, 3).cast<double>();
        // T_map_local = T_map_body * inverse(T_local_body)，消去本次扫描的局部运动。
        const M3D new_map_local_rotation =
            map_body_rotation * current_local_rotation.transpose();
        const V3D new_map_local_translation =
            -new_map_local_rotation * current_local_translation + map_body_translation;

        bool result_is_current = false;
        {
            std::lock_guard<std::mutex> lock(state_.mutex);
            // 请求编号用于识别结果所属初值；仅编号仍一致时才接受配准结果。
            if (request_id == state_.request_id)
            {
                state_.map_local_rotation = new_map_local_rotation;
                state_.map_local_translation = new_map_local_translation;
                state_.localized = true;
                state_.last_alignment = ros::WallTime::now();
                state_.request_pending = false;
                result_is_current = inputFreshLocked();
            }
        }
        if (!result_is_current)
        {
            return;
        }

        if (request_pending)
        {
            const Eigen::Quaterniond quaternion(map_body_rotation);
            ROS_INFO_STREAM("relocalization converged (rough score=" << rough_score
                            << ", refine score=" << refine_score
                            << ", body xyz=[" << map_body_translation.x() << ", "
                            << map_body_translation.y() << ", "
                            << map_body_translation.z() << "], q_xyzw=["
                            << quaternion.x() << ", " << quaternion.y() << ", "
                            << quaternion.z() << ", " << quaternion.w() << "])");
        }
        broadcastTransform(ros::Time::now(), local_frame, new_map_local_rotation,
                           new_map_local_translation);
        publishPose(message_time, map_body_rotation, map_body_translation);
        publishAlignment(message_time, cloud, initial_guess, refine_score);
        last_pose_stamp_ = message_time;
        publishStatus();
    }

    bool relocalizeCallback(fw_mid_localizer::Relocalize::Request &request,
                            fw_mid_localizer::Relocalize::Response &response)
    {
        if (request.pcd_path.empty())
        {
            response.success = false;
            response.message = "pcd_path is empty";
            return true;
        }
        std::ifstream map_file(request.pcd_path);
        if (!map_file.good())
        {
            response.success = false;
            response.message = "PCD file not found or unreadable";
            return true;
        }
        if (!finitePoseRequest(request))
        {
            response.success = false;
            response.message = "initial pose contains NaN or infinity";
            return true;
        }

        std::string load_error;
        {
            std::lock_guard<std::mutex> lock(localizer_mutex_);
            if (!localizer_->loadMap(request.pcd_path, &load_error))
            {
                response.success = false;
                response.message = load_error;
                return true;
            }
        }

        // 服务初值是 body 在 map 中的位置/姿态，平移单位米、欧拉角单位弧度。
        const Eigen::AngleAxisd yaw_angle(request.yaw, Eigen::Vector3d::UnitZ());
        const Eigen::AngleAxisd roll_angle(request.roll, Eigen::Vector3d::UnitX());
        const Eigen::AngleAxisd pitch_angle(request.pitch, Eigen::Vector3d::UnitY());
        M4F initial_guess = M4F::Identity();
        // 保留源项目的偏航、横滚、俯仰旋转组合顺序，保存初值时也必须使用此约定。
        initial_guess.block<3, 3>(0, 0) =
            (yaw_angle * roll_angle * pitch_angle).toRotationMatrix().cast<float>();
        initial_guess.block<3, 1>(0, 3) = V3F(request.x, request.y, request.z);

        {
            std::lock_guard<std::mutex> lock(state_.mutex);
            state_.initial_guess = initial_guess;
            state_.request_pending = true;
            state_.localized = false;
            ++state_.request_id;
            last_attempt_stamp_ = ros::Time();
        }
        publishMapCloud(ros::Time::now());
        publishStatus();

        response.success = true;
        response.message = "request accepted; waiting for ICP convergence";
        ROS_INFO_STREAM("accepted relocalization request for " << request.pcd_path);
        return true;
    }

    bool statusCallback(fw_mid_localizer::IsValid::Request &request,
                        fw_mid_localizer::IsValid::Response &response)
    {
        (void)request;
        std::lock_guard<std::mutex> lock(state_.mutex);
        response.valid = validLocked();
        return true;
    }

    bool validLocked() const
    {
        // 曾经收敛不等于当前可用；输入和最近一次成功配准都必须在各自时间窗内。
        const ros::WallTime now = ros::WallTime::now();
        return state_.localized && !state_.request_pending &&
               inputFreshLocked() &&
               (now - state_.last_alignment).toSec() <= config_.alignment_timeout;
    }

    bool inputFreshLocked() const
    {
        // 墙钟检查接收中断，ROS 时间检查采样过期；调用者必须已持有状态锁。
        const double stamp_age = (ros::Time::now() - state_.last_message_time).toSec();
        return state_.message_received &&
               (ros::WallTime::now() - state_.last_received).toSec() <= config_.input_timeout &&
               stamp_age >= -config_.max_sync_dt && stamp_age <= config_.input_timeout;
    }

    void publishStatus()
    {
        std_msgs::Bool message;
        {
            std::lock_guard<std::mutex> lock(state_.mutex);
            message.data = validLocked();
        }
        valid_pub_.publish(message);
    }

    void broadcastTransform(const ros::Time &stamp, const std::string &local_frame,
                            const M3D &rotation, const V3D &translation)
    {
        // TF 的 parent 为 map、child 为 local；矩阵把 local 中的点转换到 map。
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = config_.map_frame;
        transform.child_frame_id = local_frame;
        transform.transform.translation.x = translation.x();
        transform.transform.translation.y = translation.y();
        transform.transform.translation.z = translation.z();
        const Eigen::Quaterniond quaternion(rotation);
        transform.transform.rotation.x = quaternion.x();
        transform.transform.rotation.y = quaternion.y();
        transform.transform.rotation.z = quaternion.z();
        transform.transform.rotation.w = quaternion.w();
        tf_broadcaster_.sendTransform(transform);
    }

    geometry_msgs::PoseStamped makePose(const ros::Time &stamp, const M3D &rotation,
                                        const V3D &translation) const
    {
        geometry_msgs::PoseStamped message;
        message.header.stamp = stamp.isZero() ? ros::Time::now() : stamp;
        message.header.frame_id = config_.map_frame;
        message.pose.position.x = translation.x();
        message.pose.position.y = translation.y();
        message.pose.position.z = translation.z();
        Eigen::Quaterniond quaternion(rotation);
        quaternion.normalize();
        message.pose.orientation.x = quaternion.x();
        message.pose.orientation.y = quaternion.y();
        message.pose.orientation.z = quaternion.z();
        message.pose.orientation.w = quaternion.w();
        return message;
    }

    void publishPose(const ros::Time &stamp, const M3D &rotation,
                     const V3D &translation)
    {
        pose_pub_.publish(makePose(stamp, rotation, translation));
    }

    void publishAlignment(const ros::Time &stamp, const CloudType::ConstPtr &cloud,
                          const M4F &map_body, double score)
    {
        const auto pose = makePose(stamp, map_body.block<3, 3>(0, 0).cast<double>(),
                                   map_body.block<3, 1>(0, 3).cast<double>());
        icp_pose_pub_.publish(pose);
        std_msgs::Float64 fitness;
        fitness.data = score;
        fitness_pub_.publish(fitness);
        if (aligned_cloud_pub_.getNumSubscribers() > 0)
        {
            // 仅用于配准诊断：将输入 body 点云按本次 ICP 结果投影到 map。
            CloudType aligned;
            pcl::transformPointCloud(*cloud, aligned, map_body);
            sensor_msgs::PointCloud2 message;
            pcl::toROSMsg(aligned, message);
            message.header = pose.header;
            aligned_cloud_pub_.publish(message);
        }
        if (config_.print_pose)
        {
            const auto &p = pose.pose.position;
            const auto &q = pose.pose.orientation;
            double roll, pitch, yaw;
            tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);
            ROS_INFO("ICP map->body xyz_m=[%.4f, %.4f, %.4f] rpy_rad=[%.5f, %.5f, %.5f] q_xyzw=[%.5f, %.5f, %.5f, %.5f] fitness_m2=%.6f",
                     p.x, p.y, p.z, roll, pitch, yaw, q.x, q.y, q.z, q.w, score);
        }
    }

    void publishMapCloud(const ros::Time &stamp)
    {
        CloudType map_cloud;
        {
            std::lock_guard<std::mutex> lock(localizer_mutex_);
            const CloudType::ConstPtr source = localizer_->refineMap();
            if (!source || source->empty())
            {
                return;
            }
            pcl::copyPointCloud(*source, map_cloud);
        }

        sensor_msgs::PointCloud2 message;
        pcl::toROSMsg(map_cloud, message);
        message.header.frame_id = config_.map_frame;
        message.header.stamp = stamp;
        map_cloud_pub_.publish(message);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    NodeConfig config_;
    NodeState state_;
    ICPConfig icp_config_;
    std::unique_ptr<ICPLocalizer> localizer_;
    std::mutex localizer_mutex_;

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    std::unique_ptr<Synchronizer> sync_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    ros::Publisher map_cloud_pub_;
    ros::Publisher valid_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher icp_pose_pub_;
    ros::Publisher aligned_cloud_pub_;
    ros::Publisher fitness_pub_;
    ros::ServiceServer relocalize_server_;
    ros::ServiceServer status_server_;
    ros::WallTimer timer_;
    ros::WallTimer status_timer_;
    ros::WallTime last_update_time_;
    ros::Time last_pose_stamp_;
    ros::Time last_attempt_stamp_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localizer_node");
    try
    {
        LocalizerNode node;
        // 所有订阅、服务和定时器串行执行；切换多线程前须审查 ICP/地图替换的时序。
        ros::spin();
    }
    catch (const std::exception &error)
    {
        ROS_FATAL_STREAM("failed to start fw_mid_localizer: " << error.what());
        return 1;
    }
    return 0;
}
