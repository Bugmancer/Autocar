#include "localizers/icp_localizer.h"

#include <cmath>
#include <sstream>
#include <vector>

#include <pcl/common/io.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>

// 自有配准包装层：管理两级降采样与收敛门限，ICP 数值求解由 PCL 提供。
ICPLocalizer::ICPLocalizer(const ICPConfig &config)
    : config_(config),
      refine_input_(new CloudType),
      rough_input_(new CloudType),
      refine_target_(new CloudType),
      rough_target_(new CloudType)
{
}

CloudType::Ptr ICPLocalizer::downsample(const CloudType::ConstPtr &cloud,
                                        const double resolution)
{
    // 分辨率为零时只复制点云，正值为三轴一致的体素边长（米）。
    CloudType::Ptr output(new CloudType);
    if (resolution <= 0.0)
    {
        pcl::copyPointCloud(*cloud, *output);
        return output;
    }

    pcl::VoxelGrid<PointType> filter;
    const float leaf_size = static_cast<float>(resolution);
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    filter.setInputCloud(cloud);
    filter.filter(*output);
    return output;
}

bool ICPLocalizer::loadMap(const std::string &path, std::string *error_message)
{
    CloudType::Ptr cloud(new CloudType);
    if (pcl::io::loadPCDFile<PointType>(path, *cloud) < 0)
    {
        if (error_message != nullptr)
        {
            *error_message = "failed to read PCD file: " + path;
        }
        return false;
    }

    std::vector<int> valid_indices;
    cloud->is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, valid_indices);
    if (cloud->empty())
    {
        if (error_message != nullptr)
        {
            *error_message = "PCD map contains no valid points";
        }
        return false;
    }

    CloudType::Ptr refine_target = downsample(cloud, config_.refine_map_resolution);
    CloudType::Ptr rough_target = downsample(cloud, config_.rough_map_resolution);
    if (refine_target->empty() || rough_target->empty())
    {
        if (error_message != nullptr)
        {
            *error_message = "PCD map is empty after voxel filtering";
        }
        return false;
    }

    // 两级目标点云均构建成功后再替换，加载失败时保留上一份有效地图。
    refine_target_.swap(refine_target);
    rough_target_.swap(rough_target);
    return true;
}

void ICPLocalizer::setInput(const CloudType::ConstPtr &cloud)
{
    // 输入快照先去除非有限点，再分别为粗配准、精配准生成独立降采样点云。
    CloudType::Ptr finite_cloud(new CloudType);
    pcl::copyPointCloud(*cloud, *finite_cloud);
    finite_cloud->is_dense = false;
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*finite_cloud, *finite_cloud, valid_indices);
    refine_input_ = downsample(finite_cloud, config_.refine_scan_resolution);
    rough_input_ = downsample(finite_cloud, config_.rough_scan_resolution);
}

bool ICPLocalizer::align(M4F &guess)
{
    // guess 输入为 body -> map 初值，只有粗配准和精配准都通过时才回写结果。
    last_rough_score_ = std::numeric_limits<double>::infinity();
    last_refine_score_ = std::numeric_limits<double>::infinity();
    if (refine_input_->empty() || rough_input_->empty() ||
        refine_target_->empty() || rough_target_->empty())
    {
        return false;
    }

    // 先用较稀疏点云扩大收敛范围，再把粗配准结果作为精配准初值。
    CloudType aligned_cloud;
    rough_icp_.setMaximumIterations(config_.rough_max_iteration);
    rough_icp_.setInputSource(rough_input_);
    rough_icp_.setInputTarget(rough_target_);
    rough_icp_.align(aligned_cloud, guess);
    // PCL fitness 为最近邻平方距离均值，门限单位是平方米，不能按米直接配置。
    last_rough_score_ = rough_icp_.getFitnessScore();
    if (!rough_icp_.hasConverged() || !std::isfinite(last_rough_score_) ||
        last_rough_score_ > config_.rough_score_thresh)
    {
        return false;
    }

    refine_icp_.setMaximumIterations(config_.refine_max_iteration);
    refine_icp_.setInputSource(refine_input_);
    refine_icp_.setInputTarget(refine_target_);
    refine_icp_.align(aligned_cloud, rough_icp_.getFinalTransformation());
    last_refine_score_ = refine_icp_.getFitnessScore();
    // hasConverged 只反映迭代停止；还需通过有限数与 fitness 门限检查。
    if (!refine_icp_.hasConverged() || !std::isfinite(last_refine_score_) ||
        last_refine_score_ > config_.refine_score_thresh)
    {
        return false;
    }

    guess = refine_icp_.getFinalTransformation();
    return guess.allFinite();
}
