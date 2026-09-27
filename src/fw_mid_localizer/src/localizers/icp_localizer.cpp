#include "localizers/icp_localizer.h"

#include <cmath>
#include <sstream>
#include <vector>

#include <pcl/common/io.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>

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

    refine_target_.swap(refine_target);
    rough_target_.swap(rough_target);
    return true;
}

void ICPLocalizer::setInput(const CloudType::ConstPtr &cloud)
{
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
    last_rough_score_ = std::numeric_limits<double>::infinity();
    last_refine_score_ = std::numeric_limits<double>::infinity();
    if (refine_input_->empty() || rough_input_->empty() ||
        refine_target_->empty() || rough_target_->empty())
    {
        return false;
    }

    CloudType aligned_cloud;
    rough_icp_.setMaximumIterations(config_.rough_max_iteration);
    rough_icp_.setInputSource(rough_input_);
    rough_icp_.setInputTarget(rough_target_);
    rough_icp_.align(aligned_cloud, guess);
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
    if (!refine_icp_.hasConverged() || !std::isfinite(last_refine_score_) ||
        last_refine_score_ > config_.refine_score_thresh)
    {
        return false;
    }

    guess = refine_icp_.getFinalTransformation();
    return guess.allFinite();
}
