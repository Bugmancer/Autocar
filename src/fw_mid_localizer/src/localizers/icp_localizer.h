#pragma once

#include <limits>
#include <string>

#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

#include "localizers/commons.h"

struct ICPConfig
{
    double refine_scan_resolution = 0.1;
    double refine_map_resolution = 0.1;
    double refine_score_thresh = 0.1;
    int refine_max_iteration = 10;

    double rough_scan_resolution = 0.25;
    double rough_map_resolution = 0.25;
    double rough_score_thresh = 0.2;
    int rough_max_iteration = 5;
};

class ICPLocalizer
{
public:
    explicit ICPLocalizer(const ICPConfig &config);

    bool loadMap(const std::string &path, std::string *error_message = nullptr);
    void setInput(const CloudType::ConstPtr &cloud);
    bool align(M4F &guess);

    CloudType::ConstPtr refineMap() const { return refine_target_; }
    double roughScore() const { return last_rough_score_; }
    double refineScore() const { return last_refine_score_; }

private:
    static CloudType::Ptr downsample(const CloudType::ConstPtr &cloud,
                                     double resolution);

    ICPConfig config_;
    pcl::IterativeClosestPoint<PointType, PointType> refine_icp_;
    pcl::IterativeClosestPoint<PointType, PointType> rough_icp_;
    CloudType::Ptr refine_input_;
    CloudType::Ptr rough_input_;
    CloudType::Ptr refine_target_;
    CloudType::Ptr rough_target_;
    double last_rough_score_ = std::numeric_limits<double>::infinity();
    double last_refine_score_ = std::numeric_limits<double>::infinity();
};
