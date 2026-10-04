#include "map_builder.h"
MapBuilder::MapBuilder(Config &config, std::shared_ptr<IESKF> kf) : m_config(config), m_kf(kf)
{
    m_kf->setMaxIter(static_cast<size_t>(config.ieskf_max_iter));
    m_imu_processor = std::make_shared<IMUProcessor>(config, kf);
    m_lidar_processor = std::make_shared<LidarProcessor>(config, kf);
    m_status = BuilderStatus::IMU_INIT;
}

void MapBuilder::process(SyncPackage &package)
{
    // 状态顺序为 IMU 初始化、首帧建图、持续匹配；初始化帧不参与地图更新。
    if (!package.cloud || package.cloud->empty() || package.imus.empty())
        return;
    if (m_status == BuilderStatus::IMU_INIT)
    {
        if (m_imu_processor->initialize(package))
            m_status = BuilderStatus::MAP_INIT;
        return;
    }

    // 先用 IMU 预测并将所有点补偿至扫描末时刻，再用几何残差校正滤波状态。
    m_imu_processor->undistort(package);

    if (m_status == BuilderStatus::MAP_INIT)
    {
        CloudType::Ptr cloud_world = LidarProcessor::transformCloud(package.cloud, m_lidar_processor->r_wl(), m_lidar_processor->t_wl());
        m_lidar_processor->initCloudMap(cloud_world->points);
        m_status = BuilderStatus::MAPPING;
        return;
    }
    
    m_lidar_processor->process(package);
}
