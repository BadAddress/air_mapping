#ifndef FASTER_LIO_POINTCLOUD_PROCESSING_H
#define FASTER_LIO_POINTCLOUD_PROCESSING_H

// #include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common/measure_group.h"
#include "common/point_def.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

// #include "livox_ros_driver2/msg/custom_msg.hpp"

namespace lightning {

enum class LidarType { AVIA = 1, VELO32, OUST64 };

/// 点云处理模式
enum class ProcessMode {
    LOCALIZATION,  // 定位：ring/FOV/圆柱/抽点 + 盲区过滤
    MAPPING        // 建图：仅盲区过滤，保留全部有效点
};

/**
 * point cloud preprocess
 * just unify the point format from livox/velodyne to PCL
 *
 * 预处理程序
 * 主要是对各种不同的雷达处理时间戳差异
 */
class PointCloudPreprocess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointCloudPreprocess() = default;
    ~PointCloudPreprocess() = default;

    /// 统一入口：按 mode 分发到不同 handler
    void Process(const std::shared_ptr<apollo::drivers::PointCloud> &msg,
                 PointCloudType::Ptr &pcl_out,
                 ProcessMode mode = ProcessMode::LOCALIZATION);

    // void Process(const livox_ros_driver2::msg::CustomMsg::SharedPtr &cloud, PointCloudType::Ptr &pcl_out);

    void Set(LidarType lid_type, double bld, int pfilt_num);

    // accessors
    double &Blind() { return blind_; }
    double &MaxDistance() { return max_distance_; }
    double &MaxRadius() { return max_radius_; }      // 圆柱形过滤：最大水平半径
    double &MinZ() { return min_z_; }                // 圆柱形过滤：最小 Z
    double &MaxZ() { return max_z_; }                // 圆柱形过滤：最大 Z
    int &NumScans() { return num_scans_; }
    int &PointFilterNum() { return point_filter_num_; }
    int &MappingPointFilterNum() { return mapping_point_filter_num_; }
    float &TimeScale() { return time_scale_; }
    LidarType GetLidarType() const { return lidar_type_; }
    void SetLidarType(LidarType lt) { lidar_type_ = lt; }
    
    // 扇形过滤（方位角）：只保留前视区域
    bool &FovFilterEnable() { return fov_filter_enable_; }
    double &FovAngleMin() { return fov_angle_min_; }   // 最小方位角（度），如 -90
    double &FovAngleMax() { return fov_angle_max_; }   // 最大方位角（度），如 90
    
    // 扫描线过滤（ring）：只保留指定扫描线范围
    bool &RingFilterEnable() { return ring_filter_enable_; }
    int &RingMin() { return ring_min_; }   // 最小扫描线（包含）
    int &RingMax() { return ring_max_; }   // 最大扫描线（包含）

   private:
    // 定位模式 handler：全过滤（ring/FOV/圆柱/抽点/盲区）
    void ApolloCloudHandlerLoc(const std::shared_ptr<apollo::drivers::PointCloud> &msg);
    
    // 建图模式 handler：仅盲区过滤，保留全部有效点
    void ApolloCloudHandlerMapping(const std::shared_ptr<apollo::drivers::PointCloud> &msg);

    PointCloudType cloud_full_, cloud_out_;

    LidarType lidar_type_ = LidarType::AVIA;
    int point_filter_num_ = 1;
    int mapping_point_filter_num_ = 1;  // 建图模式降采样步进（1=不降采样）
    int num_scans_ = 6;
    double blind_ = 0.01;
    double max_distance_ = 80.0;   // 球形过滤：最大距离（已弃用，保留兼容）
    double max_radius_ = 60.0;     // 圆柱形过滤：最大水平半径 (米)
    double min_z_ = -10.0;         // 圆柱形过滤：最小 Z (米)
    double max_z_ = 20.0;          // 圆柱形过滤：最大 Z (米)
    float time_scale_ = 1e-3;
    bool given_offset_time_ = false;
    
    // 扇形过滤（FOV）：只保留特定方位角范围的点
    bool fov_filter_enable_ = false;   // 是否启用扇形过滤
    double fov_angle_min_ = -90.0;     // 最小方位角（度），前视左边界
    double fov_angle_max_ = 90.0;      // 最大方位角（度），前视右边界
    
    // 扫描线过滤（Ring）：只保留指定扫描线范围（用于 128 线等机械式雷达）
    bool ring_filter_enable_ = false;  // 是否启用扫描线过滤
    int ring_min_ = 0;                 // 最小扫描线（包含），如 108
    int ring_max_ = 127;               // 最大扫描线（包含），如 127（保留顶部 20 线）
    
    // 时间戳自适应（只计算一次，后续复用）
    bool time_offset_initialized_ = false;
    double time_offset_ratio_ = 0.0;  // 时间偏移比例（相对于扫描周期）
    double scan_period_ms_ = 100.0;   // 扫描周期（毫秒）
    bool effective_time_scale_initialized_ = false;
    double effective_time_scale_ = 1e-9;  // 实际生效的点时间缩放（自动判别后缓存）
};
}  // namespace lightning

#endif
