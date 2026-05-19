//
// Created by xiang on 25-5-6.
//

#ifndef LIGHTNING_SLAM_H
#define LIGHTNING_SLAM_H

// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/imu.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <atomic>
#include <memory>
#include <deque>
#include <vector>

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/common_msgs/sensor_msgs/imu.pb.h"
#include "modules/common_msgs/sensor_msgs/gnss_best_pose.pb.h"

// #include "lightning/srv/save_map.hpp"
// #include "livox_ros_driver2/msg/custom_msg.hpp"

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"
#include "common/point_def.h"
#include "common/debug_utils.h"
#include "common/gps_data.h"

namespace lightning {

class LaserMapping;  //  lio frontend
class GpsFusionOptimizer;  // GPS fusion backend optimizer

namespace ui {
class PangolinWindow;
}

namespace g2p5 {
class G2P5;
}

struct MapSaveFilterOptions {
    bool enable = true;
    double voxel_size_m = 0.15;                // meters, default 15 cm
    double chunk_voxel_size_m = 0.15;          // meters, default 15 cm
    bool remove_invalid_points = true;
    bool enable_height_crop = true;
    double min_z_m = -4.0;                     // map-frame z lower bound
    double max_z_m = 4.0;                      // map-frame z upper bound
    bool enable_statistical_outlier_removal = true;
    int sor_mean_k = 20;
    double sor_stddev_mul_thresh = 1.0;
};

/**
 * SLAM 系统调用接口
 */
class SlamSystem {
   public:
    struct Options {
        Options() {}

        bool online_mode_ = true;  // 在线模式，在线模式下会起一些子线程来做异步处理

        bool with_cc_ = true;               // 是否需要带交叉验证
        bool with_gridmap_ = true;          // enable 2D grid map
        bool with_visualization_ = true;    // 3D UI (unused in Apollo)
        bool with_2dvisualization_ = true;  // 2D UI (unused in Apollo)

        bool step_on_kf_ = true;  // 是否在关键帧处暂停p
        
        std::string map_path_ = "./data/new_map/"; // Added map path
    };

    // using SaveMapService = srv::SaveMap;

    SlamSystem(Options options);
    ~SlamSystem();

    /// 初始化
    bool Init(const std::string& yaml_path);

    /// 对外部交互接口
    /// 开始建图，输入地图名称
    void StartSLAM(std::string map_name);

    /// 保存地图，默认保存至./data/地图名/ 下方
    void SaveMap(const std::string& path = "");
    
    /// Request graceful shutdown (called on CTRL+C)
    void RequestShutdown();
    
    /// Check if shutdown is requested
    bool IsShutdownRequested() const { return shutdown_requested_.load(); }

    /// Export LIO keyframes collected by the frontend.
    std::vector<Keyframe::Ptr> GetAllKeyframes() const;

    /// Export paired GPS observations collected during stage processing.
    std::vector<GpsFullObservation> GetGpsFullHistory() const;

    /// Export a preview global map from the current LIO keyframes.
    CloudPtr GetGlobalMapFromKeyframes(const std::vector<Keyframe::Ptr>& keyframes,
                                       bool use_voxel = true,
                                       float res = 0.1) const;

    /// 处理IMU
    void ProcessIMU(const lightning::IMUPtr& imu);

    /// 处理点云
    // void ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);
    void ProcessLidar(const std::shared_ptr<apollo::drivers::PointCloud>& cloud);
    
    // void ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud);

    /// process GPS heading for LIO initialization only
    void ProcessHeading(const HeadingObservation& heading);
    
    /// process GPS position for UTM offset calculation
    void ProcessGPS(const std::shared_ptr<apollo::drivers::gnss::GnssBestPose>& gps);
    
    /// Get paired and processed GPS full observation (for downstream consumers)
    /// Returns false if no valid paired GPS data is available
    bool GetLatestGpsFullObservation(GpsFullObservation& obs) const;
    
    /// Get interpolated GPS observation at a given timestamp (for keyframes)
    /// Returns false if interpolation is not possible or data invalid
    bool GetInterpolatedGpsAt(double timestamp, GpsFullObservation& obs) const;
    
    /// Get GNSS-LiDAR time offset for synchronization
    double GetGnssLidarTimeOffset() const { return gnss_lidar_time_offset_; }

    /// realtime spin (unused in Apollo)
    void Spin();

   private:
    /// ros端保存地图的实现
    // void SaveMap(const SaveMapService::Request::SharedPtr request, SaveMapService::Response::SharedPtr response);

    Options options_;
    std::atomic_bool running_ = false;
    std::atomic_bool shutdown_requested_ = false;  // Graceful shutdown flag
    DebugConfig debug_config_;  // Debug 模式配置
    MapSaveFilterOptions map_save_filter_opts_;

    // rclcpp::Service<SaveMapService>::SharedPtr savemap_service_ = nullptr;

    std::string map_name_;  // 地图名

    std::shared_ptr<LaserMapping> lio_ = nullptr;       // LIO frontend
    // std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;  // ui (not used in Apollo)
    std::shared_ptr<g2p5::G2P5> g2p5_ = nullptr;        // 2D grid map (no OpenCV version)
    std::shared_ptr<GpsFusionOptimizer> gps_optimizer_ = nullptr;  // GPS fusion backend

    Keyframe::Ptr cur_kf_ = nullptr;
    
    /// GPS position tracking for UTM offset
    bool first_gps_received_ = false;
    Vec3d first_gps_utm_ = Vec3d::Zero();
    Vec3d gps_lever_arm_ = Vec3d::Zero();  // GPS antenna to IMU offset in IMU frame
    
    /// Time synchronization
    double gnss_lidar_time_offset_ = 0.0;  // GNSS ahead of LiDAR (seconds)
    
    /// GPS pairing mechanism (Best Pose + Heading)
    struct GpsPairCandidate {
        double timestamp = 0.0;
        
        // Best Pose data
        Vec3d antenna_utm = Vec3d::Zero();
        Vec3d position_std_dev = Vec3d::Zero();
        uint32_t sol_status = 0;
        uint32_t sol_type = 0;
        bool has_position = false;
        
        // Heading data
        double heading_rad = 0.0;
        double pitch_rad = 0.0;
        double heading_std_dev = 0.0;
        double pitch_std_dev = 0.0;
        uint32_t satellite_tracked = 0;
        bool has_heading = false;
        
        bool IsComplete() const { return has_position && has_heading; }
    };
    
    std::deque<GpsPairCandidate> gps_pair_cache_;
    mutable std::mutex mtx_gps_pair_;
    GpsFullObservation latest_gps_full_;  // Latest valid paired GPS data
    bool has_valid_gps_full_ = false;
    
    static constexpr double GPS_TIME_MATCH_TOLERANCE = 0.001;  // 1ms
    static constexpr size_t GPS_PAIR_CACHE_MAX_SIZE = 10;
    static constexpr double GPS_PAIR_TIMEOUT = 1.0;  // 1 second
    
    // GPS history for interpolation at keyframe timestamps
    std::deque<GpsFullObservation> gps_full_history_;
    static constexpr size_t GPS_HISTORY_ONLINE_MAX_SIZE = 2000;
    static constexpr double GPS_INTERP_MAX_GAP = 1.5;      // max allowed time gap for interpolation (seconds)
    
    // GPS attachment density control (sparser GPS keyframes)
    double gps_attach_min_kf_distance_ = 5.0;  // meters
    bool has_last_gps_attach_lio_pos_ = false;
    Vec3d last_gps_attach_lio_pos_ = Vec3d::Zero();
    
    /// Process paired GPS data (called when a complete pair is found)
    void ProcessGpsPair(const GpsPairCandidate& pair);

    /// Attach interpolated GPS observation to a newly created keyframe
    void AttachGpsToKeyframe(const Keyframe::Ptr& kf);
    
    /// Called when LIO creates a new keyframe (for backend optimization)
    void OnKeyframeCreated(const Keyframe::Ptr& kf);

};
}  // namespace lightning

#endif  // LIGHTNING_SLAM_H
