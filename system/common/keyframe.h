//
// Created by xiang on 25-3-12.
//

#ifndef LIGHTNING_KEYFRAME_H
#define LIGHTNING_KEYFRAME_H

#include "common/eigen_types.h"
#include "common/nav_state.h"
#include "common/point_def.h"
#include "common/std_types.h"

namespace lightning {

/// 关键帧描述
/// NOTE: 在添加后端后，需要加锁
class Keyframe {
   public:
    using Ptr = std::shared_ptr<Keyframe>;

    Keyframe() {}
    Keyframe(unsigned long id, CloudPtr cloud, NavState state)
        : id_(id), cloud_(cloud), state_(state), pose_lio_(state.GetPose()) {
        timestamp_ = state_.timestamp_;
        pose_opt_ = pose_lio_;
    }

    unsigned long GetID() const { return id_; }
    double GetTimestamp() const { return timestamp_; }
    CloudPtr GetCloud() const { return cloud_; }

    SE3 GetLIOPose() {
        UL lock(data_mutex_);
        return pose_lio_;
    }

    void SetLIOPose(const SE3& pose) {
        UL lock(data_mutex_);
        pose_lio_ = pose;

        // also set opt
        pose_opt_ = pose_lio_;
    }

    SE3 GetOptPose() {
        UL lock(data_mutex_);
        return pose_opt_;
    }

    void SetOptPose(const SE3& pose) {
        UL lock(data_mutex_);
        pose_opt_ = pose;
    }

    void SetState(NavState s) {
        UL lock(data_mutex_);
        state_ = s;
    }

    NavState GetState() {
        UL lock(data_mutex_);
        return state_;
    }

    // ========== GPS fusion related data ==========
    
    /// GPS observation data associated with this keyframe
    struct GpsData {
        bool gps_available = false;        // Whether valid GPS exists at this keyframe timestamp
        bool has_gps = false;              // Whether this keyframe is selected as a sparse GPS graph anchor
        Vec3d gps_utm_position = Vec3d::Zero();  // GPS UTM position (antenna/IMU processed as needed)
        Vec3d gps_std_dev = Vec3d::Zero();       // GPS position standard deviation [std_x, std_y, std_z]
        double gps_heading_deg = 0.0;      // GPS heading (degrees)
        double heading_std_deg = 0.0;      // Heading standard deviation (degrees)
        uint32_t sol_type = 0;             // Solution type (50 = NARROW_INT for RTK fixed)
    };
    
    void SetGpsData(const GpsData& gps) {
        UL lock(data_mutex_);
        gps_data_ = gps;
    }
    
    GpsData GetGpsData() {
        UL lock(data_mutex_);
        return gps_data_;
    }
    
    /// Relative motion to previous keyframe (for optimization)
    void SetRelativeMotion(const SE3& relative_motion) {
        UL lock(data_mutex_);
        relative_motion_ = relative_motion;
    }
    
    SE3 GetRelativeMotion() {
        UL lock(data_mutex_);
        return relative_motion_;
    }
    
    /// Covariance associated with this keyframe (for optimization)
    void SetCovariance(const Mat6d& cov) {
        UL lock(data_mutex_);
        covariance_ = cov;
    }
    
    Mat6d GetCovariance() {
        UL lock(data_mutex_);
        return covariance_;
    }

   protected:
    unsigned long id_ = 0;
    double timestamp_ = 0;
    CloudPtr cloud_ = nullptr;  /// 降采样之后的点云
    
    NavState state_;  // 卡尔曼滤波器状态 (必须在 pose_lio_ 之前，与初始化顺序一致)
    
    std::mutex data_mutex_;
    SE3 pose_lio_;  // 前端的pose
    SE3 pose_opt_;  // 后端优化后的pose
    
    // GPS fusion related
    GpsData gps_data_;
    SE3 relative_motion_;
    Mat6d covariance_ = Mat6d::Identity();
};

}  // namespace lightning

#endif  // LIGHTNING_KEYFRAME_H
