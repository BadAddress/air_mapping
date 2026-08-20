//
// Created for GPS fusion in map_builder
// GPS observation data structure
//

#ifndef LIGHTNING_GPS_DATA_H
#define LIGHTNING_GPS_DATA_H

#include <cmath>
#include <vector>
#include "common/eigen_types.h"

namespace lightning {

/// GPS Heading 观测数据结构
struct HeadingObservation {
    double timestamp = 0.0;           // 时间戳 (秒)
    double heading = 0.0;             // 航向角 (度)
    double pitch = 0.0;               // 俯仰角 (度)
    double heading_std_dev = 0.0;     // 航向角标准差 (度)
    double pitch_std_dev = 0.0;       // 俯仰角标准差 (度)
    int satellite_tracked = 0;        // 跟踪的卫星数
    bool is_valid = false;            // 是否有效
    double body_x_yaw_sign = 1.0;
    double body_x_yaw_offset_deg = -87.5171;
    
    HeadingObservation() = default;
    
    /// 将航向角转换为 LIO body frame 的 yaw 弧度（用于 Rz 旋转）
    /// 
    /// 新版传感器 heading = 车头方向（Apollo 约定: East=0°, North=90°, 逆时针正）
    /// CorrectedImu body frame: X=右, Y=前, Z=上 (RFU)
    /// ApplyGPSHeadingInit 中 Rz(yaw) 把 body X 轴对齐到 ENU 的 yaw 方向
    /// body X 轴（右侧）在 ENU 中的方向由车型配置给出：
    /// yaw = body_x_yaw_sign * heading + body_x_yaw_offset_deg
    double GetYawRad() const {
        double yaw_rad =
            (body_x_yaw_sign * heading + body_x_yaw_offset_deg) * M_PI / 180.0;
                
        while (yaw_rad > M_PI) yaw_rad -= 2.0 * M_PI;
        while (yaw_rad < -M_PI) yaw_rad += 2.0 * M_PI;
                
        return yaw_rad;
    }
};

/// GPS Position observation (from GNSS Best Pose message)
struct GpsObservation {
    double timestamp = 0.0;
    Vec3d position_utm = Vec3d::Zero();  // UTM coordinates (E, N, U)
    Vec3d std_dev = Vec3d::Zero();       // standard deviation (meters)
    uint32_t sol_status = 0;             // solution status
    uint32_t sol_type = 0;               // solution type
    bool is_valid = false;
};

/// Unified GPS observation (paired Best Pose + Heading with lever arm compensation)
struct GpsFullObservation {
    double timestamp = 0.0;
    
    // Position information (converted)
    Vec3d antenna_utm = Vec3d::Zero();   // Antenna position (UTM)
    Vec3d imu_utm = Vec3d::Zero();       // IMU position (UTM, lever-arm compensated)
    
    // Orientation information
    double heading_rad = 0.0;     // Heading (radians, ENU definition)
    double pitch_rad = 0.0;       // Pitch (radians)
    
    // Best Pose original quality information
    Vec3d position_std_dev = Vec3d::Zero();  // Position std dev (lat, lon, height)
    uint32_t sol_status = 0;                 // Solution status
    uint32_t sol_type = 0;                   // Solution type
    
    // Heading original quality information
    double heading_std_dev = 0.0;   // Heading std dev (degrees)
    double pitch_std_dev = 0.0;     // Pitch std dev (degrees)
    uint32_t satellite_tracked = 0; // Tracked satellite count
    
    // Validity flags
    bool has_position = false;  // Best Pose valid
    bool has_heading = false;   // Heading valid
    bool is_valid = false;      // Overall valid (paired + quality checked)
};

}  // namespace lightning

#endif  // LIGHTNING_GPS_DATA_H
