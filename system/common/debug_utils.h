//
// Debug utilities for air_mapping
// 用于建图和定位模块的 Debug 工具
//

#ifndef LIGHTNING_DEBUG_UTILS_H
#define LIGHTNING_DEBUG_UTILS_H

#include <string>
#include <fstream>
#include <vector>
#include <memory>
#include <mutex>
#include <filesystem>
#include <iomanip>
#include <ctime>
#include <chrono>

#include "common/eigen_types.h"
#include "common/keyframe.h"

namespace lightning {

/**
 * Debug 配置结构
 */
struct DebugConfig {
    bool enabled = false;                                                    // 是否开启 Debug 模式
    std::string output_path = "/apollo_workspace/modules/air_mapping/debug_records";  // 输出路径
    std::string slam_poses_file = "slam_poses_ba.txt";                       // SLAM 位姿文件名
    std::string localization_poses_file = "localization_poses.txt";          // 定位位姿文件名
    std::string lio_poses_file = "lio_poses.txt";                            // LIO 位姿文件名
    std::string dr_poses_file = "dr_poses.txt";                              // DR 位姿文件名
    std::string lidarloc_poses_file = "lidarloc_poses.txt";                  // LidarLoc 位姿文件名
    std::string extrap_poses_file = "extrap_poses.txt";                      // 外推位姿文件名
    
    // 获取完整的 SLAM 位姿文件路径
    std::string GetSlamPosesPath() const {
        return output_path + "/" + slam_poses_file;
    }
    
    // 获取完整的定位位姿文件路径
    std::string GetLocalizationPosesPath() const {
        return output_path + "/" + localization_poses_file;
    }

    std::string GetLioPosesPath() const { return output_path + "/" + lio_poses_file; }
    std::string GetDrPosesPath() const { return output_path + "/" + dr_poses_file; }
    std::string GetLidarLocPosesPath() const { return output_path + "/" + lidarloc_poses_file; }
    std::string GetExtrapPosesPath() const { return output_path + "/" + extrap_poses_file; }
};

/**
 * Debug 工具类
 * 提供 TUM 格式位姿保存等功能
 */
class DebugUtils {
public:
    /**
     * 从 YAML 文件加载 Debug 配置
     * @param yaml_path common_conf.yaml 的路径
     * @return DebugConfig 配置结构
     */
    static DebugConfig LoadConfig(const std::string& yaml_path);
    
    /**
     * 确保输出目录存在
     * @param path 目录路径
     * @return 是否成功
     */
    static bool EnsureDirectoryExists(const std::string& path);
    
    /**
     * 在 base_path 下创建带日期时间戳的子目录，并更新 latest 软链接
     * 例: base_path="/debug_records" -> 创建 "/debug_records/2026-03-10_143052/"
     *     并将 "/debug_records/latest" 指向该目录
     * @param base_path 基础输出路径
     * @return 新创建的带时间戳的完整路径
     */
    static std::string GenerateDateStampedDir(const std::string& base_path);

    /**
     * 获取或创建 Debug 输出目录。若 base_path/latest 已存在（由 map_builder 等先创建），
     * 则复用该目录；否则调用 GenerateDateStampedDir 创建新目录。
     * 用于 slam 等后运行组件与 map_builder 共享同一目录，避免 gps_odom_gt 与 slam_poses 分散。
     * @param base_path 基础输出路径
     * @return 可用的 Debug 输出目录完整路径
     */
    static std::string GetOrCreateDebugOutputDir(const std::string& base_path);
    
    /**
     * 生成 TUM 格式的文件头注释
     * @param description 描述信息
     * @return 头注释字符串
     */
    static std::string GenerateTUMHeader(const std::string& description);
    
    /**
     * 将 SE3 位姿转换为 TUM 格式字符串
     * TUM 格式: timestamp tx ty tz qx qy qz qw
     * @param timestamp 时间戳 (秒)
     * @param pose SE3 位姿
     * @return TUM 格式字符串
     */
    static std::string SE3ToTUMString(double timestamp, const SE3& pose);
    
    /**
     * 保存关键帧序列到 TUM 格式文件 (建图模式用)
     * @param keyframes 关键帧序列
     * @param file_path 输出文件路径
     * @param use_opt_pose 使用优化后的位姿 (true) 还是 LIO 位姿 (false)
     * @return 是否成功
     */
    static bool SaveKeyframesToTUM(const std::vector<Keyframe::Ptr>& keyframes,
                                   const std::string& file_path,
                                   bool use_opt_pose = true);
};

/**
 * 定位结果记录器
 * 用于在定位模式下实时记录位姿到 TUM 文件
 */
class LocalizationPoseRecorder {
public:
    LocalizationPoseRecorder() = default;
    ~LocalizationPoseRecorder();
    
    /**
     * 初始化记录器
     * @param file_path 输出文件路径
     * @return 是否成功
     */
    bool Init(const std::string& file_path);
    
    /**
     * 记录一帧位姿
     * @param timestamp 时间戳 (秒)
     * @param pose SE3 位姿
     */
    void RecordPose(double timestamp, const SE3& pose);
    
    /**
     * 关闭文件
     */
    void Close();
    
    /**
     * 是否已初始化
     */
    bool IsInitialized() const { return initialized_; }
    
private:
    std::mutex mutex_;
    std::ofstream file_;
    bool initialized_ = false;
    size_t pose_count_ = 0;
};

/**
 * GPS Odometry 记录器
 * 用于记录 RTK GPS 输出的位姿 (作为 Ground Truth 参考)
 * 格式: TUM (timestamp tx ty tz qx qy qz qw)
 */
class GpsOdomRecorder {
public:
    GpsOdomRecorder() = default;
    ~GpsOdomRecorder();
    
    /**
     * 初始化记录器
     * @param file_path 输出文件路径
     * @param experiment_name 实验名称 (写入文件头)
     * @return 是否成功
     */
    bool Init(const std::string& file_path, const std::string& experiment_name = "");
    
    /**
     * 记录一帧 GPS 位姿
     * @param timestamp 时间戳 (秒)
     * @param x UTM X 坐标 (米)
     * @param y UTM Y 坐标 (米)
     * @param z 高程 (米)
     * @param qx 四元数 x
     * @param qy 四元数 y
     * @param qz 四元数 z
     * @param qw 四元数 w
     */
    void RecordPose(double timestamp, double x, double y, double z,
                    double qx, double qy, double qz, double qw);
    
    /**
     * 关闭文件
     */
    void Close();
    
    /**
     * 是否已初始化
     */
    bool IsInitialized() const { return initialized_; }
    
    /**
     * 获取已记录的位姿数量
     */
    size_t GetPoseCount() const { return pose_count_; }
    
private:
    std::mutex mutex_;
    std::ofstream file_;
    bool initialized_ = false;
    size_t pose_count_ = 0;
};

}  // namespace lightning

#endif  // LIGHTNING_DEBUG_UTILS_H

