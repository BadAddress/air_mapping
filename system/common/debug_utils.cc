//
// Debug utilities implementation
//

#include "common/debug_utils.h"

#include <yaml-cpp/yaml.h>
#include "cyber/common/log.h"

namespace lightning {

DebugConfig DebugUtils::LoadConfig(const std::string& yaml_path) {
    DebugConfig config;
    
    try {
        if (!std::filesystem::exists(yaml_path)) {
            AWARN << "Debug config file not found: " << yaml_path 
                         << ", using default config";
            return config;
        }
        
        YAML::Node yaml = YAML::LoadFile(yaml_path);
        
        if (yaml["debug"]) {
            auto debug_node = yaml["debug"];
            
            if (debug_node["enabled"]) {
                config.enabled = debug_node["enabled"].as<bool>();
            }
            
            if (debug_node["output_path"]) {
                config.output_path = debug_node["output_path"].as<std::string>();
            }
            
            if (debug_node["slam_poses_file"]) {
                config.slam_poses_file = debug_node["slam_poses_file"].as<std::string>();
            }
            
            if (debug_node["localization_poses_file"]) {
                config.localization_poses_file = debug_node["localization_poses_file"].as<std::string>();
            }
        }
        
        AINFO << "Debug config loaded: enabled=" << config.enabled 
                  << ", output_path=" << config.output_path;
                  
    } catch (const std::exception& e) {
        AERROR << "Failed to load debug config: " << e.what();
    }
    
    return config;
}

bool DebugUtils::EnsureDirectoryExists(const std::string& path) {
    try {
        if (!std::filesystem::exists(path)) {
            std::filesystem::create_directories(path);
            AINFO << "Created debug output directory: " << path;
        }
        return true;
    } catch (const std::exception& e) {
        AERROR << "Failed to create directory " << path << ": " << e.what();
        return false;
    }
}

std::string DebugUtils::GenerateDateStampedDir(const std::string& base_path) {
    EnsureDirectoryExists(base_path);

    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm* now_tm = std::localtime(&now_time_t);

    std::ostringstream dir_name;
    dir_name << std::put_time(now_tm, "%Y-%m-%d_%H%M%S");

    std::filesystem::path dated_path = std::filesystem::path(base_path) / dir_name.str();
    EnsureDirectoryExists(dated_path.string());

    // Update "latest" symlink
    std::filesystem::path latest_link = std::filesystem::path(base_path) / "latest";
    try {
        if (std::filesystem::is_symlink(latest_link) || std::filesystem::exists(latest_link)) {
            std::filesystem::remove(latest_link);
        }
        std::filesystem::create_directory_symlink(dated_path, latest_link);
        AINFO << "Debug output: " << dated_path.string()
              << " (latest -> " << dir_name.str() << ")";
    } catch (const std::exception& e) {
        AWARN << "Failed to create latest symlink: " << e.what();
    }

    return dated_path.string();
}

std::string DebugUtils::GetOrCreateDebugOutputDir(const std::string& base_path) {
    std::filesystem::path latest_path = std::filesystem::path(base_path) / "latest";
    if (std::filesystem::exists(latest_path)) {
        try {
            return std::filesystem::canonical(latest_path).string();
        } catch (const std::exception& e) {
            AWARN << "Failed to resolve latest symlink: " << e.what()
                  << ", creating new dated dir";
        }
    }
    return GenerateDateStampedDir(base_path);
}

std::string DebugUtils::GenerateTUMHeader(const std::string& description) {
    // 获取当前时间
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm* now_tm = std::localtime(&now_time_t);
    
    std::ostringstream oss;
    oss << "# TUM trajectory format\n";
    oss << "# " << description << "\n";
    oss << "# Generated at: " << std::put_time(now_tm, "%Y-%m-%d %H:%M:%S") << "\n";
    oss << "# Format: timestamp tx ty tz qx qy qz qw\n";
    oss << "# Coordinate system: world frame\n";
    oss << "#\n";
    
    return oss.str();
}

std::string DebugUtils::SE3ToTUMString(double timestamp, const SE3& pose) {
    const auto& t = pose.translation();
    const auto& q = pose.unit_quaternion();
    
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9);
    oss << timestamp << " ";
    oss << std::setprecision(7);
    oss << t.x() << " " << t.y() << " " << t.z() << " ";
    oss << q.x() << " " << q.y() << " " << q.z() << " " << q.w();
    
    return oss.str();
}

bool DebugUtils::SaveKeyframesToTUM(const std::vector<Keyframe::Ptr>& keyframes,
                                    const std::string& file_path,
                                    bool use_opt_pose) {
    if (keyframes.empty()) {
        AWARN << "No keyframes to save";
        return false;
    }
    
    // 确保目录存在
    std::filesystem::path p(file_path);
    if (!EnsureDirectoryExists(p.parent_path().string())) {
        return false;
    }
    
    std::ofstream file(file_path);
    if (!file.is_open()) {
        AERROR << "Failed to open file for writing: " << file_path;
        return false;
    }
    
    // 写入头部
    std::string description = use_opt_pose ? 
        "SLAM keyframe poses after bundle adjustment (BA)" :
        "SLAM keyframe poses from LIO frontend";
    file << GenerateTUMHeader(description);
    file << "# Total keyframes: " << keyframes.size() << "\n";
    file << "#\n";
    
    // 写入关键帧位姿
    for (const auto& kf : keyframes) {
        if (!kf) continue;
        
        SE3 pose = use_opt_pose ? kf->GetOptPose() : kf->GetLIOPose();
        file << SE3ToTUMString(kf->GetTimestamp(), pose) << "\n";
    }
    
    file.close();
    
    AINFO << "Saved " << keyframes.size() << " keyframe poses to " << file_path 
              << " (use_opt_pose=" << use_opt_pose << ")";
    
    return true;
}

// LocalizationPoseRecorder implementation

LocalizationPoseRecorder::~LocalizationPoseRecorder() {
    Close();
}

bool LocalizationPoseRecorder::Init(const std::string& file_path) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (initialized_) {
        AWARN << "LocalizationPoseRecorder already initialized";
        return true;
    }
    
    // 确保目录存在
    std::filesystem::path p(file_path);
    if (!DebugUtils::EnsureDirectoryExists(p.parent_path().string())) {
        return false;
    }
    
    file_.open(file_path);
    if (!file_.is_open()) {
        AERROR << "Failed to open localization poses file: " << file_path;
        return false;
    }
    
    // 写入头部
    file_ << DebugUtils::GenerateTUMHeader("Real-time localization output poses");
    file_ << "#\n";
    
    initialized_ = true;
    pose_count_ = 0;
    
    AINFO << "LocalizationPoseRecorder initialized: " << file_path;
    
    return true;
}

void LocalizationPoseRecorder::RecordPose(double timestamp, const SE3& pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_ || !file_.is_open()) {
        return;
    }
    
    file_ << DebugUtils::SE3ToTUMString(timestamp, pose) << "\n";
    pose_count_++;
    
    // 每 100 帧刷新一次，确保数据写入
    if (pose_count_ % 100 == 0) {
        file_.flush();
    }
}

void LocalizationPoseRecorder::Close() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (file_.is_open()) {
        file_.flush();
        file_.close();
        AINFO << "LocalizationPoseRecorder closed, total poses: " << pose_count_;
    }
    
    initialized_ = false;
}

// ========== GpsOdomRecorder implementation ==========

GpsOdomRecorder::~GpsOdomRecorder() {
    Close();
}

bool GpsOdomRecorder::Init(const std::string& file_path, const std::string& experiment_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (initialized_) {
        AWARN << "GpsOdomRecorder already initialized";
        return true;
    }
    
    // 确保目录存在
    std::filesystem::path p(file_path);
    if (!DebugUtils::EnsureDirectoryExists(p.parent_path().string())) {
        return false;
    }
    
    file_.open(file_path);
    if (!file_.is_open()) {
        AERROR << "Failed to open GPS odometry file: " << file_path;
        return false;
    }
    
    // 获取当前时间
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm* now_tm = std::localtime(&now_time_t);
    
    // 写入头部信息
    file_ << "# ============================================================\n";
    file_ << "# GPS Odometry (RTK) Ground Truth\n";
    file_ << "# ============================================================\n";
    file_ << "# Experiment: " << (experiment_name.empty() ? "mapping" : experiment_name) << "\n";
    file_ << "# Generated at: " << std::put_time(now_tm, "%Y-%m-%d %H:%M:%S") << "\n";
    file_ << "# \n";
    file_ << "# Data source: /apollo/sensor/gnss/odometry\n";
    file_ << "# Message type: apollo.localization.Gps\n";
    file_ << "# Coordinate system: UTM (meters)\n";
    file_ << "# \n";
    file_ << "# Format: TUM trajectory format\n";
    file_ << "# timestamp tx ty tz qx qy qz qw\n";
    file_ << "# ============================================================\n";
    file_ << "#\n";
    
    initialized_ = true;
    pose_count_ = 0;
    
    AINFO << "[GpsOdomRecorder] Initialized: " << file_path;
    
    return true;
}

void GpsOdomRecorder::RecordPose(double timestamp, double x, double y, double z,
                                  double qx, double qy, double qz, double qw) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_ || !file_.is_open()) {
        return;
    }
    
    file_ << std::fixed << std::setprecision(9) << timestamp << " ";
    file_ << std::setprecision(7) << x << " " << y << " " << z << " ";
    file_ << qx << " " << qy << " " << qz << " " << qw << "\n";
    
    pose_count_++;
    
    // 每 100 帧刷新一次
    if (pose_count_ % 100 == 0) {
        file_.flush();
    }
}

void GpsOdomRecorder::Close() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (file_.is_open()) {
        // 写入统计信息
        file_ << "#\n";
        file_ << "# ============================================================\n";
        file_ << "# Total poses recorded: " << pose_count_ << "\n";
        file_ << "# ============================================================\n";
        
        file_.flush();
        file_.close();
        AINFO << "[GpsOdomRecorder] Closed, total poses: " << pose_count_;
    }
    
    initialized_ = false;
}

}  // namespace lightning

