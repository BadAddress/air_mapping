#include "pointcloud_preprocess.h"
#include <execution>
#include <iostream>
#include <iomanip>
#include <limits>
#include <cmath>
#include <array>

#include "cyber/common/log.h"

constexpr float RAD_TO_DEG = 180.0f / M_PI;

namespace lightning {

void PointCloudPreprocess::Set(LidarType lid_type, double bld, int pfilt_num) {
    lidar_type_ = lid_type;
    blind_ = bld;
    point_filter_num_ = pfilt_num;
}

void PointCloudPreprocess::Process(const std::shared_ptr<apollo::drivers::PointCloud> &msg,
                                   PointCloudType::Ptr &pcl_out,
                                   ProcessMode mode) {
    switch (mode) {
        case ProcessMode::LOCALIZATION:
            ApolloCloudHandlerLoc(msg);
            break;
        case ProcessMode::MAPPING:
            ApolloCloudHandlerMapping(msg);
            break;
    }
    *pcl_out = cloud_out_;
}

// ==================== 建图模式：仅盲区过滤 ====================
void PointCloudPreprocess::ApolloCloudHandlerMapping(const std::shared_ptr<apollo::drivers::PointCloud> &msg) {
    cloud_out_.clear();
    
    const int plsize = msg->point_size();
    const double base_time = msg->measurement_time();
    const float blind_sq = static_cast<float>(blind_ * blind_);
    
    cloud_out_.points.reserve(plsize);

    // 点时间单位自动判别（只执行一次）
    if (!effective_time_scale_initialized_ && plsize > 0) {
        const double ts0 = static_cast<double>(msg->point(0).timestamp());
        const std::array<double, 4> candidates = {
            static_cast<double>(time_scale_), 1e-9, 1e-6, 1e-3
        };
        double best_scale = candidates[0];
        double best_err = std::numeric_limits<double>::max();
        for (double c : candidates) {
            const double t = ts0 * c;
            const double err = std::abs(t - base_time);
            if (err < best_err) {
                best_err = err;
                best_scale = c;
            }
        }
        effective_time_scale_ = best_scale;
        effective_time_scale_initialized_ = true;
        AINFO << "[TIME_SCALE] Configured=" << time_scale_
              << ", effective=" << effective_time_scale_
              << ", first_pt_time=" << (ts0 * effective_time_scale_)
              << ", base_time=" << base_time;
    }
    
    // 时间戳初始化（只计算一次）
    if (!time_offset_initialized_) {
        double min_pt_time = std::numeric_limits<double>::max();
        double max_pt_time = std::numeric_limits<double>::lowest();
        const int sample_step = std::max(1, plsize / 1000);
        for (int i = 0; i < plsize; i += sample_step) {
            const auto& apt = msg->point(i);
            const double pt_time = static_cast<double>(apt.timestamp()) * effective_time_scale_;
            min_pt_time = std::min(min_pt_time, pt_time);
            max_pt_time = std::max(max_pt_time, pt_time);
        }
        scan_period_ms_ = (max_pt_time - min_pt_time) * 1000.0;
        const double time_offset_ms = (min_pt_time - base_time) * 1000.0;
        time_offset_ratio_ = (scan_period_ms_ > 1e-6) ? (time_offset_ms / scan_period_ms_) : 0.0;
        time_offset_initialized_ = true;
        
        std::cout << "\n========== LiDAR Timestamp Analysis (Mapping) ==========\n"
                  << "  scan_period: " << std::fixed << std::setprecision(3) << scan_period_ms_ << " ms\n"
                  << "  time_offset_ratio: " << time_offset_ratio_ << "\n"
                  << "=========================================================\n";
    }
    
    const double lidar_begin_time = base_time + (time_offset_ratio_ * scan_period_ms_) / 1000.0;
    
    const int mapping_stride = std::max(1, mapping_point_filter_num_);
    for (int i = 0; i < plsize; i += mapping_stride) {
        const auto& apt = msg->point(i);
        const float x = apt.x();
        const float y = apt.y();
        const float z = apt.z();
        
        // 仅盲区过滤（去除自身反射点）
        const float dist_sq = x * x + y * y;
        if (dist_sq < blind_sq) {
            continue;
        }
        
        PointType pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        pt.intensity = apt.intensity();
        const double pt_time_sec = static_cast<double>(apt.timestamp()) * effective_time_scale_;
        pt.time = (pt_time_sec - lidar_begin_time) * 1000.0;
        cloud_out_.points.push_back(pt);
    }
    
    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
    cloud_out_.header.stamp = static_cast<uint64_t>(lidar_begin_time * 1e9);
}

// ==================== 定位模式：全过滤 ====================
void PointCloudPreprocess::ApolloCloudHandlerLoc(const std::shared_ptr<apollo::drivers::PointCloud> &msg) {
    cloud_out_.clear();
    
    const int plsize = msg->point_size();
    if (plsize == 0) return;
    
    const double base_time = msg->measurement_time();

    // 点时间单位自动判别（只执行一次）
    if (!effective_time_scale_initialized_) {
        const double ts0 = static_cast<double>(msg->point(0).timestamp());
        const std::array<double, 4> candidates = {
            static_cast<double>(time_scale_), 1e-9, 1e-6, 1e-3
        };
        double best_scale = candidates[0];
        double best_err = std::numeric_limits<double>::max();
        for (double c : candidates) {
            const double t = ts0 * c;
            const double err = std::abs(t - base_time);
            if (err < best_err) {
                best_err = err;
                best_scale = c;
            }
        }
        effective_time_scale_ = best_scale;
        effective_time_scale_initialized_ = true;
        AINFO << "[TIME_SCALE] Configured=" << time_scale_
              << ", effective=" << effective_time_scale_
              << ", first_pt_time=" << (ts0 * effective_time_scale_)
              << ", base_time=" << base_time;
    }
    
    // ---- 预计算循环不变量 ----
    const float blind_sq = static_cast<float>(blind_ * blind_);
    const float max_radius_sq = static_cast<float>(max_radius_ * max_radius_);
    const float min_z = static_cast<float>(min_z_);
    const float max_z = static_cast<float>(max_z_);
    const int filter_num = point_filter_num_;
    
    const bool fov_enable = fov_filter_enable_;
    const float fov_min = static_cast<float>(fov_angle_min_);
    const float fov_max = static_cast<float>(fov_angle_max_);
    const bool fov_normal_range = (fov_min <= fov_max);  // 预计算，避免循环内判断
    
    if (ring_filter_enable_) {
        static bool warned = false;
        if (!warned) {
            AWARN << "Ring filter enabled but PointXYZIT has no ring field, skipping ring filter.";
            warned = true;
        }
    }

    // ---- 第一帧：计算时间偏移（只执行一次）----
    if (!time_offset_initialized_) {
        double min_pt_time = std::numeric_limits<double>::max();
        double max_pt_time = std::numeric_limits<double>::lowest();
        
        const int sample_step = std::max(1, plsize / 1000);
        for (int i = 0; i < plsize; i += sample_step) {
            const auto& apt = msg->point(i);
            const double pt_time = static_cast<double>(apt.timestamp()) * effective_time_scale_;
            min_pt_time = std::min(min_pt_time, pt_time);
            max_pt_time = std::max(max_pt_time, pt_time);
        }
        
        scan_period_ms_ = (max_pt_time - min_pt_time) * 1000.0;
        const double time_offset_ms = (min_pt_time - base_time) * 1000.0;
        time_offset_ratio_ = (scan_period_ms_ > 1e-6) ? (time_offset_ms / scan_period_ms_) : 0.0;
        time_offset_initialized_ = true;
        
        std::cout << "\n========== LiDAR Timestamp Analysis ==========\n"
                  << "  scan_period: " << std::fixed << std::setprecision(3) << scan_period_ms_ << " ms\n"
                  << "  time_offset_ratio: " << time_offset_ratio_ << "\n"
                  << "==============================================\n";
    }
    
    const double lidar_begin_time = base_time + (time_offset_ratio_ * scan_period_ms_) / 1000.0;
    // 预计算：将 begin_time 转为毫秒，循环内省掉一次 * 1000.0
    // 原式: (apt.timestamp()*time_scale - lidar_begin_time) * 1e3
    //     = apt.timestamp() * time_scale * 1e3 - lidar_begin_time * 1e3
    const double begin_time_ms = lidar_begin_time * 1e3;
    
    // ---- 预分配最大可能输出大小，用索引直写替代 push_back ----
    // push_back 每次检查 size/capacity + 可能触发重分配
    // resize + 索引写入 = 零开销追加
    const int max_output = (plsize + filter_num - 1) / filter_num;
    cloud_out_.points.resize(max_output);
    int out_count = 0;
    
    // ---- 主处理循环 ----
    // 过滤顺序按 成本/剔除率 排列：
    //   1. Cylinder — 2 mul + 1 add + 3 float 比较（中等）
    //   2. FOV     — atan2 + 1 mul + 比较（最贵，按需启用）
    for (int i = 0; i < plsize; i += filter_num) {
        const auto& apt = msg->point(i);
        
        // Filter 1: Ring — PointXYZIT 不含 ring 字段，跳过
        // 如需 ring 过滤，请使用包含 ring 字段的点云消息类型
        
        const float x = apt.x();
        const float y = apt.y();
        const float z = apt.z();
        
        // Filter 2: 圆柱形过滤（盲区 + 最大半径 + Z 范围）
        const float h_dist_sq = x * x + y * y;
        if (h_dist_sq < blind_sq || h_dist_sq > max_radius_sq ||
            z < min_z || z > max_z) {
            continue;
        }
        
        // Filter 3: FOV 扇形过滤（atan2 最贵，放最后）
        if (fov_enable) {
            const float azimuth = std::atan2(y, x) * RAD_TO_DEG;
            if (fov_normal_range) {
                if (azimuth < fov_min || azimuth > fov_max) continue;
            } else {
                if (azimuth < fov_min && azimuth > fov_max) continue;
            }
        }
        
        // 索引直写：无 size 检查、无重分配开销
        auto& pt = cloud_out_.points[out_count++];
        pt.x = x;
        pt.y = y;
        pt.z = z;
        pt.intensity = apt.intensity();
        pt.time = static_cast<float>(
            static_cast<double>(apt.timestamp()) * effective_time_scale_ * 1e3 - begin_time_ms);
    }
    
    // 截断到实际大小（仅修改 size，无内存操作）
    cloud_out_.points.resize(out_count);
    cloud_out_.width = out_count;
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
    cloud_out_.header.stamp = static_cast<uint64_t>(lidar_begin_time * 1e9);
}

}  // namespace lightning
