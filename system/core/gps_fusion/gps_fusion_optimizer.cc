//
// GPS Fusion Optimizer Implementation
//

#include "gps_fusion_optimizer.h"
#include "cyber/common/log.h"
#include "common/eigen_types.h"
#include "core/miao/core/graph/optimizer.h"
#include "core/miao/core/opti_algo/algo_select.h"
#include "core/miao/core/robust_kernel/huber.h"
#include "core/miao/core/robust_kernel/cauchy.h"
#include "core/miao/core/types/vertex_se3.h"
#include "core/miao/core/types/edge_se3.h"
#include "core/miao/core/types/edge_se3_prior.h"
#include "core/miao/core/graph/base_unary_edge.h"
#include "core/miao/core/graph/base_binary_edge.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <unordered_map>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include "core/lightning_math.hpp"

namespace lightning {

// ========== Custom GPS Edges (defined inline) ==========

/// GPS Position Prior Edge: constrains XY(Z) position
class EdgeGpsPosition : public miao::BaseUnaryEdge<3, Vec3d, miao::VertexSE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void ComputeError() override {
        SE3 pose = ((miao::VertexSE3*)(vertices_[0]))->Estimate();
        error_ = pose.translation() - measurement_;
    }
};

/// GPS Heading Prior Edge: constrains yaw angle only
class EdgeGpsHeading : public miao::BaseUnaryEdge<1, double, miao::VertexSE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void ComputeError() override {
        SE3 pose = ((miao::VertexSE3*)(vertices_[0]))->Estimate();
        
        // Extract yaw from SE3 rotation matrix (ZYX Euler: yaw = atan2(R21, R11))
        Mat3d R = pose.rotationMatrix();
        double yaw = std::atan2(R(1, 0), R(0, 0));
        
        // Angle difference, normalized to [-π, π]
        double diff = yaw - measurement_;
        while (diff > M_PI) diff -= 2.0 * M_PI;
        while (diff < -M_PI) diff += 2.0 * M_PI;
        
        error_(0) = diff;
    }
};

/// Height Smooth Edge: constrains height difference between adjacent keyframes
/// 高度平滑约束：假设相邻帧之间的 Z 变化应该接近 0（平坦地面假设）
class EdgeHeightSmooth : public miao::BaseBinaryEdge<1, double, miao::VertexSE3, miao::VertexSE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void ComputeError() override {
        SE3 pose1 = ((miao::VertexSE3*)(vertices_[0]))->Estimate();
        SE3 pose2 = ((miao::VertexSE3*)(vertices_[1]))->Estimate();
        
        double z1 = pose1.translation().z();
        double z2 = pose2.translation().z();
        
        // 误差 = 实际 Z 差 - 期望 Z 差（measurement_ 通常为 0）
        error_(0) = (z2 - z1) - measurement_;
    }
};

/// GPS Height Prior Edge: constrains Z coordinate only (for NARROW_INT RTK)
/// GPS 高程先验约束：仅约束 Z 坐标（使用 NARROW_INT 固定解，插值/延拓）
class EdgeGpsHeight : public miao::BaseUnaryEdge<1, double, miao::VertexSE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void ComputeError() override {
        SE3 pose = ((miao::VertexSE3*)(vertices_[0]))->Estimate();
        double z = pose.translation().z();
        // 误差 = 当前 Z - 参考 Z (measurement_)
        error_(0) = z - measurement_;
    }
};

struct GpsSegment {
    std::vector<size_t> keyframe_indices;
    std::vector<double> cumulative_distance_m;
};

struct OutageSegment {
    size_t start_idx = 0;
    size_t end_idx = 0;
    int left_anchor_idx = -1;
    int right_anchor_idx = -1;
    double path_length_m = 0.0;
};

struct HeightTrendTarget {
    size_t keyframe_index = 0;
    double trend_height = 0.0;
    double raw_height = 0.0;
    double path_distance_m = 0.0;
};

struct DockingConstraint {
    int support_idx = -1;
    size_t outage_idx = 0;
    SE3 measurement_support_to_outage;
    double ndt_score = 0.0;
    bool valid = false;
};

[[maybe_unused]] const char* LoopConstraintOriginTag(LoopConstraintOrigin origin) {
    switch (origin) {
        case LoopConstraintOrigin::kAsync:
            return "async";
        case LoopConstraintOrigin::kFinalCatchup:
            return "final_catchup";
        case LoopConstraintOrigin::kOutageSupportRematch:
            return "outage_recheck";
    }
    return "unknown";
}

struct OutageBlockMapConstraint {
    size_t center_idx = 0;
    size_t window_start_idx = 0;
    size_t window_end_idx = 0;
    SE3 matched_world_pose;
    double ndt_score = 0.0;
    double initial_z_offset_m = 0.0;
    Vec3d delta_translation = Vec3d::Zero();
    double delta_rotation_deg = 0.0;
    int source_keyframes = 0;
    int target_keyframes = 0;
    bool valid = false;
};

struct OutageDockingResult {
    size_t representative_idx = 0;
    int representative_vertex_id = -1;
    SE3 representative_pose;
    std::vector<SE3> local_pose_from_representative;
    DockingConstraint left_constraint;
    DockingConstraint right_constraint;
};

struct RawGpsHeightSample {
    double timestamp = 0.0;
    double path_distance_m = 0.0;
    double raw_height = 0.0;
    double sigma_z = 0.0;
};

std::vector<GpsSegment> BuildGpsSegments(const std::vector<Keyframe::Ptr>& keyframes,
                                         double segment_break_distance_m,
                                         int min_segment_points) {
    std::vector<GpsSegment> segments;
    GpsSegment current_segment;
    Vec3d prev_lio_position = Vec3d::Zero();
    bool has_prev = false;

    for (size_t i = 0; i < keyframes.size(); ++i) {
        const auto gps_data = keyframes[i]->GetGpsData();
        if (!gps_data.has_gps) {
            continue;
        }

        const Vec3d lio_position = keyframes[i]->GetLIOPose().translation();
        if (!lio_position.allFinite()) {
            continue;
        }

        if (current_segment.keyframe_indices.empty()) {
            current_segment.keyframe_indices.push_back(i);
            current_segment.cumulative_distance_m.push_back(0.0);
            prev_lio_position = lio_position;
            has_prev = true;
            continue;
        }

        const double delta_distance = has_prev ? (lio_position - prev_lio_position).norm() : 0.0;
        if (std::isfinite(delta_distance) && delta_distance > segment_break_distance_m) {
            if (current_segment.keyframe_indices.size() >= static_cast<size_t>(std::max(min_segment_points, 1))) {
                segments.push_back(current_segment);
            }
            current_segment = GpsSegment();
            current_segment.keyframe_indices.push_back(i);
            current_segment.cumulative_distance_m.push_back(0.0);
        } else {
            const double cumulative =
                current_segment.cumulative_distance_m.back() + (std::isfinite(delta_distance) ? delta_distance : 0.0);
            current_segment.keyframe_indices.push_back(i);
            current_segment.cumulative_distance_m.push_back(cumulative);
        }

        prev_lio_position = lio_position;
        has_prev = true;
    }

    if (current_segment.keyframe_indices.size() >= static_cast<size_t>(std::max(min_segment_points, 1))) {
        segments.push_back(current_segment);
    }
    return segments;
}

double ComputePathLength(const std::vector<Keyframe::Ptr>& keyframes, size_t start_idx, size_t end_idx) {
    if (start_idx >= keyframes.size() || end_idx >= keyframes.size() || end_idx <= start_idx) {
        return 0.0;
    }

    double path_length_m = 0.0;
    for (size_t i = start_idx + 1; i <= end_idx; ++i) {
        const Vec3d prev = keyframes[i - 1]->GetLIOPose().translation();
        const Vec3d curr = keyframes[i]->GetLIOPose().translation();
        if (!prev.allFinite() || !curr.allFinite()) {
            continue;
        }
        path_length_m += (curr - prev).norm();
    }
    return path_length_m;
}

std::vector<OutageSegment> BuildOutageSegments(const std::vector<Keyframe::Ptr>& keyframes,
                                               int min_keyframes,
                                               double min_length_m) {
    std::vector<OutageSegment> segments;
    if (keyframes.empty()) {
        return segments;
    }

    const size_t min_kfs = static_cast<size_t>(std::max(min_keyframes, 1));
    int last_gps_idx = -1;
    size_t i = 0;
    while (i < keyframes.size()) {
        while (i < keyframes.size() && keyframes[i]->GetGpsData().has_gps) {
            last_gps_idx = static_cast<int>(i);
            ++i;
        }
        if (i >= keyframes.size()) {
            break;
        }

        const size_t start_idx = i;
        while (i < keyframes.size() && !keyframes[i]->GetGpsData().has_gps) {
            ++i;
        }
        const size_t end_idx = i - 1;
        OutageSegment outage_segment;
        outage_segment.start_idx = start_idx;
        outage_segment.end_idx = end_idx;
        outage_segment.left_anchor_idx = last_gps_idx;
        outage_segment.right_anchor_idx = (i < keyframes.size()) ? static_cast<int>(i) : -1;
        outage_segment.path_length_m = ComputePathLength(keyframes, outage_segment.start_idx, outage_segment.end_idx);

        const size_t outage_keyframes = outage_segment.end_idx - outage_segment.start_idx + 1;
        if (outage_keyframes < min_kfs || outage_segment.path_length_m < min_length_m) {
            continue;
        }
        segments.push_back(outage_segment);
    }

    return segments;
}

std::vector<OutageSegment> SplitOutageSegmentsIntoBlocks(const std::vector<Keyframe::Ptr>& keyframes,
                                                         const std::vector<OutageSegment>& outage_segments,
                                                         int min_keyframes,
                                                         double max_block_length_m) {
    if (max_block_length_m <= 1e-6) {
        return outage_segments;
    }

    std::vector<OutageSegment> blocks;
    const size_t min_kfs = static_cast<size_t>(std::max(min_keyframes, 1));
    for (const auto& segment : outage_segments) {
        if (segment.path_length_m <= max_block_length_m) {
            blocks.push_back(segment);
            continue;
        }

        std::vector<OutageSegment> segment_blocks;
        size_t block_start = segment.start_idx;
        double block_length = 0.0;
        for (size_t i = segment.start_idx + 1; i <= segment.end_idx; ++i) {
            const Vec3d prev = keyframes[i - 1]->GetLIOPose().translation();
            const Vec3d curr = keyframes[i]->GetLIOPose().translation();
            if (prev.allFinite() && curr.allFinite()) {
                block_length += (curr - prev).norm();
            }

            const size_t block_keyframes = i - block_start + 1;
            const size_t remaining_keyframes = segment.end_idx - i;
            if (block_length < max_block_length_m || block_keyframes < min_kfs || remaining_keyframes < min_kfs) {
                continue;
            }

            OutageSegment block = segment;
            block.start_idx = block_start;
            block.end_idx = i;
            block.path_length_m = ComputePathLength(keyframes, block.start_idx, block.end_idx);
            segment_blocks.push_back(block);

            block_start = i + 1;
            block_length = 0.0;
        }

        if (block_start <= segment.end_idx) {
            OutageSegment tail_block = segment;
            tail_block.start_idx = block_start;
            tail_block.end_idx = segment.end_idx;
            tail_block.path_length_m = ComputePathLength(keyframes, tail_block.start_idx, tail_block.end_idx);
            segment_blocks.push_back(tail_block);
        }

        if (segment_blocks.size() >= 2) {
            auto& tail_block = segment_blocks.back();
            const size_t tail_keyframes = tail_block.end_idx - tail_block.start_idx + 1;
            if (tail_keyframes < min_kfs) {
                segment_blocks[segment_blocks.size() - 2].end_idx = tail_block.end_idx;
                segment_blocks[segment_blocks.size() - 2].path_length_m =
                    ComputePathLength(keyframes,
                                      segment_blocks[segment_blocks.size() - 2].start_idx,
                                      segment_blocks[segment_blocks.size() - 2].end_idx);
                segment_blocks.pop_back();
            }
        }

        blocks.insert(blocks.end(), segment_blocks.begin(), segment_blocks.end());
    }

    return blocks;
}

double ComputeBoundaryRampScale(const GpsSegment& segment, size_t local_index, double ramp_distance_m) {
    if (local_index >= segment.cumulative_distance_m.size()) {
        return 0.0;
    }
    if (ramp_distance_m <= 1e-6 || segment.cumulative_distance_m.empty()) {
        return 1.0;
    }

    const double distance_from_start = segment.cumulative_distance_m[local_index];
    const double distance_to_end = segment.cumulative_distance_m.back() - segment.cumulative_distance_m[local_index];
    const double boundary_distance = std::min(distance_from_start, distance_to_end);
    return std::clamp(boundary_distance / ramp_distance_m, 0.0, 1.0);
}

double ComputeBoundaryRampScale(double path_distance_m,
                                double segment_start_distance_m,
                                double segment_end_distance_m,
                                double ramp_distance_m) {
    if (ramp_distance_m <= 1e-6) {
        return 1.0;
    }

    const double distance_from_start = path_distance_m - segment_start_distance_m;
    const double distance_to_end = segment_end_distance_m - path_distance_m;
    const double boundary_distance = std::min(distance_from_start, distance_to_end);
    return std::clamp(boundary_distance / ramp_distance_m, 0.0, 1.0);
}

SE3 IdentityPose() {
    return SE3(SO3::exp(Vec3d::Zero()), Vec3d::Zero());
}

double NormalizeAngleRad(double angle_rad) {
    math::KeepAngleInPI(angle_rad);
    return angle_rad;
}

double InterpolateAngleRad(double angle0_rad, double angle1_rad, double alpha) {
    const double alpha_clamped = std::clamp(alpha, 0.0, 1.0);
    double delta = angle1_rad - angle0_rad;
    math::KeepAngleInPI(delta);
    return NormalizeAngleRad(angle0_rad + alpha_clamped * delta);
}

double ComputeOutageProgress(const std::vector<double>& keyframe_path_distances,
                             const OutageSegment& segment,
                             size_t keyframe_index) {
    if (segment.start_idx >= keyframe_path_distances.size() ||
        segment.end_idx >= keyframe_path_distances.size() ||
        keyframe_index >= keyframe_path_distances.size()) {
        return 0.0;
    }

    const double start_distance = keyframe_path_distances[segment.start_idx];
    const double end_distance = keyframe_path_distances[segment.end_idx];
    const double curr_distance = keyframe_path_distances[keyframe_index];
    const double total_distance = end_distance - start_distance;
    if (total_distance <= 1e-6) {
        return 0.0;
    }
    return std::clamp((curr_distance - start_distance) / total_distance, 0.0, 1.0);
}

SE3 ProjectPoseToPlanarXyYaw(const SE3& pose) {
    PoseRPYD planar = math::SE3ToRollPitchYaw(pose);
    planar.z = 0.0;
    planar.roll = 0.0;
    planar.pitch = 0.0;
    return math::XYZRPYToSE3(planar);
}

SE3 ProjectRelativeMotionToPlanarXyYaw(const SE3& relative_motion) {
    PoseRPYD planar_rel = math::SE3ToRollPitchYaw(relative_motion);
    planar_rel.z = 0.0;
    planar_rel.roll = 0.0;
    planar_rel.pitch = 0.0;
    planar_rel.x = relative_motion.translation().x();
    planar_rel.y = relative_motion.translation().y();
    return math::XYZRPYToSE3(planar_rel);
}

PoseRPYD BridgeOutagePoseZRollPitch(const SE3& planar_pose,
                                    bool has_left_anchor,
                                    const PoseRPYD& left_anchor_rpy,
                                    bool has_right_anchor,
                                    const PoseRPYD& right_anchor_rpy,
                                    double progress) {
    PoseRPYD bridged_pose = math::SE3ToRollPitchYaw(planar_pose);
    if (has_left_anchor && has_right_anchor) {
        bridged_pose.z = left_anchor_rpy.z + progress * (right_anchor_rpy.z - left_anchor_rpy.z);
        bridged_pose.roll = InterpolateAngleRad(left_anchor_rpy.roll, right_anchor_rpy.roll, progress);
        bridged_pose.pitch = InterpolateAngleRad(left_anchor_rpy.pitch, right_anchor_rpy.pitch, progress);
    } else if (has_left_anchor) {
        bridged_pose.z = left_anchor_rpy.z;
        bridged_pose.roll = left_anchor_rpy.roll;
        bridged_pose.pitch = left_anchor_rpy.pitch;
    } else if (has_right_anchor) {
        bridged_pose.z = right_anchor_rpy.z;
        bridged_pose.roll = right_anchor_rpy.roll;
        bridged_pose.pitch = right_anchor_rpy.pitch;
    }
    return bridged_pose;
}

SE3 AdjustMeasurementForRepresentative(const SE3& measurement_support_to_anchor,
                                       const SE3& local_pose_rep_to_anchor) {
    return measurement_support_to_anchor * local_pose_rep_to_anchor.inverse();
}

Mat6d BuildSe3Information(double translation_sigma_m, double rotation_sigma_rad) {
    Mat6d info = Mat6d::Identity();
    const double trans_var = std::max(translation_sigma_m * translation_sigma_m, 1e-6);
    const double rot_var = std::max(rotation_sigma_rad * rotation_sigma_rad, 1e-6);
    info.block<3, 3>(0, 0) *= 1.0 / trans_var;
    info.block<3, 3>(3, 3) *= 1.0 / rot_var;
    return info;
}

uint64_t MakeLoopConstraintKey(size_t idx0, size_t idx1) {
    const uint64_t lo = static_cast<uint64_t>(std::min(idx0, idx1));
    const uint64_t hi = static_cast<uint64_t>(std::max(idx0, idx1));
    return (lo << 32) | hi;
}

std::vector<bool> BuildSegmentMask(size_t num_keyframes, const std::vector<OutageSegment>& outage_segments) {
    std::vector<bool> mask(num_keyframes, false);
    for (const auto& segment : outage_segments) {
        for (size_t idx = segment.start_idx; idx <= segment.end_idx && idx < num_keyframes; ++idx) {
            mask[idx] = true;
        }
    }
    return mask;
}

bool IsValidKeyframeIndex(int idx, size_t total_size) {
    return idx >= 0 && static_cast<size_t>(idx) < total_size;
}

CloudPtr GetWorldCloudForKeyframe(const std::vector<Keyframe::Ptr>& keyframes,
                                  size_t keyframe_idx,
                                  const Eigen::Matrix4d& T_imu_lidar,
                                  std::unordered_map<size_t, CloudPtr>* world_cloud_cache = nullptr) {
    if (keyframe_idx >= keyframes.size()) {
        return CloudPtr(new PointCloudType);
    }

    if (world_cloud_cache) {
        auto it = world_cloud_cache->find(keyframe_idx);
        if (it != world_cloud_cache->end()) {
            return it->second;
        }
    }

    CloudPtr cloud = keyframes[keyframe_idx]->GetCloud();
    if (!cloud || cloud->empty()) {
        return CloudPtr(new PointCloudType);
    }

    const Eigen::Matrix4d T_world_lidar = keyframes[keyframe_idx]->GetOptPose().matrix() * T_imu_lidar;
    CloudPtr transformed(new PointCloudType);
    pcl::transformPointCloud(*cloud, *transformed, T_world_lidar);
    if (world_cloud_cache) {
        (*world_cloud_cache)[keyframe_idx] = transformed;
    }
    return transformed;
}

CloudPtr BuildWorldSubmap(const std::vector<Keyframe::Ptr>& keyframes,
                         const std::vector<bool>& include_mask,
                         size_t center_idx,
                         int half_range,
                         int step,
                         const Eigen::Matrix4d& T_imu_lidar,
                         std::unordered_map<size_t, CloudPtr>* world_cloud_cache = nullptr) {
    CloudPtr submap(new PointCloudType);
    if (center_idx >= keyframes.size() || include_mask.size() != keyframes.size()) {
        return submap;
    }

    const int stride = std::max(step, 1);
    const int start = std::max(0, static_cast<int>(center_idx) - std::max(half_range, 0));
    const int end = std::min(static_cast<int>(keyframes.size()) - 1,
                             static_cast<int>(center_idx) + std::max(half_range, 0));
    for (int idx = start; idx <= end; idx += stride) {
        if (!include_mask[idx]) {
            continue;
        }
        CloudPtr transformed = GetWorldCloudForKeyframe(
            keyframes, static_cast<size_t>(idx), T_imu_lidar, world_cloud_cache);
        if (!transformed || transformed->empty()) {
            continue;
        }
        *submap += *transformed;
    }
    return submap;
}

CloudPtr BuildWorldSubmapFromIndices(const std::vector<Keyframe::Ptr>& keyframes,
                                     const std::vector<size_t>& indices,
                                     const Eigen::Matrix4d& T_imu_lidar,
                                     std::unordered_map<size_t, CloudPtr>* world_cloud_cache = nullptr,
                                     int* included_keyframes = nullptr) {
    CloudPtr submap(new PointCloudType);
    int used_keyframes = 0;
    for (size_t idx : indices) {
        CloudPtr transformed = GetWorldCloudForKeyframe(keyframes, idx, T_imu_lidar, world_cloud_cache);
        if (!transformed || transformed->empty()) {
            continue;
        }
        *submap += *transformed;
        ++used_keyframes;
    }
    if (included_keyframes) {
        *included_keyframes = used_keyframes;
    }
    return submap;
}

CloudPtr BuildWorldSubmapByRadius(const std::vector<Keyframe::Ptr>& keyframes,
                                  const std::vector<bool>& include_mask,
                                  const Vec3d& center_position,
                                  double radius_xy_m,
                                  int sample_step,
                                  const Eigen::Matrix4d& T_imu_lidar,
                                  std::unordered_map<size_t, CloudPtr>* world_cloud_cache = nullptr,
                                  int* included_keyframes = nullptr) {
    CloudPtr submap(new PointCloudType);
    if (include_mask.size() != keyframes.size() || !center_position.allFinite()) {
        return submap;
    }

    const double radius_sq = std::max(radius_xy_m, 1.0) * std::max(radius_xy_m, 1.0);
    const size_t stride = static_cast<size_t>(std::max(sample_step, 1));
    int used_keyframes = 0;
    for (size_t idx = 0; idx < keyframes.size(); idx += stride) {
        if (!include_mask[idx]) {
            continue;
        }
        const Vec3d keyframe_position = keyframes[idx]->GetOptPose().translation();
        if (!keyframe_position.allFinite()) {
            continue;
        }
        const double dist_xy_sq = (keyframe_position.head<2>() - center_position.head<2>()).squaredNorm();
        if (!std::isfinite(dist_xy_sq) || dist_xy_sq > radius_sq) {
            continue;
        }

        CloudPtr transformed = GetWorldCloudForKeyframe(keyframes, idx, T_imu_lidar, world_cloud_cache);
        if (!transformed || transformed->empty()) {
            continue;
        }
        *submap += *transformed;
        ++used_keyframes;
    }

    if (included_keyframes) {
        *included_keyframes = used_keyframes;
    }
    return submap;
}

std::vector<size_t> BuildOutageBlockWindowCenters(const OutageSegment& segment,
                                                  const std::vector<double>& keyframe_path_distances,
                                                  double spacing_m) {
    std::vector<size_t> centers;
    if (segment.end_idx >= keyframe_path_distances.size() || segment.start_idx > segment.end_idx) {
        return centers;
    }

    centers.push_back(segment.start_idx);
    const double min_spacing_m = std::max(spacing_m, 1.0);
    double last_center_distance = keyframe_path_distances[segment.start_idx];
    for (size_t idx = segment.start_idx + 1; idx < segment.end_idx; ++idx) {
        if (keyframe_path_distances[idx] - last_center_distance >= min_spacing_m) {
            centers.push_back(idx);
            last_center_distance = keyframe_path_distances[idx];
        }
    }

    if (centers.back() != segment.end_idx) {
        if (keyframe_path_distances[segment.end_idx] - keyframe_path_distances[centers.back()] >=
            std::max(min_spacing_m * 0.5, 1.0)) {
            centers.push_back(segment.end_idx);
        } else {
            centers.back() = segment.end_idx;
        }
    }
    return centers;
}

std::vector<size_t> CollectOutageWindowIndices(const OutageSegment& segment,
                                               const std::vector<double>& keyframe_path_distances,
                                               size_t center_idx,
                                               double radius_m) {
    std::vector<size_t> indices;
    if (segment.end_idx >= keyframe_path_distances.size() || center_idx >= keyframe_path_distances.size()
        || segment.start_idx > segment.end_idx || center_idx < segment.start_idx || center_idx > segment.end_idx) {
        return indices;
    }

    const double center_distance = keyframe_path_distances[center_idx];
    const double max_radius_m = std::max(radius_m, 1.0);
    for (size_t idx = segment.start_idx; idx <= segment.end_idx; ++idx) {
        if (std::abs(keyframe_path_distances[idx] - center_distance) <= max_radius_m) {
            indices.push_back(idx);
        }
    }

    if (indices.empty()) {
        indices.push_back(center_idx);
    } else if (std::find(indices.begin(), indices.end(), center_idx) == indices.end()) {
        indices.push_back(center_idx);
        std::sort(indices.begin(), indices.end());
    }
    return indices;
}

bool RunNdtResolutionSchedule(const CloudPtr& target_world,
                              const CloudPtr& source_world,
                              const std::vector<double>& resolutions,
                              int max_iterations,
                              Eigen::Matrix4f* T_align,
                              double* ndt_score) {
    if (!T_align || !ndt_score || !target_world || !source_world
        || target_world->empty() || source_world->empty()) {
        return false;
    }

    if (!T_align->allFinite()) {
        *T_align = Eigen::Matrix4f::Identity();
    }
    *ndt_score = 0.0;
    for (double resolution : resolutions) {
        const float voxel_size = static_cast<float>(resolution * 0.1);
        CloudPtr target_ds = math::VoxelGrid(target_world, voxel_size);
        CloudPtr source_ds = math::VoxelGrid(source_world, voxel_size);
        if (!target_ds || !source_ds || target_ds->empty() || source_ds->empty()) {
            continue;
        }

        pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        ndt.setTransformationEpsilon(0.05);
        ndt.setStepSize(0.7);
        ndt.setMaximumIterations(std::max(max_iterations, 1));
        ndt.setResolution(static_cast<float>(resolution));
        ndt.setInputTarget(target_ds);
        ndt.setInputSource(source_ds);

        CloudPtr output(new PointCloudType);
        ndt.align(*output, *T_align);
        *T_align = ndt.getFinalTransformation();
        *ndt_score = ndt.getTransformationProbability();
    }

    return std::isfinite(*ndt_score);
}

bool RunMultiResolutionNdt(const CloudPtr& target_world,
                           const CloudPtr& source_world,
                           double score_threshold,
                           int max_iterations,
                           Eigen::Matrix4f* T_align,
                           double* ndt_score) {
    static const std::vector<double> kNdtResolutions = {10.0, 5.0, 2.0, 1.0};
    if (!RunNdtResolutionSchedule(
            target_world, source_world, kNdtResolutions, max_iterations, T_align, ndt_score)) {
        return false;
    }
    return std::isfinite(*ndt_score) && *ndt_score >= score_threshold;
}

bool RunCoarseToFineNdtWithZOffsets(const CloudPtr& target_world,
                                    const CloudPtr& source_world,
                                    double max_abs_z_search_m,
                                    double coarse_z_step_m,
                                    double score_threshold,
                                    int max_iterations,
                                    Eigen::Matrix4f* T_align,
                                    double* ndt_score,
                                    double* best_initial_z_offset_m) {
    if (!T_align || !ndt_score || !target_world || !source_world
        || target_world->empty() || source_world->empty()) {
        return false;
    }

    const Eigen::Matrix4f base_guess = T_align->allFinite() ? *T_align : Eigen::Matrix4f::Identity();
    const double z_step_m = std::max(coarse_z_step_m, 0.5);
    const int z_step_count = static_cast<int>(std::ceil(std::max(max_abs_z_search_m, 0.0) / z_step_m));

    std::vector<double> z_offsets_m;
    z_offsets_m.reserve(static_cast<size_t>(z_step_count) * 2 + 1);
    z_offsets_m.push_back(0.0);
    for (int step = 1; step <= z_step_count; ++step) {
        const double z_offset = step * z_step_m;
        z_offsets_m.push_back(-z_offset);
        z_offsets_m.push_back(z_offset);
    }

    struct NdtCandidate {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        double ndt_score = 0.0;
        double initial_z_offset_m = 0.0;
    };

    static const std::vector<double> kCoarseResolutions = {20.0, 10.0, 5.0};
    static const std::vector<double> kFineResolutions = {5.0, 2.0, 1.0};
    const int coarse_iterations = std::max(10, max_iterations / 2);

    std::vector<NdtCandidate> coarse_candidates;
    coarse_candidates.reserve(z_offsets_m.size());
    for (double z_offset_m : z_offsets_m) {
        Eigen::Matrix4f coarse_guess = base_guess;
        coarse_guess(2, 3) += static_cast<float>(z_offset_m);

        double coarse_score = 0.0;
        if (!RunNdtResolutionSchedule(
                target_world, source_world, kCoarseResolutions, coarse_iterations, &coarse_guess, &coarse_score)) {
            continue;
        }

        coarse_candidates.push_back(NdtCandidate{coarse_guess, coarse_score, z_offset_m});
    }

    if (coarse_candidates.empty()) {
        return false;
    }

    std::sort(
        coarse_candidates.begin(), coarse_candidates.end(),
        [](const NdtCandidate& lhs, const NdtCandidate& rhs) { return lhs.ndt_score > rhs.ndt_score; });

    const size_t refine_count = std::min<size_t>(3, coarse_candidates.size());
    bool found_best = false;
    double best_score = 0.0;
    double best_z_offset_m = 0.0;
    Eigen::Matrix4f best_transform = base_guess;
    for (size_t idx = 0; idx < refine_count; ++idx) {
        Eigen::Matrix4f refined_guess = coarse_candidates[idx].transform;
        double refined_score = coarse_candidates[idx].ndt_score;
        if (!RunNdtResolutionSchedule(
                target_world, source_world, kFineResolutions, max_iterations, &refined_guess, &refined_score)) {
            continue;
        }

        if (!found_best || refined_score > best_score) {
            found_best = true;
            best_score = refined_score;
            best_transform = refined_guess;
            best_z_offset_m = coarse_candidates[idx].initial_z_offset_m;
        }
    }

    if (!found_best || !std::isfinite(best_score) || best_score < score_threshold) {
        return false;
    }

    *T_align = best_transform;
    *ndt_score = best_score;
    if (best_initial_z_offset_m) {
        *best_initial_z_offset_m = best_z_offset_m;
    }
    return true;
}

std::vector<double> BuildKeyframePathDistances(const std::vector<Keyframe::Ptr>& keyframes) {
    std::vector<double> distances(keyframes.size(), 0.0);
    for (size_t i = 1; i < keyframes.size(); ++i) {
        const Vec3d prev = keyframes[i - 1]->GetLIOPose().translation();
        const Vec3d curr = keyframes[i]->GetLIOPose().translation();
        if (!prev.allFinite() || !curr.allFinite()) {
            distances[i] = distances[i - 1];
            continue;
        }
        distances[i] = distances[i - 1] + (curr - prev).norm();
    }
    return distances;
}

bool ProjectTimestampToKeyframePathDistance(double timestamp,
                                            const std::vector<Keyframe::Ptr>& keyframes,
                                            const std::vector<double>& keyframe_path_distances,
                                            double* path_distance_m) {
    if (!path_distance_m || keyframes.size() < 2 || keyframes.size() != keyframe_path_distances.size()) {
        return false;
    }

    if (timestamp < keyframes.front()->GetTimestamp() || timestamp > keyframes.back()->GetTimestamp()) {
        return false;
    }

    auto upper = std::lower_bound(
        keyframes.begin(), keyframes.end(), timestamp,
        [](const Keyframe::Ptr& kf, double ts) { return kf->GetTimestamp() < ts; });
    if (upper == keyframes.end()) {
        *path_distance_m = keyframe_path_distances.back();
        return true;
    }

    const size_t upper_idx = static_cast<size_t>(std::distance(keyframes.begin(), upper));
    if (std::abs((*upper)->GetTimestamp() - timestamp) < 1e-6 || upper_idx == 0) {
        *path_distance_m = keyframe_path_distances[upper_idx];
        return true;
    }

    const size_t lower_idx = upper_idx - 1;
    const double t0 = keyframes[lower_idx]->GetTimestamp();
    const double t1 = keyframes[upper_idx]->GetTimestamp();
    if (!std::isfinite(t0) || !std::isfinite(t1) || t1 <= t0) {
        return false;
    }

    const double alpha = std::clamp((timestamp - t0) / (t1 - t0), 0.0, 1.0);
    *path_distance_m =
        keyframe_path_distances[lower_idx]
        + alpha * (keyframe_path_distances[upper_idx] - keyframe_path_distances[lower_idx]);
    return std::isfinite(*path_distance_m);
}

std::vector<std::vector<RawGpsHeightSample>> BuildRawGpsHeightSegments(
    const std::vector<GpsFullObservation>& raw_gps_history,
    const std::vector<Keyframe::Ptr>& keyframes,
    const std::vector<double>& keyframe_path_distances,
    double utm_to_lio_offset_z,
    double segment_break_distance_m,
    double min_std_z) {
    std::vector<std::vector<RawGpsHeightSample>> segments;
    std::vector<RawGpsHeightSample> current_segment;

    for (const auto& obs : raw_gps_history) {
        if (!obs.is_valid || !obs.has_position) {
            continue;
        }

        double path_distance_m = 0.0;
        if (!ProjectTimestampToKeyframePathDistance(
                obs.timestamp, keyframes, keyframe_path_distances, &path_distance_m)) {
            continue;
        }

        const double sigma_z = std::max(obs.position_std_dev.z(), min_std_z);
        const double raw_height = obs.imu_utm.z() - utm_to_lio_offset_z;
        if (!std::isfinite(path_distance_m) || !std::isfinite(raw_height) || !std::isfinite(sigma_z)) {
            continue;
        }

        RawGpsHeightSample sample;
        sample.timestamp = obs.timestamp;
        sample.path_distance_m = path_distance_m;
        sample.raw_height = raw_height;
        sample.sigma_z = sigma_z;

        if (current_segment.empty()) {
            current_segment.push_back(sample);
            continue;
        }

        const double delta_distance = sample.path_distance_m - current_segment.back().path_distance_m;
        if (!std::isfinite(delta_distance) || delta_distance <= 0.0 || delta_distance > segment_break_distance_m) {
            if (!current_segment.empty()) {
                segments.push_back(current_segment);
            }
            current_segment.clear();
            current_segment.push_back(sample);
            continue;
        }

        current_segment.push_back(sample);
    }

    if (!current_segment.empty()) {
        segments.push_back(current_segment);
    }
    return segments;
}

bool InterpolateTrendAtPathDistance(const std::vector<double>& x,
                                    const std::vector<double>& fitted_z,
                                    const std::vector<double>& raw_z,
                                    double query_x,
                                    double* trend_z,
                                    double* raw_z_nearest) {
    if (!trend_z || x.empty() || x.size() != fitted_z.size() || x.size() != raw_z.size()) {
        return false;
    }

    if (query_x <= x.front()) {
        *trend_z = fitted_z.front();
        if (raw_z_nearest) {
            *raw_z_nearest = raw_z.front();
        }
        return std::isfinite(*trend_z);
    }
    if (query_x >= x.back()) {
        *trend_z = fitted_z.back();
        if (raw_z_nearest) {
            *raw_z_nearest = raw_z.back();
        }
        return std::isfinite(*trend_z);
    }

    auto upper = std::lower_bound(x.begin(), x.end(), query_x);
    if (upper == x.end()) {
        *trend_z = fitted_z.back();
        if (raw_z_nearest) {
            *raw_z_nearest = raw_z.back();
        }
        return std::isfinite(*trend_z);
    }

    const size_t upper_idx = static_cast<size_t>(std::distance(x.begin(), upper));
    if (*upper == query_x || upper_idx == 0) {
        *trend_z = fitted_z[upper_idx];
        if (raw_z_nearest) {
            *raw_z_nearest = raw_z[upper_idx];
        }
        return std::isfinite(*trend_z);
    }

    const size_t lower_idx = upper_idx - 1;
    const double x0 = x[lower_idx];
    const double x1 = x[upper_idx];
    const double alpha = (query_x - x0) / std::max(x1 - x0, 1e-6);
    *trend_z = fitted_z[lower_idx] + alpha * (fitted_z[upper_idx] - fitted_z[lower_idx]);

    if (raw_z_nearest) {
        *raw_z_nearest = (std::abs(query_x - x0) <= std::abs(x1 - query_x))
                             ? raw_z[lower_idx]
                             : raw_z[upper_idx];
    }
    return std::isfinite(*trend_z);
}

bool FitSmoothingSplineLikeHeights(const std::vector<double>& x,
                                   const std::vector<double>& z,
                                   const std::vector<double>& std_z,
                                   double min_std_z,
                                   double smoothing_lambda,
                                   std::vector<double>* fitted_z) {
    if (!fitted_z || x.size() != z.size() || z.size() != std_z.size() || z.empty()) {
        return false;
    }

    const size_t n = z.size();
    fitted_z->assign(n, 0.0);
    if (n == 1) {
        (*fitted_z)[0] = z[0];
        return std::isfinite(z[0]);
    }

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(n), static_cast<Eigen::Index>(n));
    Eigen::VectorXd b = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(n));

    for (size_t i = 0; i < n; ++i) {
        const double sigma = std::max(std_z[i], min_std_z);
        if (!std::isfinite(z[i]) || !std::isfinite(sigma) || sigma <= 0.0) {
            return false;
        }
        const double w = 1.0 / (sigma * sigma);
        A(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(i)) += w;
        b(static_cast<Eigen::Index>(i)) += w * z[i];
    }

    if (n >= 3 && smoothing_lambda > 0.0) {
        for (size_t i = 1; i + 1 < n; ++i) {
            const double h0 = std::max(x[i] - x[i - 1], 1e-3);
            const double h1 = std::max(x[i + 1] - x[i], 1e-3);
            Eigen::Vector3d c;
            c << 2.0 / (h0 * (h0 + h1)), -2.0 / (h0 * h1), 2.0 / (h1 * (h0 + h1));
            const Eigen::Matrix3d penalty = smoothing_lambda * (c * c.transpose());

            const Eigen::Index row = static_cast<Eigen::Index>(i - 1);
            A.block<3, 3>(row, row) += penalty;
        }
    }

    A += Eigen::MatrixXd::Identity(static_cast<Eigen::Index>(n), static_cast<Eigen::Index>(n)) * 1e-9;
    Eigen::LDLT<Eigen::MatrixXd> ldlt(A);
    if (ldlt.info() != Eigen::Success) {
        return false;
    }

    const Eigen::VectorXd solution = ldlt.solve(b);
    if (ldlt.info() != Eigen::Success) {
        return false;
    }
    for (size_t i = 0; i < n; ++i) {
        (*fitted_z)[i] = solution(static_cast<Eigen::Index>(i));
        if (!std::isfinite((*fitted_z)[i])) {
            return false;
        }
    }
    return true;
}

// ========== GpsFusionOptimizer Implementation ==========

GpsFusionOptimizer::GpsFusionOptimizer(const GpsFusionOptimizerOptions& opts)
    : options_(opts) {
    AINFO << "[GpsFusionOptimizer] Initialized — weights: LIO=" << options_.weight_lio_relative
          << ", GPS_pos=" << options_.weight_gps_position
          << ", GPS_height=" << options_.weight_gps_height
          << ", height_smooth=" << options_.weight_height_smooth;
}

GpsFusionOptimizer::~GpsFusionOptimizer() {
    StopLoopWorker();
}

void GpsFusionOptimizer::SetLoopClosureOptions(const LoopClosureOptions& opts) {
    loop_opts_ = opts;
    loop_opts_.loop_kf_gap = std::max(loop_opts_.loop_kf_gap, 1);
    loop_opts_.min_id_interval = std::max(loop_opts_.min_id_interval, 0);
    loop_opts_.closest_id_threshold =
        std::max(loop_opts_.closest_id_threshold, std::max(loop_opts_.min_keyframe_gap, 1));
    loop_opts_.history_submap_half_range = std::max(loop_opts_.history_submap_half_range, 1);
    loop_opts_.history_submap_step = std::max(loop_opts_.history_submap_step, 1);
    loop_opts_.icp_max_iterations = std::max(loop_opts_.icp_max_iterations, 1);
    if (!loop_opts_.enable) {
        StopLoopWorker();
    }
}

int GpsFusionOptimizer::GetTotalKeyframes() const {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    return static_cast<int>(keyframes_.size());
}

void GpsFusionOptimizer::AddKeyframe(const Keyframe::Ptr& kf) {
    if (!kf) {
        return;
    }

    size_t query_idx = 0;
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        keyframes_.push_back(kf);
        query_idx = keyframes_.size() - 1;
    }

    StartLoopWorkerIfNeeded();
    if (loop_opts_.enable && loop_worker_started_.load()) {
        loop_kf_thread_.AddMessage(LoopQueryTask{query_idx, false});
    }
}

void GpsFusionOptimizer::SetRawGpsHistory(const std::vector<GpsFullObservation>& gps_history) {
    std::lock_guard<std::mutex> lock(raw_gps_mutex_);
    raw_gps_history_ = gps_history;
}

void GpsFusionOptimizer::FinalOptimize() {
    EnqueuePendingLoopQueriesBeforeFinalOptimize();
    StopLoopWorker();
    AINFO << "[GpsFusionOptimizer] Running final batch optimization...";
    Optimize(true);
}

void GpsFusionOptimizer::StartLoopWorkerIfNeeded() {
    if (!loop_opts_.enable || loop_worker_started_.load()) {
        return;
    }

    loop_kf_thread_.SetProcFunc([this](const LoopQueryTask& task) {
        ProcessLoopClosureQuery(task.query_idx, task.force_check);
    });
    loop_kf_thread_.SetName("gps_fusion_loop_match");
    loop_kf_thread_.Start();
    last_loop_query_idx_ = std::numeric_limits<size_t>::max();
    loop_worker_started_.store(true);
    AINFO << "[LOOP_ASYNC] Started async loop-matching worker"
          << ", loop_kf_gap=" << loop_opts_.loop_kf_gap
          << ", closest_id_th=" << loop_opts_.closest_id_threshold
          << ", min_id_interval=" << loop_opts_.min_id_interval
          << ", history_half_range=" << loop_opts_.history_submap_half_range
          << ", history_step=" << loop_opts_.history_submap_step;
}

void GpsFusionOptimizer::StopLoopWorker() {
    if (!loop_worker_started_.exchange(false)) {
        return;
    }

    AINFO << "[LOOP_ASYNC] Flushing pending loop-matching work before optimization";
    loop_kf_thread_.Quit();
    AINFO << "[LOOP_ASYNC] Loop-matching worker stopped";
}

std::vector<GpsFusionOptimizer::PrecomputedLoopConstraint> GpsFusionOptimizer::GetLoopConstraintsSnapshot() const {
    std::lock_guard<std::mutex> lock(loop_constraints_mutex_);
    return loop_constraints_;
}

void GpsFusionOptimizer::EnqueuePendingLoopQueriesBeforeFinalOptimize() {
    if (!loop_opts_.enable) {
        return;
    }

    size_t keyframe_count = 0;
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        keyframe_count = keyframes_.size();
    }
    if (keyframe_count == 0) {
        return;
    }

    const size_t start_idx =
        (last_loop_query_idx_ == std::numeric_limits<size_t>::max()) ? size_t(0) : (last_loop_query_idx_ + 1);
    if (start_idx >= keyframe_count) {
        AINFO << "[LOOP_ASYNC] No pending loop queries before final optimization";
        return;
    }

    StartLoopWorkerIfNeeded();
    if (!loop_worker_started_.load()) {
        return;
    }

    AINFO << "[LOOP_ASYNC] Enqueue final catch-up loop queries: [" << start_idx
          << ", " << (keyframe_count - 1) << "]";
    for (size_t query_idx = start_idx; query_idx < keyframe_count; ++query_idx) {
        loop_kf_thread_.AddMessage(LoopQueryTask{query_idx, true});
    }
}

void GpsFusionOptimizer::ProcessLoopClosureQuery(size_t query_idx, bool force_check) {
    if (!loop_opts_.enable) {
        return;
    }

    std::vector<Keyframe::Ptr> keyframes_snapshot;
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        keyframes_snapshot = keyframes_;
    }
    if (query_idx >= keyframes_snapshot.size() || keyframes_snapshot.empty()) {
        return;
    }

    if (last_loop_query_idx_ != std::numeric_limits<size_t>::max() && query_idx <= last_loop_query_idx_) {
        return;
    }
    if (!force_check &&
        last_loop_query_idx_ != std::numeric_limits<size_t>::max() &&
        (query_idx - last_loop_query_idx_) <= static_cast<size_t>(loop_opts_.loop_kf_gap)) {
        return;
    }

    const Keyframe::Ptr& query_kf = keyframes_snapshot[query_idx];
    if (!query_kf) {
        return;
    }

    const Vec3d query_pos = query_kf->GetOptPose().translation();
    if (!query_pos.allFinite()) {
        return;
    }

    std::vector<size_t> candidate_indices;
    candidate_indices.reserve(16);
    const int closest_id_th = std::max(loop_opts_.closest_id_threshold, std::max(loop_opts_.min_keyframe_gap, 1));
    size_t last_candidate_idx = std::numeric_limits<size_t>::max();
    for (size_t hist_idx = 0; hist_idx < query_idx; ++hist_idx) {
        if (last_candidate_idx != std::numeric_limits<size_t>::max() &&
            std::abs(static_cast<int>(hist_idx) - static_cast<int>(last_candidate_idx)) <= loop_opts_.min_id_interval) {
            continue;
        }
        if (static_cast<int>(query_idx - hist_idx) < closest_id_th) {
            break;
        }

        const Vec3d hist_pos = keyframes_snapshot[hist_idx]->GetOptPose().translation();
        if (!hist_pos.allFinite()) {
            continue;
        }
        const double dist_xy = (hist_pos.head<2>() - query_pos.head<2>()).norm();
        if (!std::isfinite(dist_xy) || dist_xy > loop_opts_.search_radius) {
            continue;
        }
        candidate_indices.push_back(hist_idx);
        last_candidate_idx = hist_idx;
    }

    if (candidate_indices.empty()) {
        last_loop_query_idx_ = query_idx;
        return;
    }
    last_loop_query_idx_ = query_idx;

    NavState ref_state0 = keyframes_snapshot.front()->GetState();
    Eigen::Matrix4d T_imu_lidar = Eigen::Matrix4d::Identity();
    T_imu_lidar.block<3, 3>(0, 0) = ref_state0.offset_R_lidar_.matrix();
    T_imu_lidar.block<3, 1>(0, 3) = ref_state0.offset_t_lidar_;

    std::vector<bool> include_all(keyframes_snapshot.size(), true);
    std::unordered_map<size_t, CloudPtr> world_cloud_cache;
    const CloudPtr source_local = query_kf->GetCloud();
    if (!source_local || source_local->empty()) {
        return;
    }

    const LoopConstraintOrigin origin =
        force_check ? LoopConstraintOrigin::kFinalCatchup : LoopConstraintOrigin::kAsync;
    int accepted_count = 0;
    for (size_t hist_idx : candidate_indices) {
        const CloudPtr target_submap = BuildWorldSubmap(
            keyframes_snapshot,
            include_all,
            hist_idx,
            loop_opts_.history_submap_half_range,
            loop_opts_.history_submap_step,
            T_imu_lidar,
            &world_cloud_cache);
        if (!target_submap || target_submap->empty()) {
            continue;
        }

        Eigen::Matrix4f T_align = query_kf->GetOptPose().matrix().cast<float>();
        double ndt_score = 0.0;
        if (!RunMultiResolutionNdt(
                target_submap,
                source_local,
                loop_opts_.icp_fitness_threshold,
                loop_opts_.icp_max_iterations,
                &T_align,
                &ndt_score)) {
            continue;
        }

        const Eigen::Matrix4d T_query_world = T_align.cast<double>();
        const SE3 query_world_pose(
            SO3::fitToSO3(T_query_world.block<3, 3>(0, 0)),
            T_query_world.block<3, 1>(0, 3));
        const SE3 target_pose = keyframes_snapshot[hist_idx]->GetOptPose();

        PrecomputedLoopConstraint constraint;
        constraint.target_idx = hist_idx;
        constraint.source_idx = query_idx;
        constraint.measurement_target_to_source = target_pose.inverse() * query_world_pose;
        constraint.ndt_score = ndt_score;
        constraint.origin = origin;

        const uint64_t key = MakeLoopConstraintKey(hist_idx, query_idx);
        bool inserted = false;
        {
            std::lock_guard<std::mutex> lock(loop_constraints_mutex_);
            if (loop_constraint_keys_.insert(key).second) {
                loop_constraints_.push_back(constraint);
                inserted = true;
            }
        }
        if (!inserted) {
            continue;
        }

        ++accepted_count;
        AINFO << "[" << (force_check ? "LOOP_FINAL_CATCHUP" : "LOOP_ASYNC") << "] Accepted ("
              << hist_idx << "," << query_idx << ")"
              << ": ndt_score=" << std::fixed << std::setprecision(4) << ndt_score;
    }

    AINFO << "[" << (force_check ? "LOOP_FINAL_CATCHUP" : "LOOP_ASYNC") << "] Query " << query_idx
          << ": coarse_candidates=" << candidate_indices.size()
          << ", accepted=" << accepted_count;
}

void GpsFusionOptimizer::Optimize(bool is_final) {
    std::unique_lock<std::mutex> keyframes_lock(keyframes_mutex_);
    if (keyframes_.empty()) {
        AWARN << "[GpsFusionOptimizer] No keyframes to optimize";
        return;
    }

    is_optimizing_.store(true);
    AINFO << "[GpsFusionOptimizer] Optimizing " << keyframes_.size() << " keyframes"
          << (is_final ? " (FINAL)" : "");

    if (!has_utm_offset_) {
        for (size_t i = 0; i < keyframes_.size(); i++) {
            auto gps_data = keyframes_[i]->GetGpsData();
            if (gps_data.has_gps) {
                Vec3d gps_utm_pos = gps_data.gps_utm_position;
                Vec3d lio_pos = keyframes_[i]->GetLIOPose().translation();
                utm_to_lio_offset_ = gps_utm_pos - lio_pos;
                has_utm_offset_ = true;

                AINFO << "[GpsFusionOptimizer] UTM-to-LIO offset computed ONCE from KF " << i << " (LIO pose)";
                AINFO << "  GPS_UTM: " << gps_utm_pos.transpose();
                AINFO << "  LIO_pos: " << lio_pos.transpose();
                AINFO << "  Offset:  " << utm_to_lio_offset_.transpose();
                break;
            }
        }
    }

    if (!has_utm_offset_) {
        AWARN << "[GpsFusionOptimizer] No GPS data available, skipping GPS constraints";
    }

    const std::vector<double> keyframe_path_distances = BuildKeyframePathDistances(keyframes_);

    const std::vector<OutageSegment> outage_segments = options_.enable_outage_rigid_segments
                                                           ? BuildOutageSegments(
                                                                 keyframes_,
                                                                 options_.outage_rigid_min_keyframes,
                                                                 options_.outage_rigid_min_length_m)
                                                           : std::vector<OutageSegment>();
    const std::vector<bool> outage_mask = BuildSegmentMask(keyframes_.size(), outage_segments);
    std::vector<bool> support_mask(keyframes_.size(), true);
    for (size_t i = 0; i < keyframes_.size(); ++i) {
        support_mask[i] = !outage_mask[i];
    }

    const int support_keyframe_count = static_cast<int>(std::count(support_mask.begin(), support_mask.end(), true));
    const int outage_keyframe_count = static_cast<int>(std::count(outage_mask.begin(), outage_mask.end(), true));
    AINFO << "[OUTAGE] segments=" << outage_segments.size()
          << ", support_keyframes=" << support_keyframe_count
          << ", outage_keyframes=" << outage_keyframe_count
          << ", overlap_kf=" << options_.outage_overlap_keyframes;
    for (size_t seg_idx = 0; seg_idx < outage_segments.size(); ++seg_idx) {
        const auto& segment = outage_segments[seg_idx];
        AINFO << "[OUTAGE] Segment " << seg_idx
              << ": support_left=" << segment.left_anchor_idx
              << ", outage=[" << segment.start_idx << "," << segment.end_idx << "]"
              << ", support_right=" << segment.right_anchor_idx
              << ", length=" << std::fixed << std::setprecision(1) << segment.path_length_m << "m";
    }

    NavState ref_state0 = keyframes_[0]->GetState();
    Eigen::Matrix4d T_imu_lidar = Eigen::Matrix4d::Identity();
    T_imu_lidar.block<3, 3>(0, 0) = ref_state0.offset_R_lidar_.matrix();
    T_imu_lidar.block<3, 1>(0, 3) = ref_state0.offset_t_lidar_;

    auto add_lio_relative_edge = [&](const auto& optimizer,
                                     const std::shared_ptr<miao::VertexSE3>& from_vertex,
                                     const std::shared_ptr<miao::VertexSE3>& to_vertex,
                                     const SE3& relative_motion,
                                     const Mat6d& covariance,
                                     int edge_id) -> bool {
        if (!from_vertex || !to_vertex || !covariance.allFinite()) {
            return false;
        }

        auto edge = std::make_shared<miao::EdgeSE3>();
        edge->SetId(edge_id);
        edge->SetVertex(0, from_vertex);
        edge->SetVertex(1, to_vertex);
        edge->SetMeasurement(relative_motion);

        Mat6d information = options_.weight_lio_relative * covariance.inverse();
        information(2, 2) *= 0.1;
        edge->SetInformation(information);

        auto huber_kernel = std::make_shared<miao::RobustKernelHuber>();
        huber_kernel->SetDelta(options_.huber_lio_delta);
        edge->SetRobustKernel(huber_kernel);
        optimizer->AddEdge(edge);
        return true;
    };

    auto run_optimizer = [&](const auto& optimizer, const std::string& stage_tag, int max_iterations) -> bool {
        optimizer->InitializeOptimization();
        optimizer->ComputeActiveErrors();
        const double chi2_before = optimizer->ActiveChi2();
        if (!std::isfinite(chi2_before)) {
            AWARN << "[" << stage_tag << "] chi2_before is NaN/Inf, skip optimization";
            return false;
        }

        const int iterations = optimizer->Optimize(std::max(max_iterations, 1));
        optimizer->ComputeActiveErrors();
        const double chi2_after = optimizer->ActiveChi2();
        if (!std::isfinite(chi2_after)) {
            AWARN << "[" << stage_tag << "] chi2_after is NaN/Inf, skip applying poses";
            return false;
        }
        if (iterations <= 0) {
            AWARN << "[" << stage_tag << "] Solver failed or terminated early";
            return false;
        }

        const double chi2_reduction =
            (chi2_before > 1e-12) ? ((chi2_before - chi2_after) / chi2_before * 100.0) : 0.0;
        AINFO << "[" << stage_tag << "] Optimization completed:"
              << " iterations=" << iterations
              << ", chi2=" << chi2_before << " -> " << chi2_after
              << " (" << chi2_reduction << "% reduction)";
        return true;
    };

    auto log_loop_constraint_snapshot =
        [&](const std::vector<PrecomputedLoopConstraint>& constraints, const std::string& log_tag) {
            int async_count = 0;
            int final_catchup_count = 0;
            int outage_recheck_count = 0;
            for (const auto& constraint : constraints) {
                switch (constraint.origin) {
                    case LoopConstraintOrigin::kAsync:
                        ++async_count;
                        break;
                    case LoopConstraintOrigin::kFinalCatchup:
                        ++final_catchup_count;
                        break;
                    case LoopConstraintOrigin::kOutageSupportRematch:
                        ++outage_recheck_count;
                        break;
                }
            }
            AINFO << "[" << log_tag << "] cached_constraints=" << constraints.size()
                  << ", async=" << async_count
                  << ", final_catchup=" << final_catchup_count
                  << ", outage_recheck=" << outage_recheck_count;
        };

    auto add_cached_loop_edges =
        [&](const auto& optimizer,
            const std::vector<std::shared_ptr<miao::VertexSE3>>& vertices,
            const std::vector<PrecomputedLoopConstraint>& cached_loop_constraints,
            const std::vector<bool>* include_mask,
            const std::vector<bool>* outage_mask_for_stats,
            int edge_id_base,
            const std::string& log_tag) -> int {
            if (!loop_opts_.enable || cached_loop_constraints.empty()) {
                return 0;
            }

            const double info_scale = std::max(loop_opts_.info_scale, 1e-6) / 100.0;
            int loop_edge_count = 0;
            int async_edge_count = 0;
            int final_catchup_edge_count = 0;
            int outage_recheck_edge_count = 0;
            int support_support_edge_count = 0;
            int support_outage_edge_count = 0;
            int outage_outage_edge_count = 0;
            for (const auto& constraint : cached_loop_constraints) {
                if (constraint.target_idx >= vertices.size() || constraint.source_idx >= vertices.size()) {
                    continue;
                }
                if (include_mask &&
                    ((!(*include_mask)[constraint.target_idx]) || (!(*include_mask)[constraint.source_idx]))) {
                    continue;
                }
                if (!vertices[constraint.target_idx] || !vertices[constraint.source_idx] ||
                    vertices[constraint.target_idx].get() == vertices[constraint.source_idx].get()) {
                    continue;
                }

                auto edge = std::make_shared<miao::EdgeSE3>();
                edge->SetId(edge_id_base + loop_edge_count);
                edge->SetVertex(0, vertices[constraint.target_idx]);
                edge->SetVertex(1, vertices[constraint.source_idx]);
                edge->SetMeasurement(constraint.measurement_target_to_source);
                edge->SetInformation(BuildSe3Information(0.2, 3.0 * M_PI / 180.0) * info_scale);

                auto cauchy = std::make_shared<miao::RobustKernelCauchy>();
                cauchy->SetDelta(1.0);
                edge->SetRobustKernel(cauchy);
                optimizer->AddEdge(edge);
                ++loop_edge_count;

                switch (constraint.origin) {
                    case LoopConstraintOrigin::kAsync:
                        ++async_edge_count;
                        break;
                    case LoopConstraintOrigin::kFinalCatchup:
                        ++final_catchup_edge_count;
                        break;
                    case LoopConstraintOrigin::kOutageSupportRematch:
                        ++outage_recheck_edge_count;
                        break;
                }

                if (outage_mask_for_stats &&
                    constraint.target_idx < outage_mask_for_stats->size() &&
                    constraint.source_idx < outage_mask_for_stats->size()) {
                    const bool target_is_outage = (*outage_mask_for_stats)[constraint.target_idx];
                    const bool source_is_outage = (*outage_mask_for_stats)[constraint.source_idx];
                    if (target_is_outage && source_is_outage) {
                        ++outage_outage_edge_count;
                    } else if (target_is_outage || source_is_outage) {
                        ++support_outage_edge_count;
                    } else {
                        ++support_support_edge_count;
                    }
                }
            }

            AINFO << "[" << log_tag << "] cached_constraints=" << cached_loop_constraints.size()
                  << ", edges=" << loop_edge_count
                  << ", async=" << async_edge_count
                  << ", final_catchup=" << final_catchup_edge_count
                  << ", outage_recheck=" << outage_recheck_edge_count
                  << ", support_support=" << support_support_edge_count
                  << ", support_outage=" << support_outage_edge_count
                  << ", outage_outage=" << outage_outage_edge_count;
            return loop_edge_count;
        };

    // ==================== Round 1: optimize support / backbone only ====================
    int round1_lio_edge_count = 0;
    int round1_height_smooth_edge_count = 0;
    int round1_gps_pos_edge_count = 0;
    int round1_gps_heading_edge_count = 0;
    int round1_gps_height_edge_count = 0;
    int round1_loop_edge_count = 0;
    int gps_height_direct_count = 0;
    int gps_height_interp_count = 0;
    std::vector<HeightTrendTarget> gps_height_targets;
    std::vector<std::shared_ptr<miao::VertexSE3>> support_vertices(keyframes_.size(), nullptr);

    if (support_keyframe_count > 0) {
        miao::OptimizerConfig config(
            miao::AlgorithmType::LEVENBERG_MARQUARDT,
            miao::LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN,
            false);
        auto optimizer_round1 = miao::SetupOptimizer<Eigen::Dynamic, Eigen::Dynamic>(config);
        optimizer_round1->SetVerbose(options_.verbose);

        for (size_t i = 0; i < keyframes_.size(); ++i) {
            if (!support_mask[i]) {
                continue;
            }
            auto vertex = std::make_shared<miao::VertexSE3>();
            vertex->SetId(static_cast<int>(i));
            vertex->SetEstimate(keyframes_[i]->GetOptPose());
            optimizer_round1->AddVertex(vertex);
            support_vertices[i] = vertex;
        }

        for (size_t i = 1; i < keyframes_.size(); ++i) {
            if (!support_mask[i - 1] || !support_mask[i]) {
                continue;
            }
            if (add_lio_relative_edge(
                    optimizer_round1,
                    support_vertices[i - 1],
                    support_vertices[i],
                    keyframes_[i]->GetRelativeMotion(),
                    keyframes_[i]->GetCovariance(),
                    20000 + static_cast<int>(i))) {
                ++round1_lio_edge_count;
            }
        }

        if (options_.weight_height_smooth > 0.0) {
            for (size_t i = 1; i < keyframes_.size(); ++i) {
                if (!support_mask[i - 1] || !support_mask[i]) {
                    continue;
                }
                auto edge = std::make_shared<EdgeHeightSmooth>();
                edge->SetId(30000 + static_cast<int>(i));
                edge->SetVertex(0, support_vertices[i - 1]);
                edge->SetVertex(1, support_vertices[i]);
                edge->SetMeasurement(keyframes_[i]->GetRelativeMotion().translation().z());
                Eigen::Matrix<double, 1, 1> info;
                info << options_.weight_height_smooth;
                edge->SetInformation(info);
                optimizer_round1->AddEdge(edge);
                ++round1_height_smooth_edge_count;
            }
        }

        if (options_.enable_gps_position && has_utm_offset_) {
            for (size_t i = 0; i < keyframes_.size(); ++i) {
                if (!support_mask[i]) {
                    continue;
                }
                auto gps_data = keyframes_[i]->GetGpsData();
                if (!gps_data.has_gps) {
                    continue;
                }

                const Vec3d gps_lio_pos = gps_data.gps_utm_position - utm_to_lio_offset_;
                if (!gps_lio_pos.allFinite()) {
                    continue;
                }

                auto edge = std::make_shared<EdgeGpsPosition>();
                edge->SetId(40000 + static_cast<int>(i));
                edge->SetVertex(0, support_vertices[i]);
                edge->SetMeasurement(gps_lio_pos);

                const double std_x = std::max(gps_data.gps_std_dev.x(), options_.min_gps_std_xy);
                const double std_y = std::max(gps_data.gps_std_dev.y(), options_.min_gps_std_xy);
                Vec3d info_diag(
                    std::min(options_.weight_gps_position / (std_x * std_x), options_.max_gps_info_xy),
                    std::min(options_.weight_gps_position / (std_y * std_y), options_.max_gps_info_xy),
                    1e-9);
                if (!info_diag.allFinite() || info_diag.x() <= 0.0 || info_diag.y() <= 0.0) {
                    continue;
                }

                edge->SetInformation(info_diag.asDiagonal());
                auto huber_kernel = std::make_shared<miao::RobustKernelHuber>();
                huber_kernel->SetDelta(options_.huber_gps_pos_delta);
                edge->SetRobustKernel(huber_kernel);
                optimizer_round1->AddEdge(edge);
                ++round1_gps_pos_edge_count;
            }
        }

        if (options_.enable_gps_heading) {
            for (size_t i = 0; i < keyframes_.size(); ++i) {
                if (!support_mask[i]) {
                    continue;
                }
                auto gps_data = keyframes_[i]->GetGpsData();
                if (!gps_data.has_gps) {
                    continue;
                }

                const double std_rad = std::max(gps_data.heading_std_deg * M_PI / 180.0, 1e-3);
                auto edge = std::make_shared<EdgeGpsHeading>();
                edge->SetId(50000 + static_cast<int>(i));
                edge->SetVertex(0, support_vertices[i]);
                edge->SetMeasurement(gps_data.gps_heading_deg * M_PI / 180.0);
                edge->SetInformation(
                    Eigen::Matrix<double, 1, 1>::Constant(options_.weight_gps_heading / (std_rad * std_rad)));

                auto cauchy_kernel = std::make_shared<miao::RobustKernelCauchy>();
                cauchy_kernel->SetDelta(options_.cauchy_heading_delta);
                edge->SetRobustKernel(cauchy_kernel);
                optimizer_round1->AddEdge(edge);
                ++round1_gps_heading_edge_count;
            }
        }

        if (options_.enable_gps_height && has_utm_offset_) {
            std::vector<std::pair<size_t, double>> gps_anchors;
            gps_anchors.reserve(keyframes_.size());
            for (size_t i = 0; i < keyframes_.size(); ++i) {
                if (!support_mask[i]) {
                    continue;
                }
                auto gps_data = keyframes_[i]->GetGpsData();
                if (!gps_data.has_gps) {
                    continue;
                }
                const double height_lio = gps_data.gps_utm_position.z() - utm_to_lio_offset_.z();
                if (std::isfinite(height_lio)) {
                    gps_anchors.emplace_back(i, height_lio);
                }
            }

            constexpr int kMaxInterpRange = 50;
            for (size_t i = 0; i < keyframes_.size(); ++i) {
                if (!support_mask[i]) {
                    continue;
                }

                auto gps_data = keyframes_[i]->GetGpsData();
                double ref_height = 0.0;
                double weight = 0.0;
                bool is_direct = false;

                if (gps_data.has_gps) {
                    ref_height = gps_data.gps_utm_position.z() - utm_to_lio_offset_.z();
                    const double std_z = std::max(gps_data.gps_std_dev.z(), 0.05);
                    weight = options_.weight_gps_height / (std_z * std_z);
                    is_direct = true;
                } else {
                    int prev_idx = -1;
                    int next_idx = -1;
                    double prev_h = 0.0;
                    double next_h = 0.0;
                    for (const auto& anchor : gps_anchors) {
                        if (anchor.first <= i) {
                            prev_idx = static_cast<int>(anchor.first);
                            prev_h = anchor.second;
                        }
                        if (anchor.first >= i && next_idx < 0) {
                            next_idx = static_cast<int>(anchor.first);
                            next_h = anchor.second;
                        }
                    }

                    const int dist_to_prev = (prev_idx >= 0) ? static_cast<int>(i) - prev_idx : INT_MAX;
                    const int dist_to_next = (next_idx >= 0) ? next_idx - static_cast<int>(i) : INT_MAX;
                    if (prev_idx >= 0 && next_idx >= 0 && next_idx != prev_idx
                        && dist_to_prev <= kMaxInterpRange && dist_to_next <= kMaxInterpRange) {
                        const double t = double(static_cast<int>(i) - prev_idx) / double(next_idx - prev_idx);
                        ref_height = prev_h + t * (next_h - prev_h);
                        const double gap_ratio =
                            std::min(double(dist_to_prev), double(dist_to_next)) / kMaxInterpRange;
                        weight = options_.weight_gps_height * 0.2 * (1.0 - 0.8 * gap_ratio);
                    } else if (prev_idx >= 0 && dist_to_prev <= kMaxInterpRange) {
                        ref_height = prev_h;
                        weight = options_.weight_gps_height * 0.1 *
                                 (1.0 - double(dist_to_prev) / kMaxInterpRange);
                    } else if (next_idx >= 0 && dist_to_next <= kMaxInterpRange) {
                        ref_height = next_h;
                        weight = options_.weight_gps_height * 0.1 *
                                 (1.0 - double(dist_to_next) / kMaxInterpRange);
                    } else {
                        continue;
                    }
                }

                if (!std::isfinite(ref_height) || !std::isfinite(weight) || weight <= 0.0) {
                    continue;
                }

                auto edge = std::make_shared<EdgeGpsHeight>();
                edge->SetId(60000 + static_cast<int>(i));
                edge->SetVertex(0, support_vertices[i]);
                edge->SetMeasurement(ref_height);
                Eigen::Matrix<double, 1, 1> info;
                info << weight;
                edge->SetInformation(info);

                auto huber_kernel = std::make_shared<miao::RobustKernelHuber>();
                huber_kernel->SetDelta(0.5);
                edge->SetRobustKernel(huber_kernel);
                optimizer_round1->AddEdge(edge);
                ++round1_gps_height_edge_count;
                if (is_direct) {
                    ++gps_height_direct_count;
                } else {
                    ++gps_height_interp_count;
                }

                HeightTrendTarget target;
                target.keyframe_index = i;
                target.trend_height = ref_height;
                target.raw_height = ref_height;
                target.path_distance_m = keyframe_path_distances[i];
                gps_height_targets.push_back(target);
            }
        }

        const std::vector<PrecomputedLoopConstraint> cached_loop_constraints_round1 = GetLoopConstraintsSnapshot();
        log_loop_constraint_snapshot(cached_loop_constraints_round1, "LOOP_ROUND1_CACHE");
        round1_loop_edge_count = add_cached_loop_edges(
            optimizer_round1,
            support_vertices,
            cached_loop_constraints_round1,
            &support_mask,
            &outage_mask,
            70000,
            "ROUND1_LOOP");

        AINFO << "[ROUND1] Graph summary:"
              << " lio=" << round1_lio_edge_count
              << ", z_smooth=" << round1_height_smooth_edge_count
              << ", gps_xy=" << round1_gps_pos_edge_count
              << ", gps_heading=" << round1_gps_heading_edge_count
              << ", gps_z=" << round1_gps_height_edge_count
              << ", loops=" << round1_loop_edge_count;

        if (run_optimizer(optimizer_round1, "ROUND1", options_.max_iterations)) {
            for (size_t i = 0; i < keyframes_.size(); ++i) {
                if (support_vertices[i]) {
                    keyframes_[i]->SetOptPose(support_vertices[i]->Estimate());
                }
            }
        }
    } else {
        AINFO << "[ROUND1] No support keyframes, skipped";
    }

    // ==================== Stage 2: rigid outage block docking ====================
    const int kDockingTargetHalfRange = std::max(8, options_.outage_overlap_keyframes * 4);
    const int kDockingSourceHalfRange = std::max(6, options_.outage_overlap_keyframes * 3);
    const Mat6d docking_info = BuildSe3Information(0.6, 8.0 * M_PI / 180.0);
    std::vector<OutageDockingResult> docking_results(outage_segments.size());
    int round2_blocks_docked = 0;
    int round2_docking_edges = 0;
    int round2_keyframes_initialized = 0;

    for (size_t seg_idx = 0; seg_idx < outage_segments.size(); ++seg_idx) {
        const auto& segment = outage_segments[seg_idx];
        auto& docking = docking_results[seg_idx];
        docking.representative_idx = (segment.start_idx + segment.end_idx) / 2;

        const SE3 rep_lio_pose = keyframes_[docking.representative_idx]->GetLIOPose();
        const size_t segment_len = segment.end_idx - segment.start_idx + 1;
        docking.local_pose_from_representative.resize(segment_len);
        for (size_t idx = segment.start_idx; idx <= segment.end_idx; ++idx) {
            docking.local_pose_from_representative[idx - segment.start_idx] =
                rep_lio_pose.inverse() * keyframes_[idx]->GetLIOPose();
        }

        auto initialize_representative_pose = [&]() -> SE3 {
            if (IsValidKeyframeIndex(segment.left_anchor_idx, keyframes_.size())) {
                return keyframes_[segment.left_anchor_idx]->GetOptPose()
                       * (keyframes_[segment.left_anchor_idx]->GetLIOPose().inverse() * rep_lio_pose);
            }
            if (IsValidKeyframeIndex(segment.right_anchor_idx, keyframes_.size())) {
                return keyframes_[segment.right_anchor_idx]->GetOptPose()
                       * (keyframes_[segment.right_anchor_idx]->GetLIOPose().inverse() * rep_lio_pose);
            }
            return keyframes_[docking.representative_idx]->GetOptPose();
        };

        docking.representative_pose = initialize_representative_pose();
        for (size_t idx = segment.start_idx; idx <= segment.end_idx; ++idx) {
            keyframes_[idx]->SetOptPose(
                docking.representative_pose * docking.local_pose_from_representative[idx - segment.start_idx]);
            ++round2_keyframes_initialized;
        }

        std::vector<bool> segment_mask(keyframes_.size(), false);
        for (size_t idx = segment.start_idx; idx <= segment.end_idx; ++idx) {
            segment_mask[idx] = true;
        }
        std::unordered_map<size_t, CloudPtr> support_cache;
        std::unordered_map<size_t, CloudPtr> outage_cache;

        auto build_docking_constraint =
            [&](int support_idx, size_t outage_idx, DockingConstraint* constraint, const char* side_tag) -> bool {
                if (!constraint || !IsValidKeyframeIndex(support_idx, keyframes_.size()) ||
                    outage_idx >= keyframes_.size()) {
                    return false;
                }

                const CloudPtr target_submap = BuildWorldSubmap(
                    keyframes_, support_mask, static_cast<size_t>(support_idx),
                    kDockingTargetHalfRange, 2, T_imu_lidar, &support_cache);
                const CloudPtr source_submap = BuildWorldSubmap(
                    keyframes_, segment_mask, outage_idx,
                    kDockingSourceHalfRange, 1, T_imu_lidar, &outage_cache);

                Eigen::Matrix4f T_align = Eigen::Matrix4f::Identity();
                double ndt_score = 0.0;
                if (!RunMultiResolutionNdt(
                        target_submap,
                        source_submap,
                        loop_opts_.icp_fitness_threshold,
                        loop_opts_.icp_max_iterations,
                        &T_align,
                        &ndt_score)) {
                    AINFO << "[ROUND2_DOCK_SEG_" << seg_idx << "] side=" << side_tag
                          << " support=" << support_idx
                          << " outage=" << outage_idx
                          << " rejected";
                    return false;
                }

                const SE3 support_pose = keyframes_[support_idx]->GetOptPose();
                const SE3 outage_pose = keyframes_[outage_idx]->GetOptPose();
                const Eigen::Matrix4d T_corr = T_align.cast<double>();
                const SE3 delta_world_from_ndt(
                    SO3::fitToSO3(T_corr.block<3, 3>(0, 0)),
                    T_corr.block<3, 1>(0, 3));
                const SE3 aligned_outage_pose = delta_world_from_ndt * outage_pose;
                const Vec3d delta_translation = T_corr.block<3, 1>(0, 3);
                const double delta_rotation_deg =
                    SO3::fitToSO3(T_corr.block<3, 3>(0, 0)).log().norm() * 180.0 / M_PI;

                constraint->support_idx = support_idx;
                constraint->outage_idx = outage_idx;
                constraint->measurement_support_to_outage = support_pose.inverse() * aligned_outage_pose;
                constraint->ndt_score = ndt_score;
                constraint->valid = true;

                AINFO << "[ROUND2_DOCK_SEG_" << seg_idx << "] side=" << side_tag
                      << " support=" << support_idx
                      << " outage=" << outage_idx
                      << " ndt_score=" << std::fixed << std::setprecision(4) << ndt_score
                      << ", delta_xyz=[" << delta_translation.transpose() << "]"
                      << ", delta_rot_deg=" << delta_rotation_deg
                      << ", aligned_outage_z=" << aligned_outage_pose.translation().z();
                return true;
            };

        build_docking_constraint(segment.left_anchor_idx, segment.start_idx, &docking.left_constraint, "left");
        build_docking_constraint(segment.right_anchor_idx, segment.end_idx, &docking.right_constraint, "right");

        int docking_constraint_count = 0;
        miao::OptimizerConfig block_config(
            miao::AlgorithmType::LEVENBERG_MARQUARDT,
            miao::LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN,
            false);
        auto block_optimizer = miao::SetupOptimizer<Eigen::Dynamic, Eigen::Dynamic>(block_config);
        block_optimizer->SetVerbose(false);

        auto rep_vertex = std::make_shared<miao::VertexSE3>();
        rep_vertex->SetId(200000 + static_cast<int>(seg_idx));
        rep_vertex->SetEstimate(docking.representative_pose);
        block_optimizer->AddVertex(rep_vertex);

        auto add_rep_docking_edge =
            [&](const DockingConstraint& constraint, int edge_id, const char* side_tag) {
                if (!constraint.valid || constraint.outage_idx < segment.start_idx || constraint.outage_idx > segment.end_idx) {
                    return;
                }

                auto support_vertex = std::make_shared<miao::VertexSE3>();
                support_vertex->SetId(210000 + edge_id);
                support_vertex->SetEstimate(keyframes_[constraint.support_idx]->GetOptPose());
                support_vertex->SetFixed(true);
                block_optimizer->AddVertex(support_vertex);

                const SE3 local_rep_to_anchor =
                    docking.local_pose_from_representative[constraint.outage_idx - segment.start_idx];
                auto edge = std::make_shared<miao::EdgeSE3>();
                edge->SetId(220000 + edge_id);
                edge->SetVertex(0, support_vertex);
                edge->SetVertex(1, rep_vertex);
                edge->SetMeasurement(
                    AdjustMeasurementForRepresentative(constraint.measurement_support_to_outage, local_rep_to_anchor));
                edge->SetInformation(docking_info);
                auto huber = std::make_shared<miao::RobustKernelHuber>();
                huber->SetDelta(1.0);
                edge->SetRobustKernel(huber);
                block_optimizer->AddEdge(edge);
                ++docking_constraint_count;
                ++round2_docking_edges;
                AINFO << "[ROUND2_DOCK_SEG_" << seg_idx << "] side=" << side_tag
                      << " rep_edge_added";
            };

        add_rep_docking_edge(docking.left_constraint, static_cast<int>(seg_idx) * 2, "left");
        add_rep_docking_edge(docking.right_constraint, static_cast<int>(seg_idx) * 2 + 1, "right");

        if (docking_constraint_count > 0 && run_optimizer(block_optimizer, "ROUND2_DOCK_SEG_" + std::to_string(seg_idx), 20)) {
            docking.representative_pose = rep_vertex->Estimate();
            ++round2_blocks_docked;
        }

        for (size_t idx = segment.start_idx; idx <= segment.end_idx; ++idx) {
            keyframes_[idx]->SetOptPose(
                docking.representative_pose * docking.local_pose_from_representative[idx - segment.start_idx]);
        }

        AINFO << "[ROUND2_DOCK_SEG_" << seg_idx << "] representative=" << docking.representative_idx
              << ", outage=[" << segment.start_idx << "," << segment.end_idx << "]"
              << ", constraints=" << docking_constraint_count
              << ", initialized_kf=" << segment_len;
    }

    // ==================== Stage 2.5: outage block-to-global-submap matching ====================
    const Mat6d outage_block_prior_info = BuildSe3Information(
        std::max(options_.outage_block_prior_translation_sigma_m, 0.05),
        std::max(options_.outage_block_prior_rotation_sigma_deg * M_PI / 180.0, 1.0 * M_PI / 180.0));
    std::vector<std::vector<OutageBlockMapConstraint>> outage_block_constraints(outage_segments.size());
    int round2_block_queries = 0;
    int round2_block_constraints = 0;

    if (options_.enable_outage_block_scan_to_map && !outage_segments.empty()) {
        const int min_source_keyframes = std::max(options_.outage_block_min_source_keyframes, 1);
        const int min_target_keyframes = std::max(6, min_source_keyframes / 2);
        const double block_match_score_threshold = std::max(loop_opts_.icp_fitness_threshold, 1.0);
        const double max_xy_correction_m =
            std::max(30.0, options_.outage_block_target_radius_m * 0.25);
        const double max_z_correction_m =
            std::max(6.0, options_.outage_block_max_coarse_z_search_m + options_.outage_block_coarse_z_step_m + 2.0);

        AINFO << "[ROUND2_BLOCK_MAP] enable=true"
              << ", target_radius_m=" << options_.outage_block_target_radius_m
              << ", source_window_radius_m=" << options_.outage_block_source_window_radius_m
              << ", window_spacing_m=" << options_.outage_block_window_spacing_m
              << ", min_source_kf=" << min_source_keyframes
              << ", target_sample_step=" << options_.outage_block_target_sample_step
              << ", score_th=" << block_match_score_threshold
              << ", coarse_z_search_m=" << options_.outage_block_max_coarse_z_search_m
              << ", coarse_z_step_m=" << options_.outage_block_coarse_z_step_m;

        std::unordered_map<size_t, CloudPtr> support_cache;
        std::unordered_map<size_t, CloudPtr> outage_cache;
        for (size_t seg_idx = 0; seg_idx < outage_segments.size(); ++seg_idx) {
            const auto& segment = outage_segments[seg_idx];
            auto& segment_constraints = outage_block_constraints[seg_idx];
            const std::vector<size_t> window_centers = BuildOutageBlockWindowCenters(
                segment, keyframe_path_distances, options_.outage_block_window_spacing_m);

            int seg_queries = 0;
            int seg_accepts = 0;
            for (size_t center_idx : window_centers) {
                ++seg_queries;
                ++round2_block_queries;

                const std::vector<size_t> source_indices = CollectOutageWindowIndices(
                    segment, keyframe_path_distances, center_idx, options_.outage_block_source_window_radius_m);
                if (static_cast<int>(source_indices.size()) < min_source_keyframes) {
                    AINFO << "[ROUND2_BLOCK_MAP_SEG_" << seg_idx << "] center=" << center_idx
                          << " skipped source_kf=" << source_indices.size()
                          << " (<" << min_source_keyframes << ")";
                    continue;
                }

                int source_keyframes = 0;
                const CloudPtr source_submap = BuildWorldSubmapFromIndices(
                    keyframes_, source_indices, T_imu_lidar, &outage_cache, &source_keyframes);
                if (!source_submap || source_submap->empty()) {
                    AINFO << "[ROUND2_BLOCK_MAP_SEG_" << seg_idx << "] center=" << center_idx
                          << " skipped empty_source";
                    continue;
                }

                int target_keyframes = 0;
                const CloudPtr target_submap = BuildWorldSubmapByRadius(
                    keyframes_,
                    support_mask,
                    keyframes_[center_idx]->GetOptPose().translation(),
                    options_.outage_block_target_radius_m,
                    options_.outage_block_target_sample_step,
                    T_imu_lidar,
                    &support_cache,
                    &target_keyframes);
                if (!target_submap || target_submap->empty() || target_keyframes < min_target_keyframes) {
                    AINFO << "[ROUND2_BLOCK_MAP_SEG_" << seg_idx << "] center=" << center_idx
                          << " skipped target_kf=" << target_keyframes
                          << ", target_empty=" << ((!target_submap || target_submap->empty()) ? "true" : "false");
                    continue;
                }

                Eigen::Matrix4f T_delta = Eigen::Matrix4f::Identity();
                double ndt_score = 0.0;
                double initial_z_offset_m = 0.0;
                if (!RunCoarseToFineNdtWithZOffsets(
                        target_submap,
                        source_submap,
                        options_.outage_block_max_coarse_z_search_m,
                        options_.outage_block_coarse_z_step_m,
                        block_match_score_threshold,
                        loop_opts_.icp_max_iterations,
                        &T_delta,
                        &ndt_score,
                        &initial_z_offset_m)) {
                    AINFO << "[ROUND2_BLOCK_MAP_SEG_" << seg_idx << "] center=" << center_idx
                          << " rejected score<" << block_match_score_threshold;
                    continue;
                }

                const Eigen::Matrix4d T_delta_d = T_delta.cast<double>();
                const SE3 delta_world_from_ndt(
                    SO3::fitToSO3(T_delta_d.block<3, 3>(0, 0)),
                    T_delta_d.block<3, 1>(0, 3));
                const Vec3d delta_translation = T_delta_d.block<3, 1>(0, 3);
                const double delta_rotation_deg =
                    delta_world_from_ndt.so3().log().norm() * 180.0 / M_PI;
                const double delta_xy_m = delta_translation.head<2>().norm();
                if (!std::isfinite(delta_xy_m) || delta_xy_m > max_xy_correction_m ||
                    !std::isfinite(delta_translation.z()) || std::abs(delta_translation.z()) > max_z_correction_m) {
                    AINFO << "[ROUND2_BLOCK_MAP_SEG_" << seg_idx << "] center=" << center_idx
                          << " rejected_by_delta_gate"
                          << " delta_xy=" << delta_xy_m
                          << ", delta_z=" << delta_translation.z()
                          << ", max_xy=" << max_xy_correction_m
                          << ", max_z=" << max_z_correction_m;
                    continue;
                }

                OutageBlockMapConstraint constraint;
                constraint.center_idx = center_idx;
                constraint.window_start_idx = source_indices.front();
                constraint.window_end_idx = source_indices.back();
                constraint.matched_world_pose = delta_world_from_ndt * keyframes_[center_idx]->GetOptPose();
                constraint.ndt_score = ndt_score;
                constraint.initial_z_offset_m = initial_z_offset_m;
                constraint.delta_translation = delta_translation;
                constraint.delta_rotation_deg = delta_rotation_deg;
                constraint.source_keyframes = source_keyframes;
                constraint.target_keyframes = target_keyframes;
                constraint.valid = true;
                segment_constraints.push_back(constraint);
                ++seg_accepts;
                ++round2_block_constraints;

                AINFO << "[ROUND2_BLOCK_MAP_SEG_" << seg_idx << "] center=" << center_idx
                      << " accepted"
                      << " window=[" << constraint.window_start_idx << "," << constraint.window_end_idx << "]"
                      << ", source_kf=" << source_keyframes
                      << ", target_kf=" << target_keyframes
                      << ", init_z_offset_m=" << initial_z_offset_m
                      << ", ndt_score=" << std::fixed << std::setprecision(4) << ndt_score
                      << ", delta_xyz=[" << delta_translation.transpose() << "]"
                      << ", delta_rot_deg=" << delta_rotation_deg
                      << ", matched_z=" << constraint.matched_world_pose.translation().z();
            }

            AINFO << "[ROUND2_BLOCK_MAP_SEG_" << seg_idx << "] summary"
                  << " windows=" << window_centers.size()
                  << ", queries=" << seg_queries
                  << ", accepted=" << seg_accepts;
        }

        AINFO << "[ROUND2_BLOCK_MAP] summary"
              << " queries=" << round2_block_queries
              << ", constraints=" << round2_block_constraints;
    } else {
        AINFO << "[ROUND2_BLOCK_MAP] skipped"
              << " enable=" << (options_.enable_outage_block_scan_to_map ? "true" : "false")
              << ", segments=" << outage_segments.size();
    }

    auto mine_outage_support_loops = [&](const std::vector<OutageSegment>& segments) {
        if (!loop_opts_.enable || segments.empty()) {
            AINFO << "[LOOP_OUTAGE_RECHECK] skipped"
                  << " enable=" << (loop_opts_.enable ? "true" : "false")
                  << ", segments=" << segments.size();
            return;
        }

        const int closest_id_th =
            std::max(loop_opts_.closest_id_threshold, std::max(loop_opts_.min_keyframe_gap, 1));
        const size_t query_step = static_cast<size_t>(std::max(1, loop_opts_.loop_kf_gap / 2));
        std::unordered_map<size_t, CloudPtr> support_cache;

        int total_queries = 0;
        int total_queries_with_candidates = 0;
        int total_queries_with_accepts = 0;
        int total_candidate_pairs = 0;
        int total_new_constraints = 0;
        std::unordered_set<size_t> total_hit_queries;

        for (size_t seg_idx = 0; seg_idx < segments.size(); ++seg_idx) {
            const auto& segment = segments[seg_idx];
            int seg_queries = 0;
            int seg_queries_with_candidates = 0;
            int seg_queries_with_accepts = 0;
            int seg_candidate_pairs = 0;
            int seg_new_constraints = 0;
            std::unordered_set<size_t> seg_hit_queries;

            std::vector<size_t> query_indices;
            query_indices.reserve((segment.end_idx - segment.start_idx + 1) / query_step + 2);
            for (size_t query_idx = segment.start_idx; query_idx <= segment.end_idx; query_idx += query_step) {
                query_indices.push_back(query_idx);
            }
            if (query_indices.empty() || query_indices.back() != segment.end_idx) {
                query_indices.push_back(segment.end_idx);
            }

            for (size_t query_idx : query_indices) {
                if (query_idx >= keyframes_.size()) {
                    continue;
                }
                const Keyframe::Ptr& query_kf = keyframes_[query_idx];
                if (!query_kf) {
                    continue;
                }

                const CloudPtr source_local = query_kf->GetCloud();
                if (!source_local || source_local->empty()) {
                    continue;
                }

                const Vec3d query_pos = query_kf->GetOptPose().translation();
                if (!query_pos.allFinite()) {
                    continue;
                }

                ++seg_queries;
                ++total_queries;

                std::vector<size_t> candidate_indices;
                candidate_indices.reserve(32);
                size_t last_candidate_idx = std::numeric_limits<size_t>::max();
                for (size_t support_idx = 0; support_idx < keyframes_.size(); ++support_idx) {
                    if (!support_mask[support_idx]) {
                        continue;
                    }
                    if (std::abs(static_cast<int>(support_idx) - static_cast<int>(query_idx)) < closest_id_th) {
                        continue;
                    }
                    if (last_candidate_idx != std::numeric_limits<size_t>::max() &&
                        std::abs(static_cast<int>(support_idx) - static_cast<int>(last_candidate_idx)) <=
                            loop_opts_.min_id_interval) {
                        continue;
                    }

                    const Vec3d support_pos = keyframes_[support_idx]->GetOptPose().translation();
                    if (!support_pos.allFinite()) {
                        continue;
                    }
                    const double dist_xy = (support_pos.head<2>() - query_pos.head<2>()).norm();
                    if (!std::isfinite(dist_xy) || dist_xy > loop_opts_.search_radius) {
                        continue;
                    }

                    candidate_indices.push_back(support_idx);
                    last_candidate_idx = support_idx;
                }

                if (candidate_indices.empty()) {
                    continue;
                }

                ++seg_queries_with_candidates;
                ++total_queries_with_candidates;
                seg_candidate_pairs += static_cast<int>(candidate_indices.size());
                total_candidate_pairs += static_cast<int>(candidate_indices.size());

                int accepted_count = 0;
                for (size_t support_idx : candidate_indices) {
                    const CloudPtr target_submap = BuildWorldSubmap(
                        keyframes_,
                        support_mask,
                        support_idx,
                        loop_opts_.history_submap_half_range,
                        loop_opts_.history_submap_step,
                        T_imu_lidar,
                        &support_cache);
                    if (!target_submap || target_submap->empty()) {
                        continue;
                    }

                    Eigen::Matrix4f T_align = query_kf->GetOptPose().matrix().cast<float>();
                    double ndt_score = 0.0;
                    if (!RunMultiResolutionNdt(
                            target_submap,
                            source_local,
                            loop_opts_.icp_fitness_threshold,
                            loop_opts_.icp_max_iterations,
                            &T_align,
                            &ndt_score)) {
                        continue;
                    }

                    const Eigen::Matrix4d T_query_world = T_align.cast<double>();
                    const SE3 query_world_pose(
                        SO3::fitToSO3(T_query_world.block<3, 3>(0, 0)),
                        T_query_world.block<3, 1>(0, 3));
                    const SE3 target_pose = keyframes_[support_idx]->GetOptPose();

                    PrecomputedLoopConstraint constraint;
                    constraint.target_idx = support_idx;
                    constraint.source_idx = query_idx;
                    constraint.measurement_target_to_source = target_pose.inverse() * query_world_pose;
                    constraint.ndt_score = ndt_score;
                    constraint.origin = LoopConstraintOrigin::kOutageSupportRematch;

                    const uint64_t key = MakeLoopConstraintKey(support_idx, query_idx);
                    bool inserted = false;
                    {
                        std::lock_guard<std::mutex> lock(loop_constraints_mutex_);
                        if (loop_constraint_keys_.insert(key).second) {
                            loop_constraints_.push_back(constraint);
                            inserted = true;
                        }
                    }
                    if (!inserted) {
                        continue;
                    }

                    ++accepted_count;
                    ++seg_new_constraints;
                    ++total_new_constraints;
                    seg_hit_queries.insert(query_idx);
                    total_hit_queries.insert(query_idx);
                    AINFO << "[LOOP_OUTAGE_RECHECK] Accepted (" << support_idx << "," << query_idx << ")"
                          << ": ndt_score=" << std::fixed << std::setprecision(4) << ndt_score
                          << ", support_z=" << target_pose.translation().z()
                          << ", query_z=" << query_world_pose.translation().z();
                }

                if (accepted_count > 0) {
                    ++seg_queries_with_accepts;
                    ++total_queries_with_accepts;
                }
                AINFO << "[LOOP_OUTAGE_RECHECK] Query " << query_idx
                      << ": coarse_candidates=" << candidate_indices.size()
                      << ", accepted=" << accepted_count;
            }

            int hit_query_min = -1;
            int hit_query_max = -1;
            if (!seg_hit_queries.empty()) {
                hit_query_min = static_cast<int>(*std::min_element(seg_hit_queries.begin(), seg_hit_queries.end()));
                hit_query_max = static_cast<int>(*std::max_element(seg_hit_queries.begin(), seg_hit_queries.end()));
            }
            AINFO << "[LOOP_OUTAGE_RECHECK] Segment " << seg_idx
                  << ": query_step=" << query_step
                  << ", queries=" << seg_queries
                  << ", queries_with_candidates=" << seg_queries_with_candidates
                  << ", queries_with_accepts=" << seg_queries_with_accepts
                  << ", candidate_pairs=" << seg_candidate_pairs
                  << ", new_constraints=" << seg_new_constraints
                  << ", hit_query_range=[" << hit_query_min << "," << hit_query_max << "]";
        }

        int total_hit_query_min = -1;
        int total_hit_query_max = -1;
        if (!total_hit_queries.empty()) {
            total_hit_query_min = static_cast<int>(*std::min_element(total_hit_queries.begin(), total_hit_queries.end()));
            total_hit_query_max = static_cast<int>(*std::max_element(total_hit_queries.begin(), total_hit_queries.end()));
        }
        AINFO << "[LOOP_OUTAGE_RECHECK] Total:"
              << " queries=" << total_queries
              << ", queries_with_candidates=" << total_queries_with_candidates
              << ", queries_with_accepts=" << total_queries_with_accepts
              << ", candidate_pairs=" << total_candidate_pairs
              << ", new_constraints=" << total_new_constraints
              << ", unique_hit_queries=" << total_hit_queries.size()
              << ", hit_query_range=[" << total_hit_query_min << "," << total_hit_query_max << "]";
    };

    mine_outage_support_loops(outage_segments);

    // ==================== Stage 3: all-frame unified optimize with docking edges ====================
    int stage3_lio_edge_count = 0;
    int stage3_height_smooth_edge_count = 0;
    int stage3_gps_pos_edge_count = 0;
    int stage3_gps_heading_edge_count = 0;
    int stage3_gps_height_edge_count = 0;
    int stage3_docking_edge_count = 0;
    int stage3_block_prior_edge_count = 0;
    int stage3_loop_edge_count = 0;

    if (!keyframes_.empty()) {
        miao::OptimizerConfig config_stage3(
            miao::AlgorithmType::LEVENBERG_MARQUARDT,
            miao::LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN,
            false);
        auto optimizer_stage3 = miao::SetupOptimizer<Eigen::Dynamic, Eigen::Dynamic>(config_stage3);
        optimizer_stage3->SetVerbose(options_.verbose);

        std::vector<std::shared_ptr<miao::VertexSE3>> all_vertices(keyframes_.size(), nullptr);
        for (size_t i = 0; i < keyframes_.size(); ++i) {
            auto vertex = std::make_shared<miao::VertexSE3>();
            vertex->SetId(300000 + static_cast<int>(i));
            vertex->SetEstimate(keyframes_[i]->GetOptPose());
            optimizer_stage3->AddVertex(vertex);
            all_vertices[i] = vertex;
        }

        std::vector<bool> skip_seam_lio_edge(keyframes_.size(), false);
        for (size_t seg_idx = 0; seg_idx < outage_segments.size(); ++seg_idx) {
            const auto& segment = outage_segments[seg_idx];
            const auto& docking = docking_results[seg_idx];
            if (docking.left_constraint.valid && segment.start_idx < keyframes_.size()) {
                skip_seam_lio_edge[segment.start_idx] = true;
            }
            if (docking.right_constraint.valid && IsValidKeyframeIndex(segment.right_anchor_idx, keyframes_.size())) {
                skip_seam_lio_edge[static_cast<size_t>(segment.right_anchor_idx)] = true;
            }
        }

        for (size_t i = 1; i < keyframes_.size(); ++i) {
            if (skip_seam_lio_edge[i]) {
                continue;
            }
            if (add_lio_relative_edge(
                    optimizer_stage3,
                    all_vertices[i - 1],
                    all_vertices[i],
                    keyframes_[i]->GetRelativeMotion(),
                    keyframes_[i]->GetCovariance(),
                    310000 + static_cast<int>(i))) {
                ++stage3_lio_edge_count;
            }
        }

        if (options_.weight_height_smooth > 0.0) {
            for (size_t i = 1; i < keyframes_.size(); ++i) {
                if (!support_mask[i - 1] || !support_mask[i]) {
                    continue;
                }
                auto edge = std::make_shared<EdgeHeightSmooth>();
                edge->SetId(320000 + static_cast<int>(i));
                edge->SetVertex(0, all_vertices[i - 1]);
                edge->SetVertex(1, all_vertices[i]);
                edge->SetMeasurement(keyframes_[i]->GetRelativeMotion().translation().z());
                Eigen::Matrix<double, 1, 1> info;
                info << options_.weight_height_smooth;
                edge->SetInformation(info);
                optimizer_stage3->AddEdge(edge);
                ++stage3_height_smooth_edge_count;
            }
        }

        if (options_.enable_gps_position && has_utm_offset_) {
            for (size_t i = 0; i < keyframes_.size(); ++i) {
                if (!support_mask[i]) {
                    continue;
                }
                auto gps_data = keyframes_[i]->GetGpsData();
                if (!gps_data.has_gps) {
                    continue;
                }

                const Vec3d gps_lio_pos = gps_data.gps_utm_position - utm_to_lio_offset_;
                if (!gps_lio_pos.allFinite()) {
                    continue;
                }

                auto edge = std::make_shared<EdgeGpsPosition>();
                edge->SetId(330000 + static_cast<int>(i));
                edge->SetVertex(0, all_vertices[i]);
                edge->SetMeasurement(gps_lio_pos);

                const double std_x = std::max(gps_data.gps_std_dev.x(), options_.min_gps_std_xy);
                const double std_y = std::max(gps_data.gps_std_dev.y(), options_.min_gps_std_xy);
                Vec3d info_diag(
                    std::min(options_.weight_gps_position / (std_x * std_x), options_.max_gps_info_xy),
                    std::min(options_.weight_gps_position / (std_y * std_y), options_.max_gps_info_xy),
                    1e-9);
                if (!info_diag.allFinite() || info_diag.x() <= 0.0 || info_diag.y() <= 0.0) {
                    continue;
                }

                edge->SetInformation(info_diag.asDiagonal());
                auto huber_kernel = std::make_shared<miao::RobustKernelHuber>();
                huber_kernel->SetDelta(options_.huber_gps_pos_delta);
                edge->SetRobustKernel(huber_kernel);
                optimizer_stage3->AddEdge(edge);
                ++stage3_gps_pos_edge_count;
            }
        }

        if (options_.enable_gps_heading) {
            for (size_t i = 0; i < keyframes_.size(); ++i) {
                if (!support_mask[i]) {
                    continue;
                }
                auto gps_data = keyframes_[i]->GetGpsData();
                if (!gps_data.has_gps) {
                    continue;
                }

                const double std_rad = std::max(gps_data.heading_std_deg * M_PI / 180.0, 1e-3);
                auto edge = std::make_shared<EdgeGpsHeading>();
                edge->SetId(340000 + static_cast<int>(i));
                edge->SetVertex(0, all_vertices[i]);
                edge->SetMeasurement(gps_data.gps_heading_deg * M_PI / 180.0);
                edge->SetInformation(
                    Eigen::Matrix<double, 1, 1>::Constant(options_.weight_gps_heading / (std_rad * std_rad)));

                auto cauchy_kernel = std::make_shared<miao::RobustKernelCauchy>();
                cauchy_kernel->SetDelta(options_.cauchy_heading_delta);
                edge->SetRobustKernel(cauchy_kernel);
                optimizer_stage3->AddEdge(edge);
                ++stage3_gps_heading_edge_count;
            }
        }

        if (options_.enable_gps_height && has_utm_offset_) {
            std::vector<std::pair<size_t, double>> gps_anchors;
            gps_anchors.reserve(keyframes_.size());
            for (size_t i = 0; i < keyframes_.size(); ++i) {
                if (!support_mask[i]) {
                    continue;
                }
                auto gps_data = keyframes_[i]->GetGpsData();
                if (!gps_data.has_gps) {
                    continue;
                }
                const double height_lio = gps_data.gps_utm_position.z() - utm_to_lio_offset_.z();
                if (std::isfinite(height_lio)) {
                    gps_anchors.emplace_back(i, height_lio);
                }
            }

            constexpr int kMaxInterpRange = 50;
            for (size_t i = 0; i < keyframes_.size(); ++i) {
                if (!support_mask[i]) {
                    continue;
                }

                auto gps_data = keyframes_[i]->GetGpsData();
                double ref_height = 0.0;
                double weight = 0.0;

                if (gps_data.has_gps) {
                    ref_height = gps_data.gps_utm_position.z() - utm_to_lio_offset_.z();
                    const double std_z = std::max(gps_data.gps_std_dev.z(), 0.05);
                    weight = options_.weight_gps_height / (std_z * std_z);
                } else {
                    int prev_idx = -1;
                    int next_idx = -1;
                    double prev_h = 0.0;
                    double next_h = 0.0;
                    for (const auto& anchor : gps_anchors) {
                        if (anchor.first <= i) {
                            prev_idx = static_cast<int>(anchor.first);
                            prev_h = anchor.second;
                        }
                        if (anchor.first >= i && next_idx < 0) {
                            next_idx = static_cast<int>(anchor.first);
                            next_h = anchor.second;
                        }
                    }

                    const int dist_to_prev = (prev_idx >= 0) ? static_cast<int>(i) - prev_idx : INT_MAX;
                    const int dist_to_next = (next_idx >= 0) ? next_idx - static_cast<int>(i) : INT_MAX;
                    if (prev_idx >= 0 && next_idx >= 0 && next_idx != prev_idx &&
                        dist_to_prev <= kMaxInterpRange && dist_to_next <= kMaxInterpRange) {
                        const double t = double(static_cast<int>(i) - prev_idx) / double(next_idx - prev_idx);
                        ref_height = prev_h + t * (next_h - prev_h);
                        const double gap_ratio =
                            std::min(double(dist_to_prev), double(dist_to_next)) / kMaxInterpRange;
                        weight = options_.weight_gps_height * 0.2 * (1.0 - 0.8 * gap_ratio);
                    } else if (prev_idx >= 0 && dist_to_prev <= kMaxInterpRange) {
                        ref_height = prev_h;
                        weight = options_.weight_gps_height * 0.1 *
                                 (1.0 - double(dist_to_prev) / kMaxInterpRange);
                    } else if (next_idx >= 0 && dist_to_next <= kMaxInterpRange) {
                        ref_height = next_h;
                        weight = options_.weight_gps_height * 0.1 *
                                 (1.0 - double(dist_to_next) / kMaxInterpRange);
                    } else {
                        continue;
                    }
                }

                if (!std::isfinite(ref_height) || !std::isfinite(weight) || weight <= 0.0) {
                    continue;
                }

                auto edge = std::make_shared<EdgeGpsHeight>();
                edge->SetId(350000 + static_cast<int>(i));
                edge->SetVertex(0, all_vertices[i]);
                edge->SetMeasurement(ref_height);
                Eigen::Matrix<double, 1, 1> info;
                info << weight;
                edge->SetInformation(info);

                auto huber_kernel = std::make_shared<miao::RobustKernelHuber>();
                huber_kernel->SetDelta(0.5);
                edge->SetRobustKernel(huber_kernel);
                optimizer_stage3->AddEdge(edge);
                ++stage3_gps_height_edge_count;
            }
        }

        auto add_stage3_docking_edge = [&](const DockingConstraint& constraint, int edge_id) {
            if (!constraint.valid || constraint.support_idx < 0 || constraint.outage_idx >= keyframes_.size()) {
                return;
            }
            auto edge = std::make_shared<miao::EdgeSE3>();
            edge->SetId(edge_id);
            edge->SetVertex(0, all_vertices[static_cast<size_t>(constraint.support_idx)]);
            edge->SetVertex(1, all_vertices[constraint.outage_idx]);
            edge->SetMeasurement(constraint.measurement_support_to_outage);
            edge->SetInformation(docking_info);
            auto huber = std::make_shared<miao::RobustKernelHuber>();
            huber->SetDelta(1.0);
            edge->SetRobustKernel(huber);
            optimizer_stage3->AddEdge(edge);
            ++stage3_docking_edge_count;
        };

        for (size_t seg_idx = 0; seg_idx < docking_results.size(); ++seg_idx) {
            add_stage3_docking_edge(docking_results[seg_idx].left_constraint, 360000 + static_cast<int>(seg_idx) * 2);
            add_stage3_docking_edge(docking_results[seg_idx].right_constraint, 360000 + static_cast<int>(seg_idx) * 2 + 1);
        }

        auto add_stage3_block_prior_edge = [&](const OutageBlockMapConstraint& constraint, int edge_id) {
            if (!constraint.valid || constraint.center_idx >= keyframes_.size()) {
                return;
            }

            auto edge = std::make_shared<miao::EdgeSE3Prior>();
            edge->SetId(edge_id);
            edge->SetVertex(0, all_vertices[constraint.center_idx]);
            edge->SetMeasurement(constraint.matched_world_pose);
            edge->SetInformation(outage_block_prior_info);
            auto huber = std::make_shared<miao::RobustKernelHuber>();
            huber->SetDelta(1.0);
            edge->SetRobustKernel(huber);
            optimizer_stage3->AddEdge(edge);
            ++stage3_block_prior_edge_count;
        };

        int stage3_block_prior_edge_id = 365000;
        for (const auto& segment_constraints : outage_block_constraints) {
            for (const auto& constraint : segment_constraints) {
                add_stage3_block_prior_edge(constraint, stage3_block_prior_edge_id++);
            }
        }

        const std::vector<PrecomputedLoopConstraint> cached_loop_constraints_stage3 = GetLoopConstraintsSnapshot();
        log_loop_constraint_snapshot(cached_loop_constraints_stage3, "LOOP_STAGE3_CACHE");
        stage3_loop_edge_count = add_cached_loop_edges(
            optimizer_stage3,
            all_vertices,
            cached_loop_constraints_stage3,
            nullptr,
            &outage_mask,
            370000,
            "STAGE3_LOOP");

        AINFO << "[STAGE3] Graph summary:"
              << " lio=" << stage3_lio_edge_count
              << ", z_smooth=" << stage3_height_smooth_edge_count
              << ", gps_xy=" << stage3_gps_pos_edge_count
              << ", gps_heading=" << stage3_gps_heading_edge_count
              << ", gps_z=" << stage3_gps_height_edge_count
              << ", docking=" << stage3_docking_edge_count
              << ", block_prior=" << stage3_block_prior_edge_count
              << ", loops=" << stage3_loop_edge_count;

        if (run_optimizer(optimizer_stage3, "STAGE3", options_.max_iterations)) {
            for (size_t i = 0; i < keyframes_.size(); ++i) {
                keyframes_[i]->SetOptPose(all_vertices[i]->Estimate());
            }
        }
    }

    AINFO << "[GRAPH] Round1 support_lio=" << round1_lio_edge_count
          << ", support_loops=" << round1_loop_edge_count
          << ", gps_xy=" << round1_gps_pos_edge_count
          << ", gps_heading=" << round1_gps_heading_edge_count
          << ", gps_z=" << round1_gps_height_edge_count;
    AINFO << "[GRAPH] Round2 docking_blocks=" << round2_blocks_docked
          << ", docking_constraints=" << round2_docking_edges
          << ", initialized_keyframes=" << round2_keyframes_initialized;
    AINFO << "[GRAPH] Round2 block_map_queries=" << round2_block_queries
          << ", block_map_constraints=" << round2_block_constraints;
    AINFO << "[GRAPH] Stage3 docking_edges=" << stage3_docking_edge_count
          << ", block_prior_edges=" << stage3_block_prior_edge_count
          << ", loop_edges=" << stage3_loop_edge_count
          << ", lio_edges=" << stage3_lio_edge_count;

    AINFO << "[QUALITY] ==================== Post-Optimization Diagnostics ====================";
    if (has_utm_offset_) {
        double sum_gps_err = 0.0;
        double max_gps_err = 0.0;
        int gps_count = 0;
        for (size_t i = 0; i < keyframes_.size(); ++i) {
            auto gps_data = keyframes_[i]->GetGpsData();
            if (!gps_data.has_gps) {
                continue;
            }
            const Vec3d gps_lio = gps_data.gps_utm_position - utm_to_lio_offset_;
            const Vec3d opt_pos = keyframes_[i]->GetOptPose().translation();
            if (!gps_lio.allFinite() || !opt_pos.allFinite()) {
                continue;
            }
            const double err_xy = (gps_lio.head<2>() - opt_pos.head<2>()).norm();
            sum_gps_err += err_xy;
            max_gps_err = std::max(max_gps_err, err_xy);
            ++gps_count;
        }
        if (gps_count > 0) {
            AINFO << "[QUALITY] GPS XY residual: mean=" << std::fixed << std::setprecision(3)
                  << (sum_gps_err / gps_count) << " m, max=" << max_gps_err << " m";
        }
    }

    if (!gps_height_targets.empty()) {
        double sum_z_err = 0.0;
        double max_z_err = 0.0;
        int z_count = 0;
        for (const auto& target : gps_height_targets) {
            const double opt_z = keyframes_[target.keyframe_index]->GetOptPose().translation().z();
            if (!std::isfinite(opt_z)) {
                continue;
            }
            const double err = std::abs(target.trend_height - opt_z);
            sum_z_err += err;
            max_z_err = std::max(max_z_err, err);
            ++z_count;
        }
        if (z_count > 0) {
            AINFO << "[QUALITY] GPS Z residual: mean=" << std::fixed << std::setprecision(3)
                  << (sum_z_err / z_count) << " m, max=" << max_z_err
                  << " m (direct=" << gps_height_direct_count
                  << ", interp=" << gps_height_interp_count << ")";
        }
    }

    {
        double sum_lio_err = 0.0;
        double max_lio_err = 0.0;
        double sum_lio_err_xy = 0.0;
        double max_lio_err_xy = 0.0;
        double sum_lio_err_z = 0.0;
        double max_lio_err_z = 0.0;
        int lio_count = 0;
        for (size_t i = 1; i < keyframes_.size(); ++i) {
            const SE3 rel_measured = keyframes_[i]->GetRelativeMotion();
            const SE3 rel_optimized = keyframes_[i - 1]->GetOptPose().inverse() * keyframes_[i]->GetOptPose();
            const Vec3d rel_translation_err = rel_optimized.translation() - rel_measured.translation();
            const double trans_err = rel_translation_err.norm();
            const double trans_err_xy = rel_translation_err.head<2>().norm();
            const double trans_err_z = std::abs(rel_translation_err.z());
            if (!std::isfinite(trans_err)) {
                continue;
            }
            sum_lio_err += trans_err;
            max_lio_err = std::max(max_lio_err, trans_err);
            sum_lio_err_xy += trans_err_xy;
            max_lio_err_xy = std::max(max_lio_err_xy, trans_err_xy);
            sum_lio_err_z += trans_err_z;
            max_lio_err_z = std::max(max_lio_err_z, trans_err_z);
            ++lio_count;
        }
        if (lio_count > 0) {
            AINFO << "[QUALITY] LIO consistency: mean=" << std::fixed << std::setprecision(4)
                  << (sum_lio_err / lio_count) << " m, max=" << max_lio_err << " m";
            AINFO << "[QUALITY] LIO consistency XY: mean=" << std::fixed << std::setprecision(4)
                  << (sum_lio_err_xy / lio_count) << " m, max=" << max_lio_err_xy << " m";
            AINFO << "[QUALITY] LIO consistency Z: mean=" << std::fixed << std::setprecision(4)
                  << (sum_lio_err_z / lio_count) << " m, max=" << max_lio_err_z << " m";
        }
    }

    {
        double sum_shift = 0.0;
        double max_shift = 0.0;
        for (size_t i = 0; i < keyframes_.size(); ++i) {
            const double shift = (keyframes_[i]->GetOptPose().translation() - keyframes_[i]->GetLIOPose().translation()).norm();
            if (!std::isfinite(shift)) {
                continue;
            }
            sum_shift += shift;
            max_shift = std::max(max_shift, shift);
        }
        AINFO << "[QUALITY] Trajectory shift: mean=" << std::fixed << std::setprecision(3)
              << (sum_shift / std::max<size_t>(keyframes_.size(), 1)) << " m, max=" << max_shift << " m";
    }

    AINFO << "[GpsFusionOptimizer] Updated " << keyframes_.size() << " keyframe poses";
    is_optimizing_.store(false);
}

}  // namespace lightning
