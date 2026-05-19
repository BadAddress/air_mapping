#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "modules/air_mapping/system/common/eigen_types.h"
#include "modules/air_mapping/system/common/keyframe.h"

namespace apollo {
namespace air_mapping {
namespace stage3 {

struct Stage2KeyframeRecord {
  unsigned long id = 0;
  double timestamp = 0.0;
  lightning::SE3 lio_pose;
  lightning::SE3 stage2_pose;
  lightning::SE3 utm_pose;
  std::string source_stage1_dir;
  std::string source_cloud_path;
  std::string source_covariance_path;
  bool has_gps = false;
};

struct Stage2RelativeEdge {
  unsigned long from_id = 0;
  unsigned long to_id = 0;
  size_t from_index = 0;
  size_t to_index = 0;
  lightning::SE3 measurement;
};

struct Stage2GpsAnchor {
  unsigned long keyframe_id = 0;
  size_t keyframe_index = 0;
  double timestamp = 0.0;
  int segment_id = -1;
  double stage2_weight = 1.0;
  lightning::Vec3d gps_utm_position = lightning::Vec3d::Zero();
  lightning::Vec3d gps_smooth_utm_position = lightning::Vec3d::Zero();
  lightning::Vec3d gps_std_dev = lightning::Vec3d::Zero();
  lightning::Vec3d stage3_information_diag = lightning::Vec3d::Zero();
  double stage3_ramp_scale = 1.0;
  bool stage3_used = false;
  double residual_before_m = 0.0;
  double residual_after_m = 0.0;
  double residual_stage2_recomputed_m = 0.0;
  double residual_stage3_m = 0.0;
};

struct Stage3OutageBlock {
  size_t start_index = 0;
  size_t end_index = 0;
  size_t representative_index = 0;
  int left_anchor_index = -1;
  int right_anchor_index = -1;
  double path_length_m = 0.0;
  bool icp_valid = false;
  lightning::SE3 representative_prior;
  double icp_fitness = -1.0;
  int source_keyframes = 0;
  int target_keyframes = 0;
};

struct Stage2Dataset {
  std::vector<Stage2KeyframeRecord> records;
  std::vector<lightning::Keyframe::Ptr> keyframes;
  std::vector<std::string> keyframe_cloud_paths;
  std::vector<std::string> keyframe_covariance_paths;
  std::vector<Stage2RelativeEdge> relative_edges;
  std::vector<Stage2GpsAnchor> gps_anchors;
  std::string source_stage1_dir;
  std::string source_config_path;
  lightning::Vec3d utm_origin = lightning::Vec3d::Zero();
  bool has_utm_origin = false;
};

struct Stage3RefineSummary {
  bool success = false;
  size_t keyframe_count = 0;
  size_t relative_edge_count = 0;
  size_t gps_prior_count = 0;
  size_t outage_block_count = 0;
  size_t outage_icp_prior_count = 0;
  int optimizer_iterations = 0;
  double chi2_before = 0.0;
  double chi2_after = 0.0;
  double mean_gps_residual_before_m = 0.0;
  double max_gps_residual_before_m = 0.0;
  double mean_gps_residual_after_m = 0.0;
  double max_gps_residual_after_m = 0.0;
  double mean_pose_delta_m = 0.0;
  double max_pose_delta_m = 0.0;
  double mean_rotation_delta_deg = 0.0;
  double max_rotation_delta_deg = 0.0;
};

}  // namespace stage3
}  // namespace air_mapping
}  // namespace apollo
