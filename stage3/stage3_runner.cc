#include "modules/air_mapping/stage3/stage3_runner.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "pcl/common/transforms.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/registration/icp.h"

#include "cyber/common/log.h"
#include "modules/air_mapping/stage3/stage2_artifact_reader.h"
#include "modules/air_mapping/stage3/stage3_artifact_writer.h"
#include "modules/air_mapping/stage3/stage3_types.h"
#include "modules/air_mapping/system/core/miao/core/graph/base_binary_edge.h"
#include "modules/air_mapping/system/core/miao/core/opti_algo/algo_select.h"
#include "modules/air_mapping/system/core/miao/core/robust_kernel/huber.h"
#include "modules/air_mapping/system/core/miao/core/types/edge_gps_prior.h"
#include "modules/air_mapping/system/core/miao/core/types/edge_se3_prior.h"
#include "modules/air_mapping/system/core/miao/core/types/vertex_se3.h"

namespace apollo {
namespace air_mapping {
namespace stage3 {

namespace {

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
namespace miao = lightning::miao;

class EdgeOutageBlockRelative
    : public miao::BaseBinaryEdge<6, lightning::SE3, miao::VertexSE3,
                                  miao::VertexSE3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void SetOffsets(const lightning::SE3& from_offset,
                  const lightning::SE3& to_offset) {
    from_offset_ = from_offset;
    to_offset_ = to_offset;
  }

  void ComputeError() override {
    const auto from =
        static_cast<miao::VertexSE3*>(vertices_[0])->Estimate() * from_offset_;
    const auto to =
        static_cast<miao::VertexSE3*>(vertices_[1])->Estimate() * to_offset_;
    const lightning::SE3 predicted = from.inverse() * to;
    const lightning::SE3 delta = measurement_.inverse() * predicted;
    error_.head<3>() = delta.translation();
    error_.tail<3>() = delta.so3().unit_quaternion().coeffs().head<3>();
  }

 private:
  lightning::SE3 from_offset_;
  lightning::SE3 to_offset_;
};

lightning::Mat6d BuildSe3Information(double translation_sigma_m,
                                     double rotation_sigma_rad) {
  lightning::Mat6d information = lightning::Mat6d::Identity();
  const double translation_var =
      std::max(translation_sigma_m * translation_sigma_m, 1e-8);
  const double rotation_var =
      std::max(rotation_sigma_rad * rotation_sigma_rad, 1e-8);
  information.block<3, 3>(0, 0) *= 1.0 / translation_var;
  information.block<3, 3>(3, 3) *= 1.0 / rotation_var;
  return information;
}

lightning::Mat3d BuildGpsInformation(const lightning::Vec3d& std_dev,
                                     const GraphRefineConfig& config,
                                     double ramp_scale) {
  const double std_x = std::max(std::abs(std_dev.x()), config.min_gps_std_xy);
  const double std_y = std::max(std::abs(std_dev.y()), config.min_gps_std_xy);
  const double std_z = std::max(std::abs(std_dev.z()), config.min_gps_std_z);

  lightning::Mat3d information = lightning::Mat3d::Zero();
  information(0, 0) = std::min(1.0 / (std_x * std_x), config.max_gps_info_xy);
  information(1, 1) = std::min(1.0 / (std_y * std_y), config.max_gps_info_xy);
  if (config.use_gps_z) {
    information(2, 2) = std::min(1.0 / (std_z * std_z), config.max_gps_info_z);
  }
  return information * std::max(config.gps_weight_scale, 1e-6) *
         std::clamp(ramp_scale, 0.0, 1.0);
}

lightning::SE3 ToLocalUtmPose(const lightning::SE3& absolute_utm_pose,
                              const lightning::Vec3d& utm_origin) {
  return lightning::SE3(absolute_utm_pose.so3(),
                        absolute_utm_pose.translation() - utm_origin);
}

double DistanceXY(const lightning::Vec3d& lhs, const lightning::Vec3d& rhs) {
  return (lhs.head<2>() - rhs.head<2>()).norm();
}

double PathLength(const Stage2Dataset& dataset, size_t start_index,
                  size_t end_index) {
  if (start_index >= dataset.keyframes.size() ||
      end_index >= dataset.keyframes.size() || end_index <= start_index) {
    return 0.0;
  }
  double length = 0.0;
  for (size_t i = start_index + 1; i <= end_index; ++i) {
    if (!dataset.keyframes[i - 1] || !dataset.keyframes[i]) {
      continue;
    }
    length += (dataset.keyframes[i]->GetOptPose().translation() -
               dataset.keyframes[i - 1]->GetOptPose().translation())
                  .norm();
  }
  return length;
}

bool IsHighPrecisionGpsAnchor(const Stage2GpsAnchor& anchor,
                              const GraphRefineConfig& config) {
  if (config.max_fused_gps_std_xy <= 1e-6) {
    return true;
  }
  return std::abs(anchor.gps_std_dev.x()) <= config.max_fused_gps_std_xy &&
         std::abs(anchor.gps_std_dev.y()) <= config.max_fused_gps_std_xy;
}

std::vector<bool> BuildGpsSupportMask(const Stage2Dataset& dataset,
                                      const GraphRefineConfig& config) {
  std::vector<bool> supported(dataset.keyframes.size(), false);
  std::vector<Stage2GpsAnchor> anchors;
  for (const auto& anchor : dataset.gps_anchors) {
    if (IsHighPrecisionGpsAnchor(anchor, config)) {
      anchors.push_back(anchor);
    }
  }
  std::sort(anchors.begin(), anchors.end(),
            [](const Stage2GpsAnchor& lhs, const Stage2GpsAnchor& rhs) {
              return lhs.keyframe_index < rhs.keyframe_index;
            });
  for (const auto& anchor : anchors) {
    if (anchor.keyframe_index < supported.size()) {
      supported[anchor.keyframe_index] = true;
    }
  }
  for (size_t i = 1; i < anchors.size(); ++i) {
    const auto& previous = anchors[i - 1];
    const auto& current = anchors[i];
    if (previous.segment_id != current.segment_id ||
        previous.keyframe_index >= current.keyframe_index ||
        current.keyframe_index >= supported.size()) {
      continue;
    }
    if (config.gps_support_max_anchor_gap_m > 1e-6 &&
        PathLength(dataset, previous.keyframe_index, current.keyframe_index) >
            config.gps_support_max_anchor_gap_m) {
      continue;
    }
    for (size_t keyframe_index = previous.keyframe_index;
         keyframe_index <= current.keyframe_index; ++keyframe_index) {
      supported[keyframe_index] = true;
    }
  }
  return supported;
}

double ComputeGpsRampScale(const Stage2Dataset& dataset,
                           const Stage2GpsAnchor& anchor,
                           const GraphRefineConfig& config,
                           const std::vector<bool>& gps_supported) {
  if (config.gps_boundary_ramp_distance_m <= 1e-6) {
    return 1.0;
  }
  if (dataset.records.size() != gps_supported.size() ||
      anchor.keyframe_index >= gps_supported.size()) {
    return 1.0;
  }

  double distance_to_left_gap = config.gps_boundary_ramp_distance_m;
  if (anchor.keyframe_index > 0) {
    if (!gps_supported[anchor.keyframe_index - 1]) {
      distance_to_left_gap = 0.0;
    } else {
      for (size_t i = anchor.keyframe_index; i > 0; --i) {
        if (!gps_supported[i - 1]) {
          break;
        }
        distance_to_left_gap =
            std::min(config.gps_boundary_ramp_distance_m,
                     PathLength(dataset, i - 1, anchor.keyframe_index));
        if (distance_to_left_gap >= config.gps_boundary_ramp_distance_m) {
          break;
        }
      }
    }
  }

  double distance_to_right_gap = config.gps_boundary_ramp_distance_m;
  if (anchor.keyframe_index + 1 < dataset.records.size()) {
    if (!gps_supported[anchor.keyframe_index + 1]) {
      distance_to_right_gap = 0.0;
    } else {
      for (size_t i = anchor.keyframe_index + 1; i < dataset.records.size();
           ++i) {
        if (!gps_supported[i]) {
          break;
        }
        distance_to_right_gap =
            std::min(config.gps_boundary_ramp_distance_m,
                     PathLength(dataset, anchor.keyframe_index, i));
        if (distance_to_right_gap >= config.gps_boundary_ramp_distance_m) {
          break;
        }
      }
    }
  }

  const double boundary_distance =
      std::min(distance_to_left_gap, distance_to_right_gap);
  return std::clamp(boundary_distance / config.gps_boundary_ramp_distance_m,
                    0.2, 1.0);
}

lightning::CloudPtr DownsampleCloud(const lightning::CloudPtr& input,
                                    double voxel_size_m) {
  lightning::CloudPtr output(new lightning::PointCloudType);
  if (!input || input->empty()) {
    return output;
  }

  pcl::VoxelGrid<lightning::PointType> voxel;
  const float leaf = static_cast<float>(std::max(voxel_size_m, 0.01));
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.setInputCloud(input);
  voxel.filter(*output);
  output->is_dense = false;
  output->height = 1;
  output->width = output->size();
  return output;
}

Eigen::Matrix4d LidarExtrinsicMatrix(const lightning::Keyframe::Ptr& keyframe) {
  Eigen::Matrix4d t_imu_lidar = Eigen::Matrix4d::Identity();
  if (!keyframe) {
    return t_imu_lidar;
  }
  const auto state = keyframe->GetState();
  t_imu_lidar.block<3, 3>(0, 0) = state.offset_R_lidar_.matrix();
  t_imu_lidar.block<3, 1>(0, 3) = state.offset_t_lidar_;
  return t_imu_lidar;
}

lightning::CloudPtr BuildSubmap(const Stage2Dataset& dataset,
                                const std::vector<size_t>& indices,
                                double voxel_size_m) {
  lightning::CloudPtr submap(new lightning::PointCloudType);
  for (const size_t index : indices) {
    if (index >= dataset.keyframes.size() || !dataset.keyframes[index] ||
        !dataset.keyframes[index]->GetCloud() ||
        dataset.keyframes[index]->GetCloud()->empty()) {
      continue;
    }
    const Eigen::Matrix4d t_world_lidar =
        dataset.keyframes[index]->GetOptPose().matrix() *
        LidarExtrinsicMatrix(dataset.keyframes[index]);
    lightning::CloudPtr transformed(new lightning::PointCloudType);
    pcl::transformPointCloud(*dataset.keyframes[index]->GetCloud(),
                             *transformed, t_world_lidar);
    *submap += *transformed;
  }
  submap->is_dense = false;
  submap->height = 1;
  submap->width = submap->size();
  return DownsampleCloud(submap, voxel_size_m);
}

std::vector<Stage3OutageBlock> BuildOutageBlocks(
    const Stage2Dataset& dataset, const GraphRefineConfig& config,
    const std::vector<bool>& gps_supported) {
  std::vector<Stage3OutageBlock> blocks;
  if (!config.enable_outage_blocks || dataset.keyframes.empty() ||
      dataset.records.size() != dataset.keyframes.size() ||
      gps_supported.size() != dataset.keyframes.size()) {
    return blocks;
  }

  int last_gps_index = -1;
  size_t i = 0;
  while (i < dataset.records.size()) {
    while (i < dataset.records.size() && gps_supported[i]) {
      last_gps_index = static_cast<int>(i);
      ++i;
    }
    if (i >= dataset.records.size()) {
      break;
    }

    const size_t outage_start = i;
    while (i < dataset.records.size() && !gps_supported[i]) {
      ++i;
    }
    const size_t outage_end = i - 1;
    const int right_gps_index =
        (i < dataset.records.size()) ? static_cast<int>(i) : -1;
    const double outage_length = PathLength(dataset, outage_start, outage_end);
    const size_t outage_count = outage_end - outage_start + 1;
    if (outage_count < static_cast<size_t>(config.outage_min_keyframes) ||
        outage_length < config.outage_min_length_m) {
      continue;
    }

    if (config.outage_max_block_length_m <= 1e-6) {
      Stage3OutageBlock block;
      block.start_index = outage_start;
      block.end_index = outage_end;
      block.representative_index = (outage_start + outage_end) / 2;
      block.left_anchor_index = last_gps_index;
      block.right_anchor_index = right_gps_index;
      block.path_length_m = outage_length;
      block.representative_prior =
          dataset.keyframes[block.representative_index]->GetOptPose();
      blocks.push_back(block);
      continue;
    }

    size_t block_start = outage_start;
    while (block_start <= outage_end) {
      size_t block_end = block_start;
      while (block_end + 1 <= outage_end &&
             PathLength(dataset, block_start, block_end + 1) <=
                 config.outage_max_block_length_m) {
        ++block_end;
      }

      Stage3OutageBlock block;
      block.start_index = block_start;
      block.end_index = block_end;
      block.representative_index = (block_start + block_end) / 2;
      block.left_anchor_index = last_gps_index;
      block.right_anchor_index = right_gps_index;
      block.path_length_m = PathLength(dataset, block_start, block_end);
      block.representative_prior =
          dataset.keyframes[block.representative_index]->GetOptPose();
      blocks.push_back(block);

      if (block_end == outage_end) {
        break;
      }
      block_start = block_end + 1;
    }
  }
  return blocks;
}

std::vector<size_t> BuildBlockSourceIndices(const Stage3OutageBlock& block) {
  std::vector<size_t> indices;
  for (size_t i = block.start_index; i <= block.end_index; ++i) {
    indices.push_back(i);
  }
  return indices;
}

std::vector<size_t> BuildSupportIndices(
    const Stage2Dataset& dataset, const Stage3OutageBlock& block,
    const GraphRefineConfig& config, const std::vector<bool>& gps_supported) {
  std::vector<size_t> indices;
  if (dataset.keyframes.empty() ||
      gps_supported.size() != dataset.keyframes.size() ||
      block.representative_index >= dataset.keyframes.size() ||
      !dataset.keyframes[block.representative_index]) {
    return indices;
  }

  const lightning::Vec3d center =
      dataset.keyframes[block.representative_index]->GetOptPose().translation();
  const int overlap = std::max(config.outage_support_overlap_keyframes, 0);
  const int left_limit =
      std::max(0, static_cast<int>(block.start_index) - overlap);
  const int right_limit =
      std::min(static_cast<int>(dataset.keyframes.size()) - 1,
               static_cast<int>(block.end_index) + overlap);

  for (size_t i = 0; i < dataset.keyframes.size(); ++i) {
    if (i >= block.start_index && i <= block.end_index) {
      continue;
    }
    if (!dataset.keyframes[i] || !gps_supported[i]) {
      continue;
    }
    const bool near_boundary =
        static_cast<int>(i) >= left_limit && static_cast<int>(i) <= right_limit;
    const bool within_radius =
        DistanceXY(dataset.keyframes[i]->GetOptPose().translation(), center) <=
        config.outage_support_radius_m;
    if (near_boundary || within_radius) {
      indices.push_back(i);
    }
  }
  return indices;
}

void MatchOutageBlocksWithSupport(const Stage2Dataset& dataset,
                                  const GraphRefineConfig& config,
                                  const std::vector<bool>& gps_supported,
                                  std::vector<Stage3OutageBlock>* blocks) {
  if (blocks == nullptr || !config.enable_outage_blocks) {
    return;
  }

  for (auto& block : *blocks) {
    const auto source_indices = BuildBlockSourceIndices(block);
    const auto target_indices =
        BuildSupportIndices(dataset, block, config, gps_supported);
    block.source_keyframes = static_cast<int>(source_indices.size());
    block.target_keyframes = static_cast<int>(target_indices.size());
    if (source_indices.empty() || target_indices.empty()) {
      continue;
    }

    const auto source =
        BuildSubmap(dataset, source_indices, config.outage_source_voxel_size_m);
    const auto target =
        BuildSubmap(dataset, target_indices, config.outage_target_voxel_size_m);
    if (!source || source->empty() || !target || target->empty()) {
      continue;
    }

    pcl::IterativeClosestPoint<lightning::PointType, lightning::PointType> icp;
    icp.setMaximumIterations(config.outage_icp_max_iterations);
    icp.setMaxCorrespondenceDistance(
        static_cast<float>(config.outage_icp_max_corr_dist_m));
    icp.setInputSource(source);
    icp.setInputTarget(target);

    lightning::CloudPtr aligned(new lightning::PointCloudType);
    icp.align(*aligned, Eigen::Matrix4f::Identity());
    if (!icp.hasConverged()) {
      continue;
    }
    const double fitness = icp.getFitnessScore();
    if (!std::isfinite(fitness) ||
        fitness > config.outage_icp_fitness_threshold) {
      continue;
    }

    const Eigen::Matrix4d delta = icp.getFinalTransformation().cast<double>();
    if (!delta.allFinite()) {
      continue;
    }
    const auto representative_pose =
        dataset.keyframes[block.representative_index]->GetOptPose();
    const Eigen::Matrix4d matched = delta * representative_pose.matrix();
    block.representative_prior =
        lightning::SE3(lightning::SO3::fitToSO3(matched.block<3, 3>(0, 0)),
                       matched.block<3, 1>(0, 3));
    block.icp_fitness = fitness;
    block.icp_valid = true;
  }
}

void FillResidualSummary(Stage2Dataset* dataset, Stage3RefineSummary* summary) {
  if (dataset == nullptr || summary == nullptr) {
    return;
  }
  if (dataset->records.size() != dataset->keyframes.size()) {
    AERROR << "[Stage3] Invalid dataset size: records="
           << dataset->records.size()
           << ", keyframes=" << dataset->keyframes.size();
    return;
  }

  double gps_sum_before = 0.0;
  double gps_max_before = 0.0;
  double gps_sum_after = 0.0;
  double gps_max_after = 0.0;
  for (auto& anchor : dataset->gps_anchors) {
    if (anchor.keyframe_index >= dataset->keyframes.size() ||
        !dataset->keyframes[anchor.keyframe_index]) {
      continue;
    }
    const auto stage2_local = ToLocalUtmPose(
        dataset->records[anchor.keyframe_index].utm_pose, dataset->utm_origin);
    const lightning::Vec3d target =
        anchor.gps_smooth_utm_position - dataset->utm_origin;
    const double before = (stage2_local.translation() - target).norm();
    const double after =
        (dataset->keyframes[anchor.keyframe_index]->GetOptPose().translation() -
         target)
            .norm();
    anchor.residual_stage2_recomputed_m = before;
    anchor.residual_stage3_m = after;
    gps_sum_before += before;
    gps_max_before = std::max(gps_max_before, before);
    gps_sum_after += after;
    gps_max_after = std::max(gps_max_after, after);
  }

  const double gps_count =
      static_cast<double>(std::max<size_t>(dataset->gps_anchors.size(), 1));
  summary->mean_gps_residual_before_m = gps_sum_before / gps_count;
  summary->max_gps_residual_before_m = gps_max_before;
  summary->mean_gps_residual_after_m = gps_sum_after / gps_count;
  summary->max_gps_residual_after_m = gps_max_after;

  double pose_delta_sum = 0.0;
  double pose_delta_max = 0.0;
  double rotation_delta_sum = 0.0;
  double rotation_delta_max = 0.0;
  size_t pose_count = 0;
  for (size_t i = 0; i < dataset->keyframes.size(); ++i) {
    if (!dataset->keyframes[i]) {
      continue;
    }
    const auto refined_pose = dataset->keyframes[i]->GetOptPose();
    const auto stage2_pose = dataset->records[i].stage2_pose;
    const auto delta = stage2_pose.inverse() * refined_pose;
    const double translation_delta = delta.translation().norm();
    const double rotation_delta = delta.so3().log().norm() * kRadToDeg;
    pose_delta_sum += translation_delta;
    pose_delta_max = std::max(pose_delta_max, translation_delta);
    rotation_delta_sum += rotation_delta;
    rotation_delta_max = std::max(rotation_delta_max, rotation_delta);
    ++pose_count;
  }

  const double count = static_cast<double>(std::max<size_t>(pose_count, 1));
  summary->mean_pose_delta_m = pose_delta_sum / count;
  summary->max_pose_delta_m = pose_delta_max;
  summary->mean_rotation_delta_deg = rotation_delta_sum / count;
  summary->max_rotation_delta_deg = rotation_delta_max;
}

bool RunGraphRefinement(const Stage3Config& config, Stage2Dataset* dataset,
                        Stage3RefineSummary* summary,
                        std::vector<Stage3OutageBlock>* outage_blocks) {
  if (dataset == nullptr || summary == nullptr || outage_blocks == nullptr ||
      dataset->keyframes.empty()) {
    return false;
  }
  if (dataset->records.size() != dataset->keyframes.size()) {
    AERROR << "[Stage3] Invalid dataset size: records="
           << dataset->records.size()
           << ", keyframes=" << dataset->keyframes.size();
    return false;
  }

  *summary = Stage3RefineSummary();
  summary->keyframe_count = dataset->keyframes.size();

  const std::vector<bool> gps_supported =
      BuildGpsSupportMask(*dataset, config.graph);
  *outage_blocks = BuildOutageBlocks(*dataset, config.graph, gps_supported);
  MatchOutageBlocksWithSupport(*dataset, config.graph, gps_supported,
                               outage_blocks);
  summary->outage_block_count = outage_blocks->size();
  for (const auto& block : *outage_blocks) {
    if (block.icp_valid) {
      ++summary->outage_icp_prior_count;
    }
  }

  std::vector<int> keyframe_to_block(dataset->keyframes.size(), -1);
  for (size_t block_index = 0; block_index < outage_blocks->size();
       ++block_index) {
    const auto& block = (*outage_blocks)[block_index];
    for (size_t keyframe_index = block.start_index;
         keyframe_index <= block.end_index &&
         keyframe_index < keyframe_to_block.size();
         ++keyframe_index) {
      keyframe_to_block[keyframe_index] = static_cast<int>(block_index);
    }
  }

  miao::OptimizerConfig optimizer_config(
      miao::AlgorithmType::LEVENBERG_MARQUARDT,
      miao::LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN, false);
  auto optimizer =
      miao::SetupOptimizer<Eigen::Dynamic, Eigen::Dynamic>(optimizer_config);
  optimizer->SetVerbose(config.graph.verbose);

  std::vector<std::shared_ptr<miao::VertexSE3>> vertices;
  std::vector<lightning::SE3> vertex_initial_poses;
  std::vector<int> keyframe_vertex_ids(dataset->keyframes.size(), -1);
  std::vector<int> block_vertex_ids(outage_blocks->size(), -1);

  auto add_vertex = [&](const lightning::SE3& initial_pose) {
    auto vertex = std::make_shared<miao::VertexSE3>();
    const int vertex_id = static_cast<int>(vertices.size());
    vertex->SetId(vertex_id);
    vertex->SetEstimate(initial_pose);
    optimizer->AddVertex(vertex);
    vertices.push_back(vertex);
    vertex_initial_poses.push_back(initial_pose);
    return vertex_id;
  };

  for (size_t i = 0; i < dataset->keyframes.size(); ++i) {
    if (keyframe_to_block[i] >= 0) {
      continue;
    }
    keyframe_vertex_ids[i] = add_vertex(dataset->keyframes[i]->GetOptPose());
  }

  for (size_t block_index = 0; block_index < outage_blocks->size();
       ++block_index) {
    const auto& block = (*outage_blocks)[block_index];
    if (block.representative_index >= dataset->keyframes.size() ||
        !dataset->keyframes[block.representative_index]) {
      continue;
    }
    block_vertex_ids[block_index] = add_vertex(
        dataset->keyframes[block.representative_index]->GetOptPose());
  }

  auto vertex_for_keyframe = [&](size_t keyframe_index) -> int {
    if (keyframe_index >= keyframe_vertex_ids.size()) {
      return -1;
    }
    const int block_index = keyframe_to_block[keyframe_index];
    if (block_index >= 0 &&
        block_index < static_cast<int>(block_vertex_ids.size())) {
      return block_vertex_ids[block_index];
    }
    return keyframe_vertex_ids[keyframe_index];
  };

  auto fixed_offset_for_keyframe =
      [&](size_t keyframe_index) -> lightning::SE3 {
    const int block_index = keyframe_to_block[keyframe_index];
    if (block_index >= 0 &&
        block_index < static_cast<int>(outage_blocks->size())) {
      const auto& block = (*outage_blocks)[block_index];
      if (block.representative_index < dataset->records.size()) {
        return dataset->records[block.representative_index]
                   .stage2_pose.inverse() *
               dataset->records[keyframe_index].stage2_pose;
      }
    }
    return lightning::SE3();
  };

  auto add_relative_edge = [&](int from_vertex_id, int to_vertex_id,
                               const lightning::SE3& from_offset,
                               const lightning::SE3& to_offset,
                               const lightning::SE3& measurement,
                               const lightning::Mat6d& information,
                               double huber_delta, int* edge_id) {
    if (from_vertex_id < 0 || to_vertex_id < 0 ||
        from_vertex_id >= static_cast<int>(vertices.size()) ||
        to_vertex_id >= static_cast<int>(vertices.size()) ||
        from_vertex_id == to_vertex_id) {
      return false;
    }
    auto edge = std::make_shared<EdgeOutageBlockRelative>();
    edge->SetId((*edge_id)++);
    edge->SetVertex(0, vertices[from_vertex_id]);
    edge->SetVertex(1, vertices[to_vertex_id]);
    edge->SetOffsets(from_offset, to_offset);
    edge->SetMeasurement(measurement);
    edge->SetInformation(information);
    auto huber = std::make_shared<miao::RobustKernelHuber>();
    huber->SetDelta(huber_delta);
    edge->SetRobustKernel(huber);
    optimizer->AddEdge(edge);
    return true;
  };

  if (vertices.empty()) {
    AERROR << "[Stage3] No graph vertices were created";
    return false;
  }

  const lightning::Mat6d odom_information =
      BuildSe3Information(config.graph.lio_translation_sigma_m,
                          config.graph.lio_rotation_sigma_deg * kDegToRad);
  int edge_id = 1;
  for (const auto& relative_edge : dataset->relative_edges) {
    if (relative_edge.from_index >= dataset->keyframes.size() ||
        relative_edge.to_index >= dataset->keyframes.size()) {
      continue;
    }
    const int from_vertex_id = vertex_for_keyframe(relative_edge.from_index);
    const int to_vertex_id = vertex_for_keyframe(relative_edge.to_index);
    if (from_vertex_id == to_vertex_id) {
      continue;
    }

    if (add_relative_edge(from_vertex_id, to_vertex_id,
                          fixed_offset_for_keyframe(relative_edge.from_index),
                          fixed_offset_for_keyframe(relative_edge.to_index),
                          relative_edge.measurement, odom_information,
                          config.graph.lio_huber_delta, &edge_id)) {
      ++summary->relative_edge_count;
    }
  }

  int gauge_prior_vertex_id = -1;
  for (size_t i = 0; i < keyframe_vertex_ids.size(); ++i) {
    if (keyframe_to_block[i] < 0 && keyframe_vertex_ids[i] >= 0) {
      gauge_prior_vertex_id = keyframe_vertex_ids[i];
      break;
    }
  }
  if (gauge_prior_vertex_id < 0) {
    gauge_prior_vertex_id =
        vertex_for_keyframe(0) >= 0 ? vertex_for_keyframe(0) : 0;
  }
  if (gauge_prior_vertex_id >= 0 &&
      gauge_prior_vertex_id < static_cast<int>(vertices.size())) {
    auto prior = std::make_shared<miao::EdgeSE3Prior>();
    prior->SetId(500000);
    prior->SetVertex(0, vertices[gauge_prior_vertex_id]);
    prior->SetMeasurement(vertex_initial_poses[gauge_prior_vertex_id]);
    prior->SetInformation(BuildSe3Information(
        config.graph.first_pose_prior_translation_sigma_m,
        config.graph.first_pose_prior_rotation_sigma_deg * kDegToRad));
    auto huber = std::make_shared<miao::RobustKernelHuber>();
    huber->SetDelta(config.graph.first_pose_prior_huber_delta);
    prior->SetRobustKernel(huber);
    optimizer->AddEdge(prior);
  }

  int gps_edge_id = 1000000;
  for (auto& anchor : dataset->gps_anchors) {
    if (anchor.keyframe_index >= keyframe_to_block.size()) {
      continue;
    }
    const double ramp_scale =
        ComputeGpsRampScale(*dataset, anchor, config.graph, gps_supported);
    const lightning::Mat3d gps_information =
        BuildGpsInformation(anchor.gps_std_dev, config.graph, ramp_scale);
    anchor.stage3_ramp_scale = ramp_scale;
    anchor.stage3_information_diag = gps_information.diagonal();
    anchor.stage3_used = false;
    if (!IsHighPrecisionGpsAnchor(anchor, config.graph)) {
      anchor.stage3_ramp_scale = 0.0;
      anchor.stage3_information_diag.setZero();
      continue;
    }
    if (keyframe_to_block[anchor.keyframe_index] >= 0) {
      anchor.stage3_ramp_scale = 0.0;
      anchor.stage3_information_diag.setZero();
      continue;
    }

    const int vertex_id = vertex_for_keyframe(anchor.keyframe_index);
    if (vertex_id < 0 || vertex_id >= static_cast<int>(vertices.size())) {
      continue;
    }
    const lightning::Vec3d measurement =
        anchor.gps_smooth_utm_position - dataset->utm_origin;
    if (!measurement.allFinite()) {
      continue;
    }
    auto edge = std::make_shared<miao::EdgeGPSPrior>();
    edge->SetId(gps_edge_id++);
    edge->SetVertex(0, vertices[vertex_id]);
    edge->SetMeasurement(measurement);
    edge->SetInformation(gps_information);
    auto huber = std::make_shared<miao::RobustKernelHuber>();
    huber->SetDelta(config.graph.gps_huber_delta);
    edge->SetRobustKernel(huber);
    optimizer->AddEdge(edge);
    anchor.stage3_used = true;
    ++summary->gps_prior_count;
  }

  int outage_prior_edge_id = 2000000;
  const lightning::Mat6d outage_icp_information = BuildSe3Information(
      config.graph.outage_icp_translation_sigma_m,
      config.graph.outage_icp_rotation_sigma_deg * kDegToRad);
  for (size_t block_index = 0; block_index < outage_blocks->size();
       ++block_index) {
    const auto& block = (*outage_blocks)[block_index];
    if (!block.icp_valid || block_index >= block_vertex_ids.size()) {
      continue;
    }
    const int vertex_id = block_vertex_ids[block_index];
    if (vertex_id < 0 || vertex_id >= static_cast<int>(vertices.size())) {
      continue;
    }
    auto prior = std::make_shared<miao::EdgeSE3Prior>();
    prior->SetId(outage_prior_edge_id++);
    prior->SetVertex(0, vertices[vertex_id]);
    prior->SetMeasurement(block.representative_prior);
    prior->SetInformation(outage_icp_information);
    auto huber = std::make_shared<miao::RobustKernelHuber>();
    huber->SetDelta(config.graph.outage_icp_huber_delta);
    prior->SetRobustKernel(huber);
    optimizer->AddEdge(prior);
  }

  if (summary->relative_edge_count == 0 || summary->gps_prior_count == 0) {
    AERROR << "[Stage3] Graph needs both relative edges and GPS priors, got "
           << "relative_edges=" << summary->relative_edge_count
           << ", gps_priors=" << summary->gps_prior_count;
    return false;
  }

  optimizer->InitializeOptimization();
  optimizer->ComputeActiveErrors();
  summary->chi2_before = optimizer->ActiveChi2();
  if (!std::isfinite(summary->chi2_before)) {
    AERROR << "[Stage3] chi2_before is invalid";
    return false;
  }

  summary->optimizer_iterations =
      optimizer->Optimize(std::max(config.graph.max_iterations, 1));
  optimizer->ComputeActiveErrors();
  summary->chi2_after = optimizer->ActiveChi2();
  if (summary->optimizer_iterations <= 0 ||
      !std::isfinite(summary->chi2_after)) {
    AERROR << "[Stage3] optimizer failed";
    return false;
  }

  for (size_t i = 0; i < dataset->keyframes.size(); ++i) {
    if (!dataset->keyframes[i] || keyframe_to_block[i] >= 0) {
      continue;
    }
    const int vertex_id = keyframe_vertex_ids[i];
    if (vertex_id >= 0 && vertex_id < static_cast<int>(vertices.size())) {
      dataset->keyframes[i]->SetOptPose(vertices[vertex_id]->Estimate());
    }
  }

  for (size_t block_index = 0; block_index < outage_blocks->size();
       ++block_index) {
    const auto& block = (*outage_blocks)[block_index];
    if (block.representative_index >= dataset->keyframes.size() ||
        block_index >= block_vertex_ids.size()) {
      continue;
    }
    const int vertex_id = block_vertex_ids[block_index];
    if (vertex_id < 0 || vertex_id >= static_cast<int>(vertices.size())) {
      continue;
    }
    const auto representative_initial =
        dataset->records[block.representative_index].stage2_pose;
    const auto representative_optimized = vertices[vertex_id]->Estimate();
    const auto block_delta =
        representative_optimized * representative_initial.inverse();
    for (size_t keyframe_index = block.start_index;
         keyframe_index <= block.end_index &&
         keyframe_index < dataset->keyframes.size();
         ++keyframe_index) {
      if (dataset->keyframes[keyframe_index]) {
        dataset->keyframes[keyframe_index]->SetOptPose(
            block_delta * dataset->records[keyframe_index].stage2_pose);
      }
    }
  }

  summary->success = true;
  FillResidualSummary(dataset, summary);
  AINFO << "[Stage3] Graph refinement finished:"
        << " relative_edges=" << summary->relative_edge_count
        << ", gps_priors=" << summary->gps_prior_count
        << ", outage_blocks=" << summary->outage_block_count
        << ", outage_icp_priors=" << summary->outage_icp_prior_count
        << ", iterations=" << summary->optimizer_iterations
        << ", chi2=" << summary->chi2_before << " -> " << summary->chi2_after
        << ", gps_mean=" << summary->mean_gps_residual_before_m << " -> "
        << summary->mean_gps_residual_after_m << "m";
  return true;
}

lightning::CloudPtr BuildPreviewMap(const Stage3Config& config,
                                    const Stage2Dataset& dataset) {
  lightning::CloudPtr global_map(new lightning::PointCloudType);
  pcl::VoxelGrid<lightning::PointType> voxel;
  voxel.setLeafSize(config.output.preview_voxel_size,
                    config.output.preview_voxel_size,
                    config.output.preview_voxel_size);

  const size_t step =
      static_cast<size_t>(std::max(config.output.preview_keyframe_step, 1));
  for (size_t i = 0; i < dataset.keyframes.size(); i += step) {
    const auto& keyframe = dataset.keyframes[i];
    if (!keyframe || !keyframe->GetCloud() || keyframe->GetCloud()->empty()) {
      continue;
    }

    lightning::CloudPtr cloud_filtered(new lightning::PointCloudType);
    voxel.setInputCloud(keyframe->GetCloud());
    voxel.filter(*cloud_filtered);

    const auto state = keyframe->GetState();
    Eigen::Matrix4d t_imu_lidar = Eigen::Matrix4d::Identity();
    t_imu_lidar.block<3, 3>(0, 0) = state.offset_R_lidar_.matrix();
    t_imu_lidar.block<3, 1>(0, 3) = state.offset_t_lidar_;
    const Eigen::Matrix4d t_world_lidar =
        keyframe->GetOptPose().matrix() * t_imu_lidar;

    lightning::CloudPtr cloud_transformed(new lightning::PointCloudType);
    pcl::transformPointCloud(*cloud_filtered, *cloud_transformed,
                             t_world_lidar);
    *global_map += *cloud_transformed;
  }

  if (!global_map->empty()) {
    lightning::CloudPtr global_filtered(new lightning::PointCloudType);
    voxel.setInputCloud(global_map);
    voxel.filter(*global_filtered);
    global_filtered->is_dense = false;
    global_filtered->height = 1;
    global_filtered->width = global_filtered->size();
    return global_filtered;
  }
  return global_map;
}

}  // namespace

bool Stage3Runner::Run(const Stage3Config& config) {
  Stage2Dataset dataset;
  Stage2ArtifactReader reader;
  const bool load_clouds =
      config.output.save_preview_map || config.graph.enable_outage_blocks;
  if (!reader.Read(config, load_clouds, &dataset)) {
    return false;
  }

  Stage3RefineSummary summary;
  std::vector<Stage3OutageBlock> outage_blocks;
  if (!RunGraphRefinement(config, &dataset, &summary, &outage_blocks)) {
    return false;
  }

  lightning::CloudPtr preview_map;
  if (config.output.save_preview_map) {
    preview_map = BuildPreviewMap(config, dataset);
    AINFO << "[Stage3] Preview map points: "
          << (preview_map ? preview_map->size() : 0);
  }

  Stage3ArtifactWriter writer;
  return writer.Write(config, dataset, summary, outage_blocks, preview_map);
}

}  // namespace stage3
}  // namespace air_mapping
}  // namespace apollo
