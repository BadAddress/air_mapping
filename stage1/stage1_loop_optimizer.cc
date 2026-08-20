#include "modules/air_mapping/stage1/stage1_loop_optimizer.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

#include "cyber/common/log.h"
#include "modules/air_mapping/system/core/miao/core/opti_algo/algo_select.h"
#include "modules/air_mapping/system/core/miao/core/robust_kernel/cauchy.h"
#include "modules/air_mapping/system/core/miao/core/robust_kernel/huber.h"
#include "modules/air_mapping/system/core/miao/core/types/edge_se3.h"
#include "modules/air_mapping/system/core/miao/core/types/edge_se3_height_prior.h"
#include "modules/air_mapping/system/core/miao/core/types/vertex_se3.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/registration/icp.h"
#include "pcl/registration/ndt.h"

namespace apollo {
namespace air_mapping {
namespace stage1 {
namespace {

using lightning::CloudPtr;
using lightning::Keyframe;
using lightning::Mat6d;
using lightning::PointCloudType;
using lightning::PointType;
using lightning::SE3;
using lightning::SO3;
using lightning::Vec3d;
namespace miao = lightning::miao;

struct LoopCandidate {
  size_t target_index = 0;
  double distance_xy = 0.0;
};

bool PoseFinite(const SE3& pose) {
  return pose.translation().allFinite() && pose.rotationMatrix().allFinite();
}

Mat6d BuildSe3Information(double translation_sigma_m, double rotation_sigma_rad) {
  Mat6d information = Mat6d::Identity();
  const double translation_var =
      std::max(translation_sigma_m * translation_sigma_m, 1e-8);
  const double rotation_var = std::max(rotation_sigma_rad * rotation_sigma_rad, 1e-8);
  information.block<3, 3>(0, 0) *= 1.0 / translation_var;
  information.block<3, 3>(3, 3) *= 1.0 / rotation_var;
  return information;
}

double MaxAbsOptimizedHeight(const std::vector<Keyframe::Ptr>& keyframes) {
  double max_abs_height = 0.0;
  for (size_t i = 1; i < keyframes.size(); ++i) {
    if (!keyframes[i]) {
      continue;
    }
    const double z = keyframes[i]->GetOptPose().translation().z();
    if (std::isfinite(z)) {
      max_abs_height = std::max(max_abs_height, std::abs(z));
    }
  }
  return max_abs_height;
}

double MeanOptimizedZ(const std::vector<Keyframe::Ptr>& keyframes) {
  double sum = 0.0;
  size_t count = 0;
  for (const auto& keyframe : keyframes) {
    if (!keyframe) {
      continue;
    }
    sum += keyframe->GetOptPose().translation().z();
    ++count;
  }
  return count > 0 ? sum / static_cast<double>(count) : 0.0;
}

Eigen::Matrix4d LidarExtrinsicMatrix(const Keyframe::Ptr& keyframe) {
  Eigen::Matrix4d T_imu_lidar = Eigen::Matrix4d::Identity();
  if (!keyframe) {
    return T_imu_lidar;
  }
  const auto state = keyframe->GetState();
  T_imu_lidar.block<3, 3>(0, 0) = state.offset_R_lidar_.matrix();
  T_imu_lidar.block<3, 1>(0, 3) = state.offset_t_lidar_;
  return T_imu_lidar;
}

CloudPtr DownsampleCloud(const CloudPtr& input, float voxel_size) {
  CloudPtr output(new PointCloudType);
  if (!input || input->empty()) {
    return output;
  }

  // Full-density keyframe clouds intentionally preserve the driver payload,
  // which can include non-finite XYZ samples.  VoxelGrid computes integer
  // voxel indices before filtering them and can overflow on Inf/NaN, so keep
  // the saved cloud untouched but sanitize the temporary registration input.
  CloudPtr finite(new PointCloudType);
  finite->points.reserve(input->size());
  for (const auto& point : input->points) {
    // Do not trust cloud.is_dense here: this driver's payload contains
    // non-finite coordinates even when an intermediate PCL operation marks
    // the cloud dense.
    if (std::isfinite(point.x) && std::isfinite(point.y) &&
        std::isfinite(point.z)) {
      finite->points.push_back(point);
    }
  }
  finite->header = input->header;
  finite->sensor_origin_ = input->sensor_origin_;
  finite->sensor_orientation_ = input->sensor_orientation_;
  finite->is_dense = true;
  finite->height = 1;
  finite->width = finite->size();
  if (finite->empty()) {
    return output;
  }
  if (!(voxel_size > 1e-4f)) {
    *output = *finite;
    return output;
  }

  pcl::VoxelGrid<PointType> voxel;
  voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel.setInputCloud(finite);
  voxel.filter(*output);
  output->is_dense = false;
  output->height = 1;
  output->width = output->size();
  return output;
}

CloudPtr GetWorldCloudForKeyframe(
    const std::vector<Keyframe::Ptr>& keyframes, size_t keyframe_index,
    const Eigen::Matrix4d& T_imu_lidar,
    std::unordered_map<size_t, CloudPtr>* world_cloud_cache) {
  CloudPtr empty(new PointCloudType);
  if (keyframe_index >= keyframes.size() || !keyframes[keyframe_index]) {
    return empty;
  }
  if (world_cloud_cache) {
    const auto it = world_cloud_cache->find(keyframe_index);
    if (it != world_cloud_cache->end()) {
      return it->second;
    }
  }

  const auto cloud = keyframes[keyframe_index]->GetCloud();
  if (!cloud || cloud->empty()) {
    return empty;
  }
  const Eigen::Matrix4d T_world_lidar =
      keyframes[keyframe_index]->GetOptPose().matrix() * T_imu_lidar;

  CloudPtr transformed(new PointCloudType);
  pcl::transformPointCloud(*cloud, *transformed, T_world_lidar);
  transformed->is_dense = false;
  transformed->height = 1;
  transformed->width = transformed->size();
  if (world_cloud_cache) {
    (*world_cloud_cache)[keyframe_index] = transformed;
  }
  return transformed;
}

CloudPtr BuildHistorySubmap(const std::vector<Keyframe::Ptr>& keyframes,
                            size_t center_index, int half_range, int step,
                            const Eigen::Matrix4d& T_imu_lidar,
                            std::unordered_map<size_t, CloudPtr>* world_cloud_cache) {
  CloudPtr submap(new PointCloudType);
  if (center_index >= keyframes.size()) {
    return submap;
  }

  const int stride = std::max(step, 1);
  const int start =
      std::max(0, static_cast<int>(center_index) - std::max(half_range, 0));
  const int end = std::min(static_cast<int>(keyframes.size()) - 1,
                           static_cast<int>(center_index) +
                               std::max(half_range, 0));
  for (int index = start; index <= end; index += stride) {
    const CloudPtr transformed = GetWorldCloudForKeyframe(
        keyframes, static_cast<size_t>(index), T_imu_lidar, world_cloud_cache);
    if (!transformed || transformed->empty()) {
      continue;
    }
    *submap += *transformed;
  }

  submap->is_dense = false;
  submap->height = 1;
  submap->width = submap->size();
  return submap;
}

bool RunMultiResolutionNdt(const CloudPtr& target_world,
                           const CloudPtr& source_lidar,
                           const LoopClosureConfig& config,
                           Eigen::Matrix4f* T_world_lidar,
                           double* ndt_score) {
  if (!target_world || target_world->empty() || !source_lidar ||
      source_lidar->empty() || T_world_lidar == nullptr || ndt_score == nullptr) {
    return false;
  }
  if (!T_world_lidar->allFinite()) {
    *T_world_lidar = Eigen::Matrix4f::Identity();
  }

  *ndt_score = -std::numeric_limits<double>::infinity();
  Eigen::Matrix4f current = *T_world_lidar;
  bool ran_resolution = false;
  const double voxel_ratio = std::max(config.ndt_voxel_ratio, 0.01);
  for (double resolution : config.ndt_resolutions) {
    if (!(resolution > 0.0) || !std::isfinite(resolution)) {
      continue;
    }
    const float voxel_size = static_cast<float>(resolution * voxel_ratio);
    const CloudPtr target_ds = DownsampleCloud(target_world, voxel_size);
    const CloudPtr source_ds = DownsampleCloud(source_lidar, voxel_size);
    if (!target_ds || target_ds->empty() || !source_ds || source_ds->empty()) {
      continue;
    }

    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    ndt.setTransformationEpsilon(0.05);
    ndt.setStepSize(0.7);
    ndt.setMaximumIterations(std::max(config.ndt_max_iterations, 1));
    ndt.setResolution(static_cast<float>(resolution));
    ndt.setInputTarget(target_ds);
    ndt.setInputSource(source_ds);

    CloudPtr aligned(new PointCloudType);
    ndt.align(*aligned, current);
    current = ndt.getFinalTransformation();
    *ndt_score = ndt.getTransformationProbability();
    ran_resolution = true;
  }

  if (!ran_resolution || !std::isfinite(*ndt_score) || !current.allFinite()) {
    return false;
  }

  *T_world_lidar = current;
  return *ndt_score >= config.ndt_score_threshold;
}

bool TryIcpRefine(const CloudPtr& target_world, const CloudPtr& source_lidar,
                  const LoopClosureConfig& config,
                  Eigen::Matrix4f* T_world_lidar, double* icp_fitness) {
  if (!config.use_icp_refine || !target_world || target_world->empty() ||
      !source_lidar || source_lidar->empty() || T_world_lidar == nullptr ||
      icp_fitness == nullptr) {
    return false;
  }

  const Eigen::Matrix4f ndt_transform = *T_world_lidar;
  const float voxel_size = 0.2f;
  const CloudPtr target_ds = DownsampleCloud(target_world, voxel_size);
  const CloudPtr source_ds = DownsampleCloud(source_lidar, voxel_size);
  if (!target_ds || target_ds->empty() || !source_ds || source_ds->empty()) {
    return false;
  }

  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setMaximumIterations(std::max(config.icp_max_iterations, 1));
  icp.setMaxCorrespondenceDistance(
      static_cast<float>(std::max(config.icp_max_corr_dist, 0.1)));
  icp.setInputTarget(target_ds);
  icp.setInputSource(source_ds);

  CloudPtr aligned(new PointCloudType);
  icp.align(*aligned, ndt_transform);
  if (!icp.hasConverged()) {
    return false;
  }

  const Eigen::Matrix4f icp_transform = icp.getFinalTransformation();
  if (!icp_transform.allFinite()) {
    return false;
  }
  *icp_fitness = icp.getFitnessScore();
  if (!std::isfinite(*icp_fitness) ||
      *icp_fitness > config.icp_fitness_threshold) {
    return false;
  }

  const Eigen::Matrix4d delta =
      ndt_transform.cast<double>().inverse() * icp_transform.cast<double>();
  const double translation_delta = delta.block<3, 1>(0, 3).norm();
  const double rotation_delta =
      SO3::fitToSO3(delta.block<3, 3>(0, 0)).log().norm();
  const double rotation_gate =
      config.icp_max_rotation_delta_deg * M_PI / 180.0;
  if (!std::isfinite(translation_delta) || !std::isfinite(rotation_delta) ||
      translation_delta > config.icp_max_translation_delta ||
      rotation_delta > rotation_gate) {
    return false;
  }

  *T_world_lidar = icp_transform;
  return true;
}

std::vector<LoopCandidate> FindLoopCandidates(
    const std::vector<Keyframe::Ptr>& keyframes, size_t query_index,
    const LoopClosureConfig& config) {
  std::vector<LoopCandidate> candidates;
  if (query_index >= keyframes.size() || !keyframes[query_index]) {
    return candidates;
  }

  const SE3 query_pose = keyframes[query_index]->GetOptPose();
  if (!PoseFinite(query_pose)) {
    return candidates;
  }

  const int closest_id_threshold =
      std::max(config.closest_id_threshold, std::max(config.min_keyframe_gap, 1));
  if (static_cast<int>(query_index) <= closest_id_threshold) {
    return candidates;
  }

  std::vector<LoopCandidate> raw_candidates;
  raw_candidates.reserve(16);
  const double radius = std::max(config.search_radius, 0.1);
  for (size_t target_index = 0; target_index < query_index; ++target_index) {
    if (static_cast<int>(query_index - target_index) < closest_id_threshold) {
      continue;
    }
    if (!keyframes[target_index]) {
      continue;
    }
    const SE3 target_pose = keyframes[target_index]->GetOptPose();
    if (!PoseFinite(target_pose)) {
      continue;
    }
    const double distance_xy =
        (target_pose.translation().head<2>() -
         query_pose.translation().head<2>())
            .norm();
    if (!std::isfinite(distance_xy) || distance_xy > radius) {
      continue;
    }
    raw_candidates.push_back(LoopCandidate{target_index, distance_xy});
  }

  std::sort(raw_candidates.begin(), raw_candidates.end(),
            [](const LoopCandidate& lhs, const LoopCandidate& rhs) {
              return lhs.distance_xy < rhs.distance_xy;
            });

  const int max_candidates = std::max(config.max_candidates_per_query, 1);
  const int min_id_interval = std::max(config.min_id_interval, 0);
  for (const auto& candidate : raw_candidates) {
    bool suppressed = false;
    for (const auto& selected : candidates) {
      if (std::abs(static_cast<int>(candidate.target_index) -
                   static_cast<int>(selected.target_index)) <= min_id_interval) {
        suppressed = true;
        break;
      }
    }
    if (suppressed) {
      continue;
    }
    candidates.push_back(candidate);
    if (static_cast<int>(candidates.size()) >= max_candidates) {
      break;
    }
  }
  return candidates;
}

void DetectLoopConstraints(const Stage1Config& config,
                           const std::vector<Keyframe::Ptr>& keyframes,
                           std::vector<Stage1LoopConstraint>* loop_constraints,
                           Stage1LoopSummary* summary) {
  if (loop_constraints == nullptr || summary == nullptr || keyframes.empty()) {
    return;
  }

  const auto& loop_config = config.loop_closure;
  const Eigen::Matrix4d T_imu_lidar = LidarExtrinsicMatrix(keyframes.front());
  const Eigen::Matrix4d T_lidar_imu = T_imu_lidar.inverse();
  std::unordered_map<size_t, CloudPtr> world_cloud_cache;
  size_t last_query_index = std::numeric_limits<size_t>::max();
  const int query_stride = std::max(loop_config.loop_kf_gap, 1);

  for (size_t query_index = 0; query_index < keyframes.size(); ++query_index) {
    if (last_query_index != std::numeric_limits<size_t>::max() &&
        static_cast<int>(query_index - last_query_index) < query_stride) {
      continue;
    }
    const auto& query_keyframe = keyframes[query_index];
    if (!query_keyframe || !query_keyframe->GetCloud() ||
        query_keyframe->GetCloud()->empty()) {
      continue;
    }

    ++summary->query_count;
    last_query_index = query_index;

    const auto candidates =
        FindLoopCandidates(keyframes, query_index, loop_config);
    summary->coarse_candidate_count += candidates.size();
    if (candidates.empty()) {
      continue;
    }

    const CloudPtr source_lidar = query_keyframe->GetCloud();
    for (const auto& candidate : candidates) {
      const CloudPtr target_submap = BuildHistorySubmap(
          keyframes, candidate.target_index,
          loop_config.history_submap_half_range,
          loop_config.history_submap_step, T_imu_lidar, &world_cloud_cache);
      if (!target_submap || target_submap->empty()) {
        continue;
      }

      Eigen::Matrix4f T_world_lidar =
          (query_keyframe->GetOptPose().matrix() * T_imu_lidar).cast<float>();
      double ndt_score = 0.0;
      if (!RunMultiResolutionNdt(target_submap, source_lidar, loop_config,
                                 &T_world_lidar, &ndt_score)) {
        continue;
      }

      double icp_fitness = -1.0;
      const bool used_icp = TryIcpRefine(target_submap, source_lidar, loop_config,
                                         &T_world_lidar, &icp_fitness);

      const Eigen::Matrix4d T_world_lidar_d = T_world_lidar.cast<double>();
      const Eigen::Matrix4d T_world_imu_d = T_world_lidar_d * T_lidar_imu;
      const SE3 source_world_pose(
          SO3::fitToSO3(T_world_imu_d.block<3, 3>(0, 0)),
          T_world_imu_d.block<3, 1>(0, 3));
      const SE3 target_world_pose = keyframes[candidate.target_index]->GetOptPose();
      const SE3 initial_target_to_source =
          target_world_pose.inverse() * query_keyframe->GetOptPose();
      const SE3 measurement_target_to_source =
          target_world_pose.inverse() * source_world_pose;
      const SE3 delta = measurement_target_to_source.inverse() *
                        initial_target_to_source;

      Stage1LoopConstraint constraint;
      constraint.target_index = candidate.target_index;
      constraint.source_index = query_index;
      constraint.target_id = keyframes[candidate.target_index]->GetID();
      constraint.source_id = query_keyframe->GetID();
      constraint.measurement_target_to_source = measurement_target_to_source;
      constraint.ndt_score = ndt_score;
      constraint.icp_fitness = icp_fitness;
      constraint.used_icp = used_icp;
      constraint.delta_from_initial_translation_m = delta.translation().norm();
      constraint.delta_from_initial_rotation_deg =
          delta.so3().log().norm() * 180.0 / M_PI;
      loop_constraints->push_back(constraint);
      ++summary->accepted_loop_count;

      AINFO << "[Stage1Loop] Accepted loop target=" << constraint.target_id
            << ", source=" << constraint.source_id
            << ", ndt_score=" << std::fixed << std::setprecision(4)
            << ndt_score
            << ", delta_t=" << constraint.delta_from_initial_translation_m
            << "m, delta_r=" << constraint.delta_from_initial_rotation_deg
            << "deg" << (used_icp ? ", icp=used" : "");
    }
  }
}

bool RunPoseGraphOptimization(const Stage1Config& config,
                              const std::vector<Keyframe::Ptr>& keyframes,
                              const std::vector<Stage1LoopConstraint>& loops,
                              Stage1LoopSummary* summary,
                              bool add_height_prior_edges,
                              bool update_primary_summary) {
  if (summary == nullptr || keyframes.empty()) {
    return false;
  }
  if (loops.empty() && !add_height_prior_edges) {
    AINFO << "[Stage1Loop] No accepted loop constraints, keep LIO poses";
    return true;
  }

  miao::OptimizerConfig optimizer_config(
      miao::AlgorithmType::LEVENBERG_MARQUARDT,
      miao::LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN, false);
  auto optimizer =
      miao::SetupOptimizer<Eigen::Dynamic, Eigen::Dynamic>(optimizer_config);
  optimizer->SetVerbose(config.loop_closure.verbose);

  std::vector<std::shared_ptr<miao::VertexSE3>> vertices(keyframes.size());
  for (size_t i = 0; i < keyframes.size(); ++i) {
    if (!keyframes[i]) {
      continue;
    }
    auto vertex = std::make_shared<miao::VertexSE3>();
    vertex->SetId(static_cast<int>(i));
    vertex->SetEstimate(keyframes[i]->GetOptPose());
    if (i == 0) {
      vertex->SetFixed(true);
    }
    optimizer->AddVertex(vertex);
    vertices[i] = vertex;
  }

  const auto& loop_config = config.loop_closure;
  size_t lio_edge_count = 0;
  size_t loop_edge_count = 0;
  size_t height_prior_edge_count = 0;
  size_t loop_outlier_edge_count = 0;
  std::vector<std::shared_ptr<miao::EdgeSE3>> loop_edges;
  const Mat6d lio_information =
      BuildSe3Information(loop_config.lio_translation_sigma_m,
                          loop_config.lio_rotation_sigma_deg * M_PI / 180.0);
  for (size_t i = 1; i < keyframes.size(); ++i) {
    if (!keyframes[i - 1] || !keyframes[i] || !vertices[i - 1] || !vertices[i]) {
      continue;
    }
    SE3 relative_motion = keyframes[i]->GetRelativeMotion();
    if (!PoseFinite(relative_motion)) {
      relative_motion = keyframes[i - 1]->GetLIOPose().inverse() *
                        keyframes[i]->GetLIOPose();
    }

    auto edge = std::make_shared<miao::EdgeSE3>();
    edge->SetId(static_cast<int>(i));
    edge->SetVertex(0, vertices[i - 1]);
    edge->SetVertex(1, vertices[i]);
    edge->SetMeasurement(relative_motion);
    edge->SetInformation(lio_information);
    auto huber = std::make_shared<miao::RobustKernelHuber>();
    huber->SetDelta(loop_config.lio_huber_delta);
    edge->SetRobustKernel(huber);
    optimizer->AddEdge(edge);
    ++lio_edge_count;
  }

  const Mat6d loop_information =
      BuildSe3Information(loop_config.loop_translation_sigma_m,
                          loop_config.loop_rotation_sigma_deg * M_PI / 180.0) *
      std::max(loop_config.loop_info_scale, 1e-6);
  int loop_edge_offset = 100000;
  for (const auto& loop : loops) {
    if (loop.target_index >= vertices.size() ||
        loop.source_index >= vertices.size() || !vertices[loop.target_index] ||
        !vertices[loop.source_index]) {
      continue;
    }
    auto edge = std::make_shared<miao::EdgeSE3>();
    edge->SetId(loop_edge_offset++);
    edge->SetVertex(0, vertices[loop.target_index]);
    edge->SetVertex(1, vertices[loop.source_index]);
    edge->SetMeasurement(loop.measurement_target_to_source);
    edge->SetInformation(loop_information);
    auto cauchy = std::make_shared<miao::RobustKernelCauchy>();
    cauchy->SetDelta(loop_config.loop_cauchy_delta);
    edge->SetRobustKernel(cauchy);
    optimizer->AddEdge(edge);
    loop_edges.push_back(edge);
    ++loop_edge_count;
  }

  if (add_height_prior_edges) {
    const double height_noise = std::max(config.zleveling.height_noise_m, 1e-4);
    const double height_info = 1.0 / (height_noise * height_noise);
    int height_prior_edge_offset = 200000;
    for (size_t i = 0; i < keyframes.size(); ++i) {
      if (!keyframes[i] || !vertices[i]) {
        continue;
      }
      if (!std::isfinite(keyframes[i]->GetOptPose().translation().z())) {
        continue;
      }

      auto edge = std::make_shared<miao::EdgeHeightPrior>();
      edge->SetId(height_prior_edge_offset++);
      edge->SetVertex(0, vertices[i]);
      edge->SetMeasurement(0.0);
      Eigen::Matrix<double, 1, 1> information;
      information(0, 0) = height_info;
      edge->SetInformation(information);
      auto huber = std::make_shared<miao::RobustKernelHuber>();
      huber->SetDelta(std::max(height_noise * 3.0, 1e-4));
      edge->SetRobustKernel(huber);
      optimizer->AddEdge(edge);
      ++height_prior_edge_count;
    }
  }

  if (loop_edge_count == 0 && height_prior_edge_count == 0) {
    AINFO << "[Stage1Loop] No loop or height-prior edges were added to graph";
    return true;
  }

  optimizer->InitializeOptimization();
  optimizer->ComputeActiveErrors();
  const double chi2_before = optimizer->ActiveChi2();
  if (!std::isfinite(chi2_before)) {
    AWARN << "[Stage1Loop] chi2_before is invalid, skip applying loop PGO";
    return true;
  }

  int optimizer_iterations =
      optimizer->Optimize(std::max(loop_config.max_iterations, 1));
  optimizer->ComputeActiveErrors();
  double chi2_after = optimizer->ActiveChi2();
  if (optimizer_iterations <= 0 || !std::isfinite(chi2_after)) {
    AWARN << "[Stage1Loop] optimizer failed, keep LIO poses";
    return true;
  }

  if (loop_config.enable_loop_outlier_rejection && !loop_edges.empty()) {
    const double outlier_chi2 =
        std::max(loop_config.loop_outlier_chi2_threshold, 1e-6);
    for (const auto& edge : loop_edges) {
      if (!edge) {
        continue;
      }
      edge->ComputeError();
      const double chi2 = edge->Chi2();
      if (!std::isfinite(chi2) || chi2 > outlier_chi2) {
        edge->SetLevel(1);
        ++loop_outlier_edge_count;
      } else {
        edge->SetRobustKernel(nullptr);
      }
    }

    if (loop_outlier_edge_count > 0 &&
        loop_outlier_edge_count < loop_edges.size()) {
      optimizer->InitializeOptimization();
      optimizer->ComputeActiveErrors();
      const int retry_iterations =
          optimizer->Optimize(std::max(loop_config.max_iterations, 1));
      optimizer->ComputeActiveErrors();
      const double retry_chi2_after = optimizer->ActiveChi2();
      if (retry_iterations > 0 && std::isfinite(retry_chi2_after)) {
        optimizer_iterations += retry_iterations;
        chi2_after = retry_chi2_after;
      }
    } else if (loop_outlier_edge_count == loop_edges.size()) {
      AWARN << "[Stage1Loop] all loop edges were classified as outliers; "
            << "keep robust first-pass result";
      loop_outlier_edge_count = 0;
      for (const auto& edge : loop_edges) {
        if (edge) {
          edge->SetLevel(0);
        }
      }
    }
  }

  for (size_t i = 0; i < keyframes.size(); ++i) {
    if (keyframes[i] && vertices[i]) {
      keyframes[i]->SetOptPose(vertices[i]->Estimate());
    }
  }

  if (update_primary_summary) {
    summary->lio_edge_count = lio_edge_count;
    summary->loop_edge_count = loop_edge_count;
    summary->loop_outlier_edge_count = loop_outlier_edge_count;
    summary->optimizer_iterations = optimizer_iterations;
    summary->chi2_before = chi2_before;
    summary->chi2_after = chi2_after;
  } else {
    summary->zleveling_height_prior_edge_count = height_prior_edge_count;
    summary->zleveling_optimizer_iterations = optimizer_iterations;
    summary->zleveling_chi2_before = chi2_before;
    summary->zleveling_chi2_after = chi2_after;
  }

  AINFO << "[Stage1Loop] PGO finished: loops=" << loop_edge_count
        << ", loop_outliers=" << loop_outlier_edge_count
        << ", lio_edges=" << lio_edge_count
        << ", height_prior_edges=" << height_prior_edge_count
        << ", iterations=" << optimizer_iterations
        << ", chi2=" << chi2_before << " -> " << chi2_after
        << (add_height_prior_edges ? ", round=zleveling" : ", round=loop");
  return true;
}

}  // namespace

bool Stage1LoopOptimizer::Optimize(
    const Stage1Config& config,
    const std::vector<lightning::Keyframe::Ptr>& keyframes,
    std::vector<Stage1LoopConstraint>* loop_constraints,
    Stage1LoopSummary* summary) const {
  if (loop_constraints == nullptr || summary == nullptr) {
    return false;
  }

  loop_constraints->clear();
  *summary = Stage1LoopSummary();
  summary->enabled = config.loop_closure.enable;
  summary->zleveling_enabled = config.zleveling.enable;
  summary->keyframe_count = keyframes.size();

  for (const auto& keyframe : keyframes) {
    if (keyframe) {
      keyframe->SetOptPose(keyframe->GetLIOPose());
    }
  }

  if (!config.loop_closure.enable) {
    AINFO << "[Stage1Loop] Disabled, Stage1 output keeps raw LIO poses";
    return true;
  }
  if (keyframes.size() < 2) {
    AINFO << "[Stage1Loop] Too few keyframes, skip loop optimization";
    return true;
  }

  DetectLoopConstraints(config, keyframes, loop_constraints, summary);
  AINFO << "[Stage1Loop] Detection summary: queries=" << summary->query_count
        << ", coarse_candidates=" << summary->coarse_candidate_count
        << ", accepted_loops=" << summary->accepted_loop_count;
  if (!RunPoseGraphOptimization(config, keyframes, *loop_constraints, summary,
                                false, true)) {
    return false;
  }
  if (!config.zleveling.enable) {
    return true;
  }

  summary->zleveling_max_abs_height_before_m =
      MaxAbsOptimizedHeight(keyframes);
  summary->zleveling_mean_z_before = MeanOptimizedZ(keyframes);
  if (!RunPoseGraphOptimization(config, keyframes, *loop_constraints, summary,
                                true, false)) {
    return false;
  }
  summary->zleveling_max_abs_height_after_m =
      MaxAbsOptimizedHeight(keyframes);
  summary->zleveling_mean_z_after = MeanOptimizedZ(keyframes);
  return true;
}

}  // namespace stage1
}  // namespace air_mapping
}  // namespace apollo
