#include "modules/air_mapping/stage2/stage2_runner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/SVD>

#include "pcl/common/transforms.h"
#include "pcl/filters/voxel_grid.h"

#include "cyber/common/log.h"
#include "modules/air_mapping/stage2/stage1_artifact_reader.h"
#include "modules/air_mapping/stage2/stage2_artifact_writer.h"
#include "modules/air_mapping/stage2/stage2_types.h"

namespace apollo {
namespace air_mapping {
namespace stage2 {

namespace {

constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kDegToRad = M_PI / 180.0;
constexpr uint32_t kSolTypeNarrowInt = 50;
constexpr uint32_t kSolTypeInsRtkFixed = 56;

double DistanceXY(const lightning::Vec3d& lhs, const lightning::Vec3d& rhs) {
  return (lhs.head<2>() - rhs.head<2>()).norm();
}

bool IsUsableGpsObservation(const Stage1GpsKeyframeObservation& observation) {
  return observation.has_gps && observation.utm_position.allFinite() &&
         observation.std_dev.allFinite();
}

lightning::Vec3d FindConfiguredLeverArm(const Stage1Dataset& dataset) {
  for (const auto& observation : dataset.gps_raw_keyframe_observations) {
    if (observation.configured_lever_arm.allFinite()) {
      return observation.configured_lever_arm;
    }
  }
  return lightning::Vec3d::Zero();
}

double NormalizeAngleRad(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double GnssLioYawDiffDeg(const Stage1GpsRawKeyframeObservation& observation) {
  return std::abs(NormalizeAngleRad(observation.gnss_heading_rad -
                                    observation.lio_opt_yaw_rad)) *
         kRadToDeg;
}

bool IsAcceptedFixedSolution(const Stage1GpsRawKeyframeObservation& observation,
                             const LeverArmCalibrationConfig& config) {
  if (!config.require_rtk_fixed) {
    return true;
  }
  if (observation.sol_status != config.required_sol_status) {
    return false;
  }
  return observation.sol_type == config.required_sol_type ||
         observation.sol_type == kSolTypeNarrowInt ||
         observation.sol_type == kSolTypeInsRtkFixed;
}

bool IsHighPrecisionRawGps(const Stage1GpsRawKeyframeObservation& observation,
                           const LeverArmCalibrationConfig& config,
                           std::string* reject_reason) {
  if (!observation.has_raw_gps ||
      !observation.antenna_utm_position.allFinite()) {
    if (reject_reason) {
      *reject_reason = "no_raw_gps";
    }
    return false;
  }
  if (config.require_stage1_gps_anchor && !observation.has_stage1_gps) {
    if (reject_reason) {
      *reject_reason = "not_stage1_anchor";
    }
    return false;
  }
  if (!IsAcceptedFixedSolution(observation, config)) {
    if (reject_reason) {
      *reject_reason = "not_rtk_fixed";
    }
    return false;
  }
  if (config.max_heading_std_deg > 0.0 &&
      observation.heading_std_deg > 0.0 &&
      observation.heading_std_deg > config.max_heading_std_deg) {
    if (reject_reason) {
      *reject_reason = "heading_std_too_large";
    }
    return false;
  }
  if (!observation.std_dev.allFinite()) {
    if (reject_reason) {
      *reject_reason = "invalid_std";
    }
    return false;
  }
  if (std::abs(observation.std_dev.x()) > config.max_gps_std_xy_m ||
      std::abs(observation.std_dev.y()) > config.max_gps_std_xy_m) {
    if (reject_reason) {
      *reject_reason = "std_xy_too_large";
    }
    return false;
  }
  const double max_interp_gap =
      std::max(std::abs(observation.interp_gap_before_s),
               std::abs(observation.interp_gap_after_s));
  if (max_interp_gap > config.max_interp_gap_s) {
    if (reject_reason) {
      *reject_reason = "interp_gap_too_large";
    }
    return false;
  }
  if (reject_reason) {
    reject_reason->clear();
  }
  return true;
}

double ComputeAnchorWeight(const lightning::Vec3d& std_dev,
                           const AlignmentConfig& config) {
  const double std_x = std::max(std::abs(std_dev.x()), config.min_gps_std_xy);
  const double std_y = std::max(std::abs(std_dev.y()), config.min_gps_std_xy);
  const double std_z = std::max(std::abs(std_dev.z()), config.min_gps_std_z);
  const double variance =
      config.constrain_to_yaw_only
          ? (std_x * std_x + std_y * std_y) / 2.0
          : (std_x * std_x + std_y * std_y + std_z * std_z) / 3.0;
  return 1.0 / std::max(variance, 1e-6);
}

lightning::Mat3d GnssHeadingOrientation(
    const Stage1GpsRawKeyframeObservation& observation,
    double heading_bias_rad = 0.0) {
  return lightning::SO3::rotZ(observation.gnss_heading_rad + heading_bias_rad)
      .matrix();
}

lightning::Mat3d LeverArmOrientation(
    const lightning::SE3& lio_to_utm_local,
    const Stage1GpsRawKeyframeObservation& observation,
    const LeverArmCalibrationConfig& config, double heading_bias_rad = 0.0) {
  (void)lio_to_utm_local;
  (void)config;
  return GnssHeadingOrientation(observation, heading_bias_rad);
}

lightning::Vec3d PredictAntennaUtm(
    const lightning::SE3& lio_to_utm_local,
    const Stage1GpsRawKeyframeObservation& observation,
    const lightning::Vec3d& lever_arm, const lightning::Vec3d& utm_origin,
    const LeverArmCalibrationConfig& config, double heading_bias_rad = 0.0) {
  const lightning::Vec3d imu_utm =
      lio_to_utm_local * observation.lio_opt_pose.translation() + utm_origin;
  return imu_utm +
         LeverArmOrientation(lio_to_utm_local, observation, config,
                             heading_bias_rad) *
             lever_arm;
}

double RobustWeight(double residual_xy_m, double huber_delta_m) {
  if (huber_delta_m <= 0.0 || residual_xy_m <= huber_delta_m) {
    return 1.0;
  }
  return huber_delta_m / std::max(residual_xy_m, 1e-9);
}

bool SolveLeverArmAndHeadingBiasStep(
    const std::vector<const Stage1GpsRawKeyframeObservation*>& samples,
    const Stage2AlignmentResult& alignment,
    const lightning::Vec3d& initial_lever_arm,
    const lightning::Vec3d& current_lever_arm, double current_heading_bias_rad,
    const LeverArmCalibrationConfig& config, lightning::Vec3d* lever_step,
    double* heading_bias_step) {
  if (lever_step == nullptr || heading_bias_step == nullptr ||
      samples.empty()) {
    return false;
  }

  constexpr int kStateSize = 4;
  Eigen::Matrix<double, kStateSize, kStateSize> normal =
      Eigen::Matrix<double, kStateSize, kStateSize>::Zero();
  Eigen::Matrix<double, kStateSize, 1> rhs =
      Eigen::Matrix<double, kStateSize, 1>::Zero();
  for (const auto* sample : samples) {
    if (sample == nullptr) {
      continue;
    }
    const lightning::Mat3d orientation =
        LeverArmOrientation(alignment.lio_to_utm_local, *sample, config,
                            current_heading_bias_rad);
    const lightning::Vec3d predicted =
        PredictAntennaUtm(alignment.lio_to_utm_local, *sample,
                          current_lever_arm, alignment.utm_origin, config,
                          current_heading_bias_rad);
    const lightning::Vec3d residual = sample->antenna_utm_position - predicted;
    const double robust_weight =
        RobustWeight(residual.head<2>().norm(), config.huber_delta_xy_m);
    const double std_x = std::max(std::abs(sample->std_dev.x()), 1e-3);
    const double std_y = std::max(std::abs(sample->std_dev.y()), 1e-3);
    const double std_z = std::max(std::abs(sample->std_dev.z()), 1e-3);
    const Eigen::Vector3d row_weight(
        1.0 / (std_x * std_x), 1.0 / (std_y * std_y),
        config.estimate_z ? 1.0 / (std_z * std_z) : 0.0);
    for (int row = 0; row < 3; ++row) {
      if (row_weight(row) <= 0.0) {
        continue;
      }
      Eigen::Matrix<double, 1, kStateSize> jacobian =
          Eigen::Matrix<double, 1, kStateSize>::Zero();
      jacobian.block<1, 3>(0, 0) = orientation.row(row);
      if (config.estimate_heading_bias) {
        const Eigen::Vector3d rotated = orientation * current_lever_arm;
        jacobian(0, 3) = row == 0 ? -rotated.y() : (row == 1 ? rotated.x() : 0.0);
      }
      const double weight = robust_weight * row_weight(row);
      normal += weight * jacobian.transpose() * jacobian;
      rhs += weight * jacobian.transpose() * residual(row);
    }
  }

  const lightning::Vec3d lever_delta = current_lever_arm - initial_lever_arm;
  const double info_xy =
      1.0 / (config.prior_sigma_xy_m * config.prior_sigma_xy_m);
  const double info_z =
      1.0 / (config.prior_sigma_z_m * config.prior_sigma_z_m);
  normal(0, 0) += info_xy;
  normal(1, 1) += info_xy;
  normal(2, 2) += info_z;
  rhs(0) -= info_xy * lever_delta.x();
  rhs(1) -= info_xy * lever_delta.y();
  rhs(2) -= info_z * lever_delta.z();
  if (config.estimate_heading_bias) {
    const double sigma_rad =
        config.prior_sigma_heading_bias_deg * kDegToRad;
    const double heading_info = 1.0 / (sigma_rad * sigma_rad);
    normal(3, 3) += heading_info;
    rhs(3) -= heading_info * current_heading_bias_rad;
  } else {
    normal(3, 3) += 1.0;
  }

  Eigen::LDLT<Eigen::Matrix<double, kStateSize, kStateSize>> ldlt(normal);
  if (ldlt.info() != Eigen::Success) {
    return false;
  }
  const Eigen::Matrix<double, kStateSize, 1> step = ldlt.solve(rhs);
  if (ldlt.info() != Eigen::Success || !step.allFinite()) {
    return false;
  }
  *lever_step = step.head<3>();
  *heading_bias_step = config.estimate_heading_bias ? step(3) : 0.0;
  if (!config.estimate_z) {
    lever_step->z() = 0.0;
  }
  const lightning::Vec3d proposed_delta =
      current_lever_arm + *lever_step - initial_lever_arm;
  const double norm = proposed_delta.norm();
  if (config.max_correction_norm_m > 0.0 && norm > config.max_correction_norm_m) {
    *lever_step =
        initial_lever_arm +
        proposed_delta * (config.max_correction_norm_m / norm) -
        current_lever_arm;
  }
  if (config.max_heading_bias_deg > 0.0) {
    const double max_bias_rad = config.max_heading_bias_deg * kDegToRad;
    const double proposed_bias = current_heading_bias_rad + *heading_bias_step;
    const double clamped_bias =
        std::clamp(proposed_bias, -max_bias_rad, max_bias_rad);
    *heading_bias_step = clamped_bias - current_heading_bias_rad;
  }
  return true;
}

bool FitSmoothedHeights(const std::vector<double>& x,
                        const std::vector<double>& z,
                        const std::vector<double>& std_z,
                        const AlignmentConfig& config,
                        std::vector<double>* smoothed_z) {
  if (smoothed_z == nullptr || x.size() != z.size() ||
      z.size() != std_z.size() || z.empty()) {
    return false;
  }

  const size_t n = z.size();
  smoothed_z->assign(n, 0.0);
  if (!config.smooth_gps_height || n < 3 ||
      config.gps_height_smoothing_lambda <= 0.0) {
    *smoothed_z = z;
    return true;
  }

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(n),
                                            static_cast<Eigen::Index>(n));
  Eigen::VectorXd b = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(n));

  for (size_t i = 0; i < n; ++i) {
    const double sigma = std::max(std::abs(std_z[i]), config.min_gps_std_z);
    const double weight = 1.0 / (sigma * sigma);
    A(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(i)) += weight;
    b(static_cast<Eigen::Index>(i)) += weight * z[i];
  }

  for (size_t i = 1; i + 1 < n; ++i) {
    const double h0 = std::max(x[i] - x[i - 1], 1e-3);
    const double h1 = std::max(x[i + 1] - x[i], 1e-3);
    Eigen::Vector3d c;
    c << 2.0 / (h0 * (h0 + h1)), -2.0 / (h0 * h1), 2.0 / (h1 * (h0 + h1));
    const Eigen::Matrix3d penalty =
        config.gps_height_smoothing_lambda * (c * c.transpose());
    const Eigen::Index row = static_cast<Eigen::Index>(i - 1);
    A.block<3, 3>(row, row) += penalty;
  }

  A += Eigen::MatrixXd::Identity(static_cast<Eigen::Index>(n),
                                 static_cast<Eigen::Index>(n)) *
       1e-9;
  Eigen::LDLT<Eigen::MatrixXd> ldlt(A);
  if (ldlt.info() != Eigen::Success) {
    return false;
  }

  const Eigen::VectorXd solution = ldlt.solve(b);
  if (ldlt.info() != Eigen::Success) {
    return false;
  }

  for (size_t i = 0; i < n; ++i) {
    (*smoothed_z)[i] = solution(static_cast<Eigen::Index>(i));
    if (!std::isfinite((*smoothed_z)[i])) {
      return false;
    }
  }
  return true;
}

bool BuildAlignmentAnchors(const Stage1Dataset& dataset,
                           const AlignmentConfig& config,
                           std::vector<AlignmentAnchor>* anchors,
                           std::vector<GpsSegmentSummary>* segments) {
  if (anchors == nullptr || segments == nullptr) {
    return false;
  }
  anchors->clear();
  segments->clear();

  std::unordered_map<unsigned long, size_t> id_to_index;
  for (size_t i = 0; i < dataset.keyframes.size(); ++i) {
    if (dataset.keyframes[i]) {
      id_to_index[dataset.keyframes[i]->GetID()] = i;
    }
  }

  for (const auto& observation : dataset.gps_keyframe_observations) {
    if (!IsUsableGpsObservation(observation)) {
      continue;
    }
    auto it = id_to_index.find(observation.keyframe_id);
    if (it == id_to_index.end()) {
      continue;
    }
    const auto& keyframe = dataset.keyframes[it->second];
    if (!keyframe) {
      continue;
    }

    AlignmentAnchor anchor;
    anchor.keyframe_index = it->second;
    anchor.keyframe_id = observation.keyframe_id;
    anchor.timestamp = observation.timestamp;
    anchor.lio_position = keyframe->GetLIOPose().translation();
    anchor.gps_utm_position = observation.utm_position;
    anchor.smoothed_gps_utm_position = observation.utm_position;
    anchor.gps_std_dev = observation.std_dev;
    anchor.weight = ComputeAnchorWeight(observation.std_dev, config);
    anchors->push_back(anchor);
  }

  std::sort(anchors->begin(), anchors->end(),
            [](const AlignmentAnchor& lhs, const AlignmentAnchor& rhs) {
              if (lhs.keyframe_index != rhs.keyframe_index) {
                return lhs.keyframe_index < rhs.keyframe_index;
              }
              return lhs.timestamp < rhs.timestamp;
            });

  if (anchors->empty()) {
    AERROR << "No valid GPS anchors found in stage1 artifacts";
    return false;
  }

  int current_segment = -1;
  size_t segment_start = 0;
  for (size_t i = 0; i < anchors->size(); ++i) {
    if (i == 0 || DistanceXY((*anchors)[i - 1].gps_utm_position,
                             (*anchors)[i].gps_utm_position) >
                      config.gps_segment_break_distance_m) {
      if (i > 0) {
        GpsSegmentSummary summary;
        summary.segment_id = current_segment;
        summary.first_anchor_index = segment_start;
        summary.last_anchor_index = i - 1;
        summary.first_keyframe_id = (*anchors)[segment_start].keyframe_id;
        summary.last_keyframe_id = (*anchors)[i - 1].keyframe_id;
        summary.anchor_count = i - segment_start;
        segments->push_back(summary);
      }
      ++current_segment;
      segment_start = i;
    }
    (*anchors)[i].segment_id = current_segment;
  }

  GpsSegmentSummary summary;
  summary.segment_id = current_segment;
  summary.first_anchor_index = segment_start;
  summary.last_anchor_index = anchors->size() - 1;
  summary.first_keyframe_id = (*anchors)[segment_start].keyframe_id;
  summary.last_keyframe_id = anchors->back().keyframe_id;
  summary.anchor_count = anchors->size() - segment_start;
  segments->push_back(summary);

  for (auto& segment : *segments) {
    const size_t start = segment.first_anchor_index;
    const size_t end = segment.last_anchor_index;
    const size_t count = end - start + 1;
    std::vector<double> x(count, 0.0);
    std::vector<double> z(count, 0.0);
    std::vector<double> std_z(count, 0.0);
    for (size_t i = 0; i < count; ++i) {
      const auto& anchor = (*anchors)[start + i];
      if (i > 0) {
        x[i] = x[i - 1] + DistanceXY((*anchors)[start + i - 1].gps_utm_position,
                                     anchor.gps_utm_position);
      }
      z[i] = anchor.gps_utm_position.z();
      std_z[i] = anchor.gps_std_dev.z();
    }

    std::vector<double> smoothed_z;
    if (!FitSmoothedHeights(x, z, std_z, config, &smoothed_z)) {
      AERROR << "Failed to smooth GPS heights for segment "
             << segment.segment_id;
      return false;
    }

    double sum_abs_delta = 0.0;
    double max_abs_delta = 0.0;
    for (size_t i = 0; i < count; ++i) {
      auto& anchor = (*anchors)[start + i];
      anchor.smoothed_gps_utm_position.z() = smoothed_z[i];
      const double abs_delta = std::abs(smoothed_z[i] - z[i]);
      sum_abs_delta += abs_delta;
      max_abs_delta = std::max(max_abs_delta, abs_delta);
    }
    segment.length_xy_m = x.back();
    segment.mean_abs_z_smoothing_delta_m =
        sum_abs_delta / static_cast<double>(std::max<size_t>(count, 1));
    segment.max_abs_z_smoothing_delta_m = max_abs_delta;
  }

  AINFO << "[Stage2] GPS anchors=" << anchors->size()
        << ", segments=" << segments->size()
        << ", break_distance_m=" << config.gps_segment_break_distance_m;
  return true;
}

bool BuildLeverArmCorrectedAlignmentAnchors(
    const Stage1Dataset& dataset, const Stage2AlignmentResult& alignment,
    const lightning::Vec3d& lever_arm,
    double heading_bias_rad,
    const LeverArmCalibrationConfig& lever_config,
    const AlignmentConfig& alignment_config,
    std::vector<AlignmentAnchor>* anchors,
    std::vector<GpsSegmentSummary>* segments,
    bool* used_raw_high_precision) {
  if (anchors == nullptr || segments == nullptr) {
    return false;
  }
  if (used_raw_high_precision != nullptr) {
    *used_raw_high_precision = false;
  }
  anchors->clear();
  segments->clear();
  if (dataset.gps_raw_keyframe_observations.empty()) {
    return BuildAlignmentAnchors(dataset, alignment_config, anchors, segments);
  }

  std::unordered_map<unsigned long, size_t> id_to_index;
  for (size_t i = 0; i < dataset.keyframes.size(); ++i) {
    if (dataset.keyframes[i]) {
      id_to_index[dataset.keyframes[i]->GetID()] = i;
    }
  }

  for (const auto& observation : dataset.gps_raw_keyframe_observations) {
    std::string reject_reason;
    if (!IsHighPrecisionRawGps(observation, lever_config, &reject_reason)) {
      continue;
    }
    auto it = id_to_index.find(observation.keyframe_id);
    if (it == id_to_index.end()) {
      continue;
    }
    const auto& keyframe = dataset.keyframes[it->second];
    if (!keyframe) {
      continue;
    }

    AlignmentAnchor anchor;
    anchor.keyframe_index = it->second;
    anchor.keyframe_id = observation.keyframe_id;
    anchor.timestamp = observation.timestamp;
    anchor.lio_position = keyframe->GetLIOPose().translation();
    anchor.gps_utm_position = observation.antenna_utm_position -
                              LeverArmOrientation(alignment.lio_to_utm_local,
                                                  observation, lever_config,
                                                  heading_bias_rad) *
                                  lever_arm;
    anchor.smoothed_gps_utm_position = anchor.gps_utm_position;
    anchor.gps_std_dev = observation.std_dev;
    anchor.weight = ComputeAnchorWeight(observation.std_dev, alignment_config);
    anchors->push_back(anchor);
  }

  if (anchors->size() < static_cast<size_t>(std::max(
                            alignment_config.min_alignment_anchors, 3))) {
    AWARN << "[Stage2] Not enough raw high-precision GPS anchors after "
             "lever-arm correction, fallback to stage1 GPS associations. raw="
          << anchors->size();
    return BuildAlignmentAnchors(dataset, alignment_config, anchors, segments);
  }

  if (used_raw_high_precision != nullptr) {
    *used_raw_high_precision = true;
  }

  std::sort(anchors->begin(), anchors->end(),
            [](const AlignmentAnchor& lhs, const AlignmentAnchor& rhs) {
              if (lhs.keyframe_index != rhs.keyframe_index) {
                return lhs.keyframe_index < rhs.keyframe_index;
              }
              return lhs.timestamp < rhs.timestamp;
            });

  int current_segment = -1;
  size_t segment_start = 0;
  for (size_t i = 0; i < anchors->size(); ++i) {
    if (i == 0 || DistanceXY((*anchors)[i - 1].gps_utm_position,
                             (*anchors)[i].gps_utm_position) >
                      alignment_config.gps_segment_break_distance_m) {
      if (i > 0) {
        GpsSegmentSummary summary;
        summary.segment_id = current_segment;
        summary.first_anchor_index = segment_start;
        summary.last_anchor_index = i - 1;
        summary.first_keyframe_id = (*anchors)[segment_start].keyframe_id;
        summary.last_keyframe_id = (*anchors)[i - 1].keyframe_id;
        summary.anchor_count = i - segment_start;
        segments->push_back(summary);
      }
      ++current_segment;
      segment_start = i;
    }
    (*anchors)[i].segment_id = current_segment;
  }

  GpsSegmentSummary summary;
  summary.segment_id = current_segment;
  summary.first_anchor_index = segment_start;
  summary.last_anchor_index = anchors->size() - 1;
  summary.first_keyframe_id = (*anchors)[segment_start].keyframe_id;
  summary.last_keyframe_id = anchors->back().keyframe_id;
  summary.anchor_count = anchors->size() - segment_start;
  segments->push_back(summary);

  for (auto& segment : *segments) {
    const size_t start = segment.first_anchor_index;
    const size_t end = segment.last_anchor_index;
    const size_t count = end - start + 1;
    std::vector<double> x(count, 0.0);
    std::vector<double> z(count, 0.0);
    std::vector<double> std_z(count, 0.0);
    for (size_t i = 0; i < count; ++i) {
      const auto& anchor = (*anchors)[start + i];
      if (i > 0) {
        x[i] = x[i - 1] + DistanceXY((*anchors)[start + i - 1].gps_utm_position,
                                     anchor.gps_utm_position);
      }
      z[i] = anchor.gps_utm_position.z();
      std_z[i] = anchor.gps_std_dev.z();
    }

    std::vector<double> smoothed_z;
    if (!FitSmoothedHeights(x, z, std_z, alignment_config, &smoothed_z)) {
      AERROR << "Failed to smooth lever-arm corrected GPS heights for segment "
             << segment.segment_id;
      return false;
    }

    double sum_abs_delta = 0.0;
    double max_abs_delta = 0.0;
    for (size_t i = 0; i < count; ++i) {
      auto& anchor = (*anchors)[start + i];
      anchor.smoothed_gps_utm_position.z() = smoothed_z[i];
      const double abs_delta = std::abs(smoothed_z[i] - z[i]);
      sum_abs_delta += abs_delta;
      max_abs_delta = std::max(max_abs_delta, abs_delta);
    }
    segment.length_xy_m = x.back();
    segment.mean_abs_z_smoothing_delta_m =
        sum_abs_delta / static_cast<double>(std::max<size_t>(count, 1));
    segment.max_abs_z_smoothing_delta_m = max_abs_delta;
  }

  AINFO << "[Stage2] Lever-arm corrected GPS anchors=" << anchors->size()
        << ", segments=" << segments->size();
  return true;
}

bool SolveWeightedRigidAlignment(const std::vector<AlignmentAnchor>& anchors,
                                 const AlignmentConfig& config,
                                 Stage2AlignmentResult* result) {
  if (result == nullptr) {
    return false;
  }
  if (anchors.size() <
      static_cast<size_t>(std::max(config.min_alignment_anchors, 3))) {
    AERROR << "Not enough GPS anchors for stage2 alignment: " << anchors.size()
           << " < " << config.min_alignment_anchors;
    return false;
  }

  result->utm_origin = anchors.front().smoothed_gps_utm_position;

  std::vector<double> robust_weights(anchors.size(), 1.0);
  const int iterations =
      config.robust_huber_delta_m > 0.0 ? config.robust_max_iterations : 1;

  for (int iteration = 0; iteration < iterations; ++iteration) {
    double weight_sum = 0.0;
    lightning::Vec3d source_center = lightning::Vec3d::Zero();
    lightning::Vec3d target_center = lightning::Vec3d::Zero();
    for (size_t i = 0; i < anchors.size(); ++i) {
      const auto& anchor = anchors[i];
      const double weight =
          std::max(anchor.weight, 1e-6) * robust_weights[i];
      const lightning::Vec3d target =
          anchor.smoothed_gps_utm_position - result->utm_origin;
      source_center += weight * anchor.lio_position;
      target_center += weight * target;
      weight_sum += weight;
    }
    if (weight_sum <= 0.0) {
      AERROR << "Invalid GPS anchor weights";
      return false;
    }
    source_center /= weight_sum;
    target_center /= weight_sum;

    lightning::Mat3d rotation = lightning::Mat3d::Identity();
    lightning::Vec3d translation = lightning::Vec3d::Zero();
    bool yaw_only = false;

    if (config.constrain_to_yaw_only) {
      double numerator = 0.0;
      double denominator = 0.0;
      for (size_t i = 0; i < anchors.size(); ++i) {
        const auto& anchor = anchors[i];
        const double weight =
            std::max(anchor.weight, 1e-6) * robust_weights[i];
        const lightning::Vec3d source = anchor.lio_position - source_center;
        const lightning::Vec3d target =
            anchor.smoothed_gps_utm_position - result->utm_origin -
            target_center;
        numerator +=
            weight * (source.x() * target.y() - source.y() * target.x());
        denominator +=
            weight * (source.x() * target.x() + source.y() * target.y());
      }
      const double yaw = std::atan2(numerator, denominator);
      rotation = lightning::SO3::rotZ(yaw).matrix();
      translation = target_center - rotation * source_center;
      yaw_only = true;
    } else {
      lightning::Mat3d covariance = lightning::Mat3d::Zero();
      lightning::Mat3d source_spread = lightning::Mat3d::Zero();
      for (size_t i = 0; i < anchors.size(); ++i) {
        const auto& anchor = anchors[i];
        const double weight =
            std::max(anchor.weight, 1e-6) * robust_weights[i];
        const lightning::Vec3d source = anchor.lio_position - source_center;
        const lightning::Vec3d target =
            anchor.smoothed_gps_utm_position - result->utm_origin -
            target_center;
        covariance += weight * source * target.transpose();
        source_spread += weight * source * source.transpose();
      }

      if (iteration == 0) {
        Eigen::JacobiSVD<lightning::Mat3d> spread_svd(source_spread);
        const auto spread_sv = spread_svd.singularValues();
        if (spread_sv(1) < 1e-3) {
          AWARN << "[Stage2] GPS/LIO anchors are close to collinear; roll/pitch "
                   "tilt may be weakly observable. singular_values="
                << spread_sv.transpose();
        }
      }

      Eigen::JacobiSVD<lightning::Mat3d> svd(
          covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
      rotation = svd.matrixV() * svd.matrixU().transpose();
      if (rotation.determinant() < 0.0) {
        lightning::Mat3d correction = lightning::Mat3d::Identity();
        correction(2, 2) = -1.0;
        rotation = svd.matrixV() * correction * svd.matrixU().transpose();
      }
      rotation = lightning::SO3::fitToSO3(rotation).matrix();
      translation = target_center - rotation * source_center;
    }

    result->lio_to_utm_local =
        lightning::SE3(lightning::SO3::fitToSO3(rotation), translation);
    result->anchor_count = anchors.size();
    result->yaw_only = yaw_only;
    result->success = true;

    if (iteration + 1 >= iterations) {
      break;
    }

    double max_weight_delta = 0.0;
    for (size_t i = 0; i < anchors.size(); ++i) {
      const auto& anchor = anchors[i];
      const lightning::Vec3d target =
          anchor.smoothed_gps_utm_position - result->utm_origin;
      const lightning::Vec3d aligned =
          result->lio_to_utm_local * anchor.lio_position;
      const double residual = (aligned - target).norm();
      const double next_weight =
          RobustWeight(residual, config.robust_huber_delta_m);
      max_weight_delta =
          std::max(max_weight_delta, std::abs(next_weight - robust_weights[i]));
      robust_weights[i] = next_weight;
    }
    if (max_weight_delta < 1e-3) {
      break;
    }
  }

  return true;
}

bool BuildCalibrationAlignmentAnchors(
    const std::vector<const Stage1GpsRawKeyframeObservation*>& samples,
    const Stage2AlignmentResult& alignment, const lightning::Vec3d& lever_arm,
    double heading_bias_rad,
    const LeverArmCalibrationConfig& lever_config,
    const AlignmentConfig& alignment_config,
    std::vector<AlignmentAnchor>* anchors) {
  if (anchors == nullptr) {
    return false;
  }
  anchors->clear();
  for (const auto* sample : samples) {
    if (sample == nullptr) {
      continue;
    }
    AlignmentAnchor anchor;
    anchor.keyframe_id = sample->keyframe_id;
    anchor.timestamp = sample->timestamp;
    anchor.lio_position = sample->lio_opt_pose.translation();
    anchor.gps_utm_position =
        sample->antenna_utm_position -
        LeverArmOrientation(alignment.lio_to_utm_local, *sample, lever_config,
                            heading_bias_rad) *
            lever_arm;
    anchor.smoothed_gps_utm_position = anchor.gps_utm_position;
    anchor.gps_std_dev = sample->std_dev;
    anchor.weight = ComputeAnchorWeight(sample->std_dev, alignment_config);
    anchors->push_back(anchor);
  }
  return !anchors->empty();
}

void FillLeverArmSampleDiagnostics(
    const Stage1Dataset& dataset,
    const std::vector<const Stage1GpsRawKeyframeObservation*>& selected_samples,
    const Stage2AlignmentResult& initial_alignment,
    const Stage2AlignmentResult& optimized_alignment,
    const lightning::Vec3d& initial_lever_arm,
    const lightning::Vec3d& optimized_lever_arm,
    double heading_bias_rad,
    const AlignmentConfig& alignment_config,
    const LeverArmCalibrationConfig& config,
    LeverArmCalibrationResult* result) {
  if (result == nullptr) {
    return;
  }

  std::unordered_map<unsigned long, const Stage1GpsRawKeyframeObservation*>
      selected_by_id;
  for (const auto* sample : selected_samples) {
    if (sample) {
      selected_by_id[sample->keyframe_id] = sample;
    }
  }

  double sum_initial = 0.0;
  double sum_optimized = 0.0;
  double max_initial = 0.0;
  double max_optimized = 0.0;
  size_t selected_count = 0;
  result->weighted_cost_initial = 0.0;
  result->weighted_cost_optimized = 0.0;
  result->samples.clear();
  result->samples.reserve(dataset.gps_raw_keyframe_observations.size());

  for (const auto& observation : dataset.gps_raw_keyframe_observations) {
    LeverArmCalibrationSampleDiagnostic diagnostic;
    diagnostic.keyframe_id = observation.keyframe_id;
    diagnostic.timestamp = observation.timestamp;
    diagnostic.antenna_utm_position = observation.antenna_utm_position;
    diagnostic.gps_std_dev = observation.std_dev;
    diagnostic.max_interp_gap_s =
        std::max(std::abs(observation.interp_gap_before_s),
                 std::abs(observation.interp_gap_after_s));
    diagnostic.lio_position = observation.lio_opt_pose.translation();
    diagnostic.lio_raw_yaw_rad = observation.lio_raw_yaw_rad;
    diagnostic.lio_opt_yaw_rad = observation.lio_opt_yaw_rad;
    diagnostic.gnss_heading_rad = observation.gnss_heading_rad;
    diagnostic.gnss_pitch_rad = observation.gnss_pitch_rad;
    diagnostic.gnss_lio_yaw_diff_deg = GnssLioYawDiffDeg(observation);
    diagnostic.heading_std_deg = observation.heading_std_deg;
    diagnostic.pitch_std_deg = observation.pitch_std_deg;
    diagnostic.sol_status = observation.sol_status;
    diagnostic.sol_type = observation.sol_type;
    diagnostic.satellite_tracked = observation.satellite_tracked;

    std::string reject_reason;
    diagnostic.selected =
        IsHighPrecisionRawGps(observation, config, &reject_reason);
    diagnostic.reject_reason = diagnostic.selected ? "selected" : reject_reason;
    if (selected_by_id.find(observation.keyframe_id) == selected_by_id.end()) {
      diagnostic.selected = false;
      if (diagnostic.reject_reason == "selected") {
        diagnostic.reject_reason = "not_used";
      }
    }

    if (observation.has_raw_gps &&
        observation.antenna_utm_position.allFinite()) {
      diagnostic.predicted_antenna_initial = PredictAntennaUtm(
          initial_alignment.lio_to_utm_local, observation, initial_lever_arm,
          initial_alignment.utm_origin, config);
      diagnostic.predicted_antenna_optimized = PredictAntennaUtm(
          optimized_alignment.lio_to_utm_local, observation,
          optimized_lever_arm, optimized_alignment.utm_origin, config,
          heading_bias_rad);
      diagnostic.residual_initial = observation.antenna_utm_position -
                                    diagnostic.predicted_antenna_initial;
      diagnostic.residual_optimized = observation.antenna_utm_position -
                                      diagnostic.predicted_antenna_optimized;
      diagnostic.residual_initial_xy_m =
          diagnostic.residual_initial.head<2>().norm();
      diagnostic.residual_optimized_xy_m =
          diagnostic.residual_optimized.head<2>().norm();
      diagnostic.weight =
          ComputeAnchorWeight(observation.std_dev, alignment_config);
    }

    if (diagnostic.selected) {
      const double robust_weight_initial =
          RobustWeight(diagnostic.residual_initial_xy_m,
                       config.huber_delta_xy_m);
      const double robust_weight =
          RobustWeight(diagnostic.residual_optimized_xy_m,
                       config.huber_delta_xy_m);
      sum_initial += diagnostic.residual_initial_xy_m;
      sum_optimized += diagnostic.residual_optimized_xy_m;
      result->weighted_cost_initial +=
          robust_weight_initial * diagnostic.weight *
          diagnostic.residual_initial_xy_m *
          diagnostic.residual_initial_xy_m;
      result->weighted_cost_optimized +=
          robust_weight * diagnostic.weight *
          diagnostic.residual_optimized_xy_m *
          diagnostic.residual_optimized_xy_m;
      max_initial = std::max(max_initial, diagnostic.residual_initial_xy_m);
      max_optimized =
          std::max(max_optimized, diagnostic.residual_optimized_xy_m);
      ++selected_count;
    }
    result->samples.push_back(diagnostic);
  }

  if (selected_count > 0) {
    result->mean_residual_initial_xy_m =
        sum_initial / static_cast<double>(selected_count);
    result->mean_residual_optimized_xy_m =
        sum_optimized / static_cast<double>(selected_count);
    result->max_residual_initial_xy_m = max_initial;
    result->max_residual_optimized_xy_m = max_optimized;
  }
}

bool RunLeverArmCalibration(const Stage1Dataset& dataset,
                            const Stage2Config& config,
                            const Stage2AlignmentResult& initial_alignment,
                            LeverArmCalibrationResult* result,
                            Stage2AlignmentResult* calibrated_alignment) {
  if (result == nullptr || calibrated_alignment == nullptr) {
    return false;
  }
  const auto& lever_config = config.lever_arm_calibration;
  *result = LeverArmCalibrationResult();
  result->enabled = lever_config.enable;
  result->orientation_model = "gnss_heading";
  result->estimate_heading_bias = lever_config.estimate_heading_bias;
  *calibrated_alignment = initial_alignment;

  if (!lever_config.enable) {
    result->status_message = "disabled";
    return true;
  }
  if (dataset.gps_raw_keyframe_observations.empty()) {
    result->status_message = "missing gps_keyframe_raw_assoc.csv";
    return true;
  }

  result->candidate_count = dataset.gps_raw_keyframe_observations.size();
  result->initial_lever_arm = FindConfiguredLeverArm(dataset);
  result->optimized_lever_arm = result->initial_lever_arm;

  std::vector<const Stage1GpsRawKeyframeObservation*> selected_samples;
  selected_samples.reserve(dataset.gps_raw_keyframe_observations.size());
  for (const auto& observation : dataset.gps_raw_keyframe_observations) {
    std::string reject_reason;
    if (!IsHighPrecisionRawGps(observation, lever_config, &reject_reason)) {
      continue;
    }
    selected_samples.push_back(&observation);
  }

  result->selected_count = selected_samples.size();
  if (selected_samples.size() <
      static_cast<size_t>(std::max(lever_config.min_samples, 3))) {
    result->status_message = "not enough high precision raw GPS samples";
    FillLeverArmSampleDiagnostics(dataset, selected_samples, initial_alignment,
                                  initial_alignment, result->initial_lever_arm,
                                  result->optimized_lever_arm, 0.0,
                                  config.alignment,
                                  lever_config,
                                  result);
    return true;
  }

  const Stage2AlignmentResult calibration_reference = initial_alignment;
  lightning::Vec3d current_lever_arm = result->initial_lever_arm;
  double current_heading_bias_rad = 0.0;
  bool had_failure = false;
  bool made_update = false;
  int iteration = 0;
  for (; iteration < lever_config.max_iterations; ++iteration) {
    lightning::Vec3d lever_step = lightning::Vec3d::Zero();
    double heading_bias_step = 0.0;
    if (!SolveLeverArmAndHeadingBiasStep(
            selected_samples, calibration_reference, result->initial_lever_arm,
            current_lever_arm, current_heading_bias_rad, lever_config,
            &lever_step, &heading_bias_step)) {
      result->status_message = "failed to solve lever arm calibration step";
      had_failure = true;
      break;
    }
    current_lever_arm += lever_step;
    current_heading_bias_rad += heading_bias_step;
    made_update = true;
    result->iterations = iteration + 1;
    if (lever_step.norm() < 1e-4 &&
        std::abs(heading_bias_step) < 1e-6) {
      break;
    }
  }

  result->optimized_lever_arm = current_lever_arm;
  result->heading_bias_rad = current_heading_bias_rad;
  result->correction = result->optimized_lever_arm - result->initial_lever_arm;
  result->correction_norm_m = result->correction.norm();
  if (!had_failure && made_update) {
    std::vector<AlignmentAnchor> calibration_anchors;
    if (!BuildCalibrationAlignmentAnchors(
            selected_samples, calibration_reference,
            result->optimized_lever_arm, result->heading_bias_rad, lever_config,
            config.alignment, &calibration_anchors)) {
      result->status_message = "failed to build calibration anchors";
      had_failure = true;
    } else {
      Stage2AlignmentResult next_alignment;
      if (!SolveWeightedRigidAlignment(calibration_anchors, config.alignment,
                                       &next_alignment)) {
        result->status_message = "failed to solve calibration alignment";
        had_failure = true;
      } else {
        *calibrated_alignment = next_alignment;
      }
    }
  }

  result->success = !had_failure && made_update;
  if (result->success && result->status_message.empty()) {
    result->status_message = "ok";
  }
  const Stage2AlignmentResult& diagnostic_alignment =
      result->success ? *calibrated_alignment : initial_alignment;
  FillLeverArmSampleDiagnostics(dataset, selected_samples, initial_alignment,
                                diagnostic_alignment, result->initial_lever_arm,
                                result->optimized_lever_arm,
                                result->heading_bias_rad, config.alignment,
                                lever_config, result);

  AINFO << "[Stage2] Lever-arm calibration: status=" << result->status_message
        << ", samples=" << result->selected_count << "/"
        << result->candidate_count
        << ", initial=" << result->initial_lever_arm.transpose()
        << ", optimized=" << result->optimized_lever_arm.transpose()
        << ", correction=" << result->correction.transpose()
        << ", heading_bias_deg=" << result->heading_bias_rad * kRadToDeg
        << ", mean_xy=" << result->mean_residual_initial_xy_m << " -> "
        << result->mean_residual_optimized_xy_m;
  return true;
}

void FillAlignmentDiagnostics(std::vector<AlignmentAnchor>* anchors,
                              Stage2AlignmentResult* result) {
  if (anchors == nullptr || result == nullptr || anchors->empty()) {
    return;
  }

  const lightning::Vec3d baseline_translation =
      anchors->front().smoothed_gps_utm_position - result->utm_origin -
      anchors->front().lio_position;

  double sum_before = 0.0;
  double max_before = 0.0;
  double sum_after = 0.0;
  double max_after = 0.0;
  for (auto& anchor : *anchors) {
    const lightning::Vec3d target =
        anchor.smoothed_gps_utm_position - result->utm_origin;
    const lightning::Vec3d baseline =
        anchor.lio_position + baseline_translation;
    const lightning::Vec3d aligned =
        result->lio_to_utm_local * anchor.lio_position;
    anchor.residual_before_m = (baseline - target).norm();
    anchor.residual_after_m = (aligned - target).norm();
    sum_before += anchor.residual_before_m;
    max_before = std::max(max_before, anchor.residual_before_m);
    sum_after += anchor.residual_after_m;
    max_after = std::max(max_after, anchor.residual_after_m);
  }

  result->mean_residual_before_m =
      sum_before / static_cast<double>(anchors->size());
  result->max_residual_before_m = max_before;
  result->mean_residual_after_m =
      sum_after / static_cast<double>(anchors->size());
  result->max_residual_after_m = max_after;

  const auto rotation = result->lio_to_utm_local.rotationMatrix();
  const double sy = std::sqrt(rotation(0, 0) * rotation(0, 0) +
                              rotation(1, 0) * rotation(1, 0));
  if (sy > 1e-6) {
    result->roll_deg = std::atan2(rotation(2, 1), rotation(2, 2)) * kRadToDeg;
    result->pitch_deg = std::atan2(-rotation(2, 0), sy) * kRadToDeg;
    result->yaw_deg = std::atan2(rotation(1, 0), rotation(0, 0)) * kRadToDeg;
  } else {
    result->roll_deg = std::atan2(-rotation(1, 2), rotation(1, 1)) * kRadToDeg;
    result->pitch_deg = std::atan2(-rotation(2, 0), sy) * kRadToDeg;
    result->yaw_deg = 0.0;
  }
}

void ApplyAlignmentToAllKeyframes(const Stage2AlignmentResult& result,
                                  Stage1Dataset* dataset) {
  if (dataset == nullptr) {
    return;
  }
  for (const auto& keyframe : dataset->keyframes) {
    if (!keyframe) {
      continue;
    }
    keyframe->SetOptPose(result.lio_to_utm_local * keyframe->GetLIOPose());
  }
}

lightning::CloudPtr BuildPreviewMap(const Stage2Config& config,
                                    const Stage1Dataset& dataset) {
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

bool Stage2Runner::Run(const Stage2Config& config) {
  Stage1Dataset dataset;
  Stage1ArtifactReader reader;
  const bool load_clouds = config.output.save_preview_map;
  if (!reader.Read(config, load_clouds, &dataset)) {
    return false;
  }

  std::vector<AlignmentAnchor> anchors;
  std::vector<GpsSegmentSummary> segments;
  bool initial_anchors_from_raw_gps = false;
  if (config.lever_arm_calibration.enable &&
      !dataset.gps_raw_keyframe_observations.empty()) {
    Stage2AlignmentResult identity_alignment;
    if (!BuildLeverArmCorrectedAlignmentAnchors(
            dataset, identity_alignment, FindConfiguredLeverArm(dataset), 0.0,
            config.lever_arm_calibration, config.alignment, &anchors,
            &segments, &initial_anchors_from_raw_gps)) {
      return false;
    }
  } else if (!BuildAlignmentAnchors(dataset, config.alignment, &anchors,
                                    &segments)) {
    return false;
  }

  Stage2AlignmentResult initial_result;
  initial_result.segment_count = segments.size();
  if (!SolveWeightedRigidAlignment(anchors, config.alignment,
                                   &initial_result)) {
    return false;
  }
  AINFO << "[Stage2] Initial alignment anchors source="
        << (initial_anchors_from_raw_gps ? "raw_high_precision_gps"
                                         : "stage1_gps_assoc");

  LeverArmCalibrationResult lever_arm_result;
  Stage2AlignmentResult calibrated_alignment;
  if (!RunLeverArmCalibration(dataset, config, initial_result,
                              &lever_arm_result, &calibrated_alignment)) {
    return false;
  }

  if (lever_arm_result.success) {
    anchors.clear();
    segments.clear();
    if (!BuildLeverArmCorrectedAlignmentAnchors(
            dataset, calibrated_alignment, lever_arm_result.optimized_lever_arm,
            lever_arm_result.heading_bias_rad, config.lever_arm_calibration,
            config.alignment, &anchors, &segments, nullptr)) {
      return false;
    }
  }

  Stage2AlignmentResult result;
  result.segment_count = segments.size();
  if (!SolveWeightedRigidAlignment(anchors, config.alignment, &result)) {
    return false;
  }
  FillAlignmentDiagnostics(&anchors, &result);

  AINFO << "[Stage2] alignment solved:"
        << " model=" << (result.yaw_only ? "yaw_only" : "se3")
        << " anchors=" << result.anchor_count
        << ", segments=" << result.segment_count
        << ", residual_before_mean=" << result.mean_residual_before_m
        << "m, residual_after_mean=" << result.mean_residual_after_m
        << "m, rpy_deg=[" << result.roll_deg << ", " << result.pitch_deg << ", "
        << result.yaw_deg << "]";

  ApplyAlignmentToAllKeyframes(result, &dataset);

  lightning::CloudPtr preview_map;
  if (config.output.save_preview_map) {
    preview_map = BuildPreviewMap(config, dataset);
    AINFO << "[Stage2] Preview map points: "
          << (preview_map ? preview_map->size() : 0);
  }

  Stage2ArtifactWriter writer;
  return writer.Write(config, dataset, anchors, segments, result,
                      lever_arm_result, preview_map);
}

}  // namespace stage2
}  // namespace air_mapping
}  // namespace apollo
