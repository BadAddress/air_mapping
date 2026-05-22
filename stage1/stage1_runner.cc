#include "modules/air_mapping/stage1/stage1_runner.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <system_error>

#include "cyber/common/log.h"
#include "cyber/record/record_reader.h"
#include "modules/air_mapping/stage1/stage1_artifact_writer.h"
#include "modules/air_mapping/stage1/stage1_loop_optimizer.h"
#include "modules/air_mapping/system/common/artifact_utils.h"

namespace apollo {
namespace air_mapping {
namespace stage1 {

using apollo::cyber::record::RecordMessage;
using apollo::cyber::record::RecordReader;

namespace {

bool IsHighPrecisionGpsSolType(uint32_t sol_type, uint32_t required_sol_type) {
  if (sol_type == required_sol_type) {
    return true;
  }
  return sol_type == 50U || sol_type == 56U;
}

}  // namespace

bool Stage1Runner::Run(const Stage1Config& config) {
  config_ = config;
  if (!PrepareCleanOutputDirectory(config_.output.directory, "stage1")) {
    return false;
  }

  lightning::SlamSystem::Options options;
  options.online_mode_ = false;
  slam_system_ = std::make_shared<lightning::SlamSystem>(options);
  if (!slam_system_->Init(config_.algorithm_config_path)) {
    AERROR << "Failed to initialize SlamSystem with "
           << config_.algorithm_config_path;
    return false;
  }
  slam_system_->StartSLAM(config_.map_name);

  if (config_.dual_lidar.enable) {
    dual_lidar_fusion_ = std::make_unique<DualLidarFusion>();
    if (!dual_lidar_fusion_->Init(config_.dual_lidar.config_path)) {
      AERROR << "Failed to initialize dual LiDAR fusion with "
             << config_.dual_lidar.config_path;
      return false;
    }
    if (!dual_lidar_fusion_->enabled()) {
      dual_lidar_fusion_.reset();
    }
  }

  const std::filesystem::path gps_odom_path =
      std::filesystem::path(config_.output.directory) / "gps" / "gps_odom_gt.tum";
  std::error_code error;
  std::filesystem::create_directories(gps_odom_path.parent_path(), error);
  if (error) {
    AERROR << "Failed to create GPS odom output directory: "
           << gps_odom_path.parent_path().string()
           << ", error: " << error.message();
    return false;
  }
  gps_odom_recorder_ = std::make_unique<lightning::GpsOdomRecorder>();
  gps_odom_recorder_->Init(gps_odom_path.string(), config_.map_name);

  for (const auto& record : config_.records) {
    if (!ProcessRecord(record)) {
      return false;
    }
  }
  if (dual_lidar_fusion_) {
    for (const auto& cloud : dual_lidar_fusion_->FlushPrimaryOnly()) {
      slam_system_->ProcessLidar(cloud);
    }
  }

  if (gps_odom_recorder_) {
    gps_odom_recorder_->Close();
  }

  const auto keyframes = slam_system_->GetAllKeyframes();
  const auto gps_history = slam_system_->GetGpsFullHistory();

  std::vector<Stage1LoopConstraint> loop_constraints;
  Stage1LoopSummary loop_summary;
  Stage1LoopOptimizer loop_optimizer;
  if (!loop_optimizer.Optimize(config_, keyframes, &loop_constraints,
                               &loop_summary)) {
    AERROR << "Stage1 LiDAR-only loop optimization failed";
    return false;
  }

  const Stage1GpsZLevelingResult z_leveling_result =
      ApplyGpsZLeveling(keyframes);

  lightning::CloudPtr preview_map;
  if (config_.output.save_preview_map && !keyframes.empty()) {
    preview_map = slam_system_->GetGlobalMapFromKeyframes(
        keyframes, true, config_.output.preview_voxel_size);
  }

  Stage1ArtifactWriter writer;
  return writer.Write(config_, keyframes, gps_history, loop_constraints,
                      loop_summary, z_leveling_result, preview_map);
}

Stage1GpsZLevelingResult Stage1Runner::ApplyGpsZLeveling(
    const std::vector<lightning::Keyframe::Ptr>& keyframes) const {
  Stage1GpsZLevelingResult result;
  result.enabled = config_.gps_z_leveling.enable;
  if (!result.enabled) {
    result.status_message = "disabled";
    return result;
  }

  double gps_z_sum = 0.0;
  double lio_z_sum = 0.0;
  const auto& leveling = config_.gps_z_leveling;
  for (const auto& keyframe : keyframes) {
    if (!keyframe) {
      continue;
    }
    const auto gps_data = keyframe->GetGpsData();
    if (!gps_data.has_gps || !gps_data.gps_utm_position.allFinite()) {
      continue;
    }

    Stage1GpsZLevelingSample sample;
    sample.keyframe_id = keyframe->GetID();
    sample.timestamp = keyframe->GetTimestamp();
    sample.gps_z = gps_data.gps_utm_position.z();
    sample.lio_z_before = keyframe->GetOptPose().translation().z();
    sample.lio_z_after = sample.lio_z_before;
    sample.std_x = gps_data.gps_std_dev.x();
    sample.std_y = gps_data.gps_std_dev.y();
    sample.std_z = gps_data.gps_std_dev.z();
    sample.sol_type = gps_data.sol_type;

    const double std_xy =
        std::max(std::abs(sample.std_x), std::abs(sample.std_y));
    if (!std::isfinite(sample.gps_z) || !std::isfinite(sample.lio_z_before) ||
        !std::isfinite(std_xy) || !std::isfinite(sample.std_z)) {
      sample.reject_reason = "non_finite";
    } else if (!(sample.std_x > 0.0) || !(sample.std_y > 0.0) ||
               !(sample.std_z > 0.0)) {
      sample.reject_reason = "std_not_positive";
    } else if (std_xy > leveling.max_gps_std_xy_m) {
      sample.reject_reason = "std_xy_too_large";
    } else if (std::abs(sample.std_z) > leveling.max_gps_std_z_m) {
      sample.reject_reason = "std_z_too_large";
    } else if (leveling.require_rtk_fixed &&
               !IsHighPrecisionGpsSolType(sample.sol_type,
                                          leveling.required_sol_type)) {
      sample.reject_reason = "sol_type_mismatch";
    } else {
      sample.selected = true;
      sample.reject_reason = "selected";
      gps_z_sum += sample.gps_z;
      lio_z_sum += sample.lio_z_before;
      ++result.selected_count;
    }
    result.samples.push_back(sample);
  }

  result.candidate_count = result.samples.size();
  if (result.selected_count <
      static_cast<size_t>(std::max(leveling.min_samples, 1))) {
    result.status_message = "not_enough_samples";
    AWARN << "[Stage1GpsZLeveling] skip: selected=" << result.selected_count
          << " < min_samples=" << leveling.min_samples;
    return result;
  }

  result.gps_mean_z = gps_z_sum / static_cast<double>(result.selected_count);
  result.lio_mean_z_before =
      lio_z_sum / static_cast<double>(result.selected_count);
  result.z_offset_m = result.gps_mean_z - result.lio_mean_z_before;
  if (!std::isfinite(result.z_offset_m)) {
    result.status_message = "non_finite_offset";
    return result;
  }
  if (std::abs(result.z_offset_m) > leveling.max_abs_z_offset_m) {
    result.status_message = "offset_too_large";
    AWARN << "[Stage1GpsZLeveling] skip: z_offset=" << result.z_offset_m
          << " exceeds max_abs_z_offset_m=" << leveling.max_abs_z_offset_m;
    return result;
  }

  for (const auto& keyframe : keyframes) {
    if (!keyframe) {
      continue;
    }
    lightning::SE3 pose = keyframe->GetOptPose();
    lightning::Vec3d translation = pose.translation();
    translation.z() += result.z_offset_m;
    keyframe->SetOptPose(lightning::SE3(pose.so3(), translation));
  }
  for (auto& sample : result.samples) {
    sample.lio_z_after = sample.lio_z_before + result.z_offset_m;
  }
  result.lio_mean_z_after = result.lio_mean_z_before + result.z_offset_m;
  result.applied = true;
  result.status_message = "applied";
  AINFO << "[Stage1GpsZLeveling] applied: selected=" << result.selected_count
        << ", gps_mean_z=" << result.gps_mean_z
        << ", lio_mean_z_before=" << result.lio_mean_z_before
        << ", z_offset_m=" << result.z_offset_m;
  return result;
}

bool Stage1Runner::ProcessRecord(const std::string& record_path) {
  AINFO << "Processing record: " << record_path;
  RecordReader reader(record_path);
  if (!reader.IsValid()) {
    AERROR << "Invalid record file: " << record_path;
    return false;
  }

  RecordMessage message;
  uint64_t processed = 0;
  while (reader.ReadMessage(&message)) {
    if (dual_lidar_fusion_ &&
        message.channel_name == config_.dual_lidar.primary_channel) {
      auto cloud = std::make_shared<apollo::drivers::PointCloud>();
      if (cloud->ParseFromString(message.content)) {
        ProcessLidarCloud(cloud, true);
      }
    } else if (dual_lidar_fusion_ &&
               message.channel_name == config_.dual_lidar.secondary_channel) {
      auto cloud = std::make_shared<apollo::drivers::PointCloud>();
      if (cloud->ParseFromString(message.content)) {
        ProcessLidarCloud(cloud, false);
      }
    } else if (!dual_lidar_fusion_ &&
               message.channel_name == config_.channels.lidar) {
      auto cloud = std::make_shared<apollo::drivers::PointCloud>();
      if (cloud->ParseFromString(message.content)) {
        ProcessLidarCloud(cloud, true);
      }
    } else if (message.channel_name == config_.channels.imu) {
      if (config_.channels.imu_type == "raw") {
        apollo::drivers::gnss::Imu imu;
        if (imu.ParseFromString(message.content)) {
          ProcessRawImu(imu);
        }
      } else {
        apollo::localization::CorrectedImu imu;
        if (imu.ParseFromString(message.content)) {
          ProcessCorrectedImu(imu);
        }
      }
    } else if (message.channel_name == config_.channels.ins_stat) {
      apollo::drivers::gnss::InsStat ins_stat;
      if (ins_stat.ParseFromString(message.content)) {
        ProcessInsStat(ins_stat);
      }
    } else if (message.channel_name == config_.channels.heading) {
      apollo::drivers::gnss::Heading heading;
      if (heading.ParseFromString(message.content)) {
        ProcessHeading(heading);
      }
    } else if (message.channel_name == config_.channels.gnss_best_pose) {
      apollo::drivers::gnss::GnssBestPose best_pose;
      if (best_pose.ParseFromString(message.content)) {
        ProcessBestPose(best_pose);
      }
    } else if (message.channel_name == config_.channels.gps_odom) {
      apollo::localization::Gps gps;
      if (gps.ParseFromString(message.content)) {
        ProcessGpsOdom(gps);
      }
    }

    ++processed;
    if (processed % 10000 == 0) {
      AINFO << "Processed " << processed << " record messages from "
            << record_path;
    }
  }

  AINFO << "Finished record: " << record_path
        << ", messages=" << processed;
  return true;
}

void Stage1Runner::ProcessLidarCloud(
    const std::shared_ptr<apollo::drivers::PointCloud>& cloud,
    bool is_primary) {
  if (!dual_lidar_fusion_) {
    slam_system_->ProcessLidar(cloud);
    return;
  }
  if (is_primary) {
    dual_lidar_fusion_->PushPrimary(cloud);
  } else {
    dual_lidar_fusion_->PushSecondary(cloud);
  }
  const auto fused = dual_lidar_fusion_->TryPopFused();
  if (fused) {
    slam_system_->ProcessLidar(fused);
  }
}

void Stage1Runner::ProcessCorrectedImu(
    const apollo::localization::CorrectedImu& imu) {
  if (!imu.has_imu()) {
    return;
  }
  auto l_imu = std::make_shared<lightning::IMU>();
  l_imu->timestamp =
      imu.header().timestamp_sec() - slam_system_->GetGnssLidarTimeOffset();

  const auto& pose = imu.imu();
  if (pose.has_angular_velocity()) {
    l_imu->angular_velocity = lightning::Vec3d(
        pose.angular_velocity().x(), pose.angular_velocity().y(),
        pose.angular_velocity().z());
  }
  if (pose.has_linear_acceleration()) {
    l_imu->linear_acceleration = lightning::Vec3d(
        pose.linear_acceleration().x(), pose.linear_acceleration().y(),
        pose.linear_acceleration().z());
  }
  slam_system_->ProcessIMU(l_imu);
}

void Stage1Runner::ProcessRawImu(const apollo::drivers::gnss::Imu& imu) {
  auto l_imu = std::make_shared<lightning::IMU>();
  l_imu->timestamp = imu.header().timestamp_sec();
  if (imu.has_angular_velocity()) {
    l_imu->angular_velocity = lightning::Vec3d(
        imu.angular_velocity().x(), imu.angular_velocity().y(),
        imu.angular_velocity().z());
  }
  if (imu.has_linear_acceleration()) {
    l_imu->linear_acceleration = lightning::Vec3d(
        imu.linear_acceleration().x(), imu.linear_acceleration().y(),
        imu.linear_acceleration().z());
  }
  slam_system_->ProcessIMU(l_imu);
}

void Stage1Runner::ProcessHeading(
    const apollo::drivers::gnss::Heading& heading_msg) {
  if (!config_.gps_gate.enable_gps_heading_init) {
    return;
  }
  if (heading_msg.has_solution_status() &&
      heading_msg.solution_status() != apollo::drivers::gnss::SOL_COMPUTED) {
    return;
  }
  if (heading_msg.has_position_type() &&
      !IsAcceptedRtkSolutionType(heading_msg.position_type())) {
    return;
  }
  if (!heading_msg.has_heading()) {
    return;
  }
  lightning::HeadingObservation heading;
  const double raw_timestamp = GetHeadingTimestampSec(heading_msg);
  if (raw_timestamp <= 0.0) {
    return;
  }
  heading.timestamp = raw_timestamp - slam_system_->GetGnssLidarTimeOffset();
  heading.heading = heading_msg.has_heading() ? heading_msg.heading() : 0.0;
  heading.pitch = heading_msg.has_pitch() ? heading_msg.pitch() : 0.0;
  heading.heading_std_dev =
      heading_msg.has_heading_std_dev() ? heading_msg.heading_std_dev() : 0.0;
  heading.pitch_std_dev =
      heading_msg.has_pitch_std_dev() ? heading_msg.pitch_std_dev() : 0.0;
  heading.satellite_tracked =
      heading_msg.has_satellite_tracked_number()
          ? static_cast<int>(heading_msg.satellite_tracked_number())
          : 0;
  heading.is_valid =
      heading.heading_std_dev < config_.gps_gate.heading_std_threshold;
  if (!heading.is_valid) {
    return;
  }

  slam_system_->ProcessHeading(heading);
}

void Stage1Runner::ProcessBestPose(
    const apollo::drivers::gnss::GnssBestPose& best_pose) {
  if (!config_.gps_gate.enable_gps_heading_init) {
    return;
  }
  if (!IsGpsSolutionValid(best_pose)) {
    return;
  }
  auto gps = std::make_shared<apollo::drivers::gnss::GnssBestPose>(best_pose);
  slam_system_->ProcessGPS(gps);
}

void Stage1Runner::ProcessInsStat(
    const apollo::drivers::gnss::InsStat& ins_stat) {
  latest_ins_status_.store(ins_stat.has_ins_status() ? ins_stat.ins_status() : 0U);
  latest_pos_type_.store(ins_stat.has_pos_type() ? ins_stat.pos_type() : 0U);
  has_ins_stat_.store(true);
}

void Stage1Runner::ProcessGpsOdom(const apollo::localization::Gps& gps) {
  if (!gps_odom_recorder_ || !gps_odom_recorder_->IsInitialized() ||
      !gps.has_localization()) {
    return;
  }

  const auto& localization = gps.localization();
  const double timestamp = gps.has_header() ? gps.header().timestamp_sec() : 0.0;
  const double x = localization.has_position() ? localization.position().x() : 0.0;
  const double y = localization.has_position() ? localization.position().y() : 0.0;
  const double z = localization.has_position() ? localization.position().z() : 0.0;
  const double qx =
      localization.has_orientation() ? localization.orientation().qx() : 0.0;
  const double qy =
      localization.has_orientation() ? localization.orientation().qy() : 0.0;
  const double qz =
      localization.has_orientation() ? localization.orientation().qz() : 0.0;
  const double qw =
      localization.has_orientation() ? localization.orientation().qw() : 1.0;
  gps_odom_recorder_->RecordPose(timestamp, x, y, z, qx, qy, qz, qw);
}

bool Stage1Runner::IsGpsInsValid() const {
  if (!config_.gps_gate.enable_ins_gate) {
    return true;
  }
  if (!has_ins_stat_.load()) {
    return false;
  }
  return latest_ins_status_.load() == config_.gps_gate.ins_gate_status &&
         latest_pos_type_.load() == config_.gps_gate.ins_gate_pos_type;
}

bool Stage1Runner::IsGpsSolutionValid(
    const apollo::drivers::gnss::GnssBestPose& msg) const {
  if (!msg.has_sol_status() || !msg.has_sol_type()) {
    return false;
  }
  return msg.sol_status() == apollo::drivers::gnss::SOL_COMPUTED &&
         IsAcceptedRtkSolutionType(msg.sol_type());
}

bool Stage1Runner::IsAcceptedRtkSolutionType(uint32_t type) const {
  using apollo::drivers::gnss::SolutionType;
  switch (type) {
    case SolutionType::L1_FLOAT:
    case SolutionType::IONOFREE_FLOAT:
    case SolutionType::NARROW_FLOAT:
    case SolutionType::L1_INT:
    case SolutionType::WIDE_INT:
    case SolutionType::NARROW_INT:
    case SolutionType::INS_RTKFLOAT:
    case SolutionType::INS_RTKFIXED:
      return true;
    default:
      return false;
  }
}

double Stage1Runner::GetHeadingTimestampSec(
    const apollo::drivers::gnss::Heading& msg) const {
  if (msg.has_measurement_time() && msg.measurement_time() > 0.0) {
    return msg.measurement_time();
  }
  if (msg.has_header()) {
    return msg.header().timestamp_sec();
  }
  return 0.0;
}

}  // namespace stage1
}  // namespace air_mapping
}  // namespace apollo
