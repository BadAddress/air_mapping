#include "modules/air_mapping/stage1/stage1_artifact_writer.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <system_error>
#include <vector>

#include "pcl/io/pcd_io.h"
#include "yaml-cpp/yaml.h"

#include "cyber/common/log.h"
#include "modules/air_mapping/system/common/debug_utils.h"
#include "modules/air_mapping/system/common/artifact_utils.h"

namespace apollo {
namespace air_mapping {
namespace stage1 {

namespace {

constexpr double kRadToDeg = 180.0 / M_PI;

struct InterpolatedGpsFullObservation {
  bool success = false;
  size_t before_index = 0;
  size_t after_index = 0;
  double before_timestamp = 0.0;
  double after_timestamp = 0.0;
  double gap_before_s = 0.0;
  double gap_after_s = 0.0;
  double alpha = 0.0;
  lightning::GpsFullObservation observation;
};

bool EnsureDirectory(const std::filesystem::path& path) {
  std::error_code error;
  std::filesystem::create_directories(path, error);
  if (error) {
    AERROR << "Failed to create directory: " << path.string()
           << ", error: " << error.message();
    return false;
  }
  return true;
}

double PoseYawRad(const lightning::SE3& pose) {
  const lightning::Mat3d rotation = pose.rotationMatrix();
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

double PoseRollRad(const lightning::SE3& pose) {
  const lightning::Mat3d rotation = pose.rotationMatrix();
  return std::atan2(rotation(2, 1), rotation(2, 2));
}

double PosePitchRad(const lightning::SE3& pose) {
  const lightning::Mat3d rotation = pose.rotationMatrix();
  const double sy = std::sqrt(rotation(0, 0) * rotation(0, 0) +
                              rotation(1, 0) * rotation(1, 0));
  return std::atan2(-rotation(2, 0), sy);
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

double InterpolateAngleRad(double before, double after, double alpha) {
  return NormalizeAngleRad(before + alpha * NormalizeAngleRad(after - before));
}

bool LoadGpsLeverArm(const std::string& config_path,
                     lightning::Vec3d* lever_arm) {
  if (lever_arm == nullptr) {
    return false;
  }
  *lever_arm = lightning::Vec3d::Zero();
  std::error_code error;
  if (!std::filesystem::exists(config_path, error)) {
    AWARN << "Algorithm config not found, write zero GPS lever arm: "
          << config_path;
    return true;
  }

  try {
    const YAML::Node yaml = YAML::LoadFile(config_path);
    if (!yaml["gps_heading_init"] ||
        !yaml["gps_heading_init"]["gps_to_imu_translation"]) {
      AWARN << "gps_heading_init.gps_to_imu_translation missing in "
            << config_path << ", write zero GPS lever arm";
      return true;
    }
    const auto values = yaml["gps_heading_init"]["gps_to_imu_translation"]
                            .as<std::vector<double>>();
    if (values.size() != 3) {
      AERROR << "Invalid gps_to_imu_translation size in " << config_path;
      return false;
    }
    *lever_arm = lightning::Vec3d(values[0], values[1], values[2]);
  } catch (const std::exception& e) {
    AERROR << "Failed to load GPS lever arm from " << config_path
           << ", error: " << e.what();
    return false;
  }
  return true;
}

bool InterpolateGpsHistoryAt(
    const std::vector<lightning::GpsFullObservation>& gps_history,
    double timestamp, InterpolatedGpsFullObservation* result) {
  if (result == nullptr) {
    return false;
  }
  *result = InterpolatedGpsFullObservation();
  if (gps_history.size() < 2) {
    return false;
  }

  const auto after_hint = std::lower_bound(
      gps_history.begin(), gps_history.end(), timestamp,
      [](const lightning::GpsFullObservation& gps, double query_timestamp) {
        return gps.timestamp < query_timestamp;
      });
  const size_t after_hint_index =
      static_cast<size_t>(std::distance(gps_history.begin(), after_hint));

  size_t after_index = gps_history.size();
  for (size_t i = after_hint_index; i < gps_history.size(); ++i) {
    if (gps_history[i].timestamp >= timestamp && gps_history[i].is_valid &&
        gps_history[i].antenna_utm.allFinite()) {
      after_index = i;
      break;
    }
  }

  size_t before_index = gps_history.size();
  size_t before_scan_index = gps_history.size();
  if (after_hint_index < gps_history.size() &&
      gps_history[after_hint_index].timestamp <= timestamp) {
    before_scan_index = after_hint_index;
  } else if (after_hint_index > 0) {
    before_scan_index = after_hint_index - 1;
  }
  if (before_scan_index < gps_history.size()) {
    for (size_t reverse_i = before_scan_index + 1; reverse_i > 0; --reverse_i) {
      const size_t i = reverse_i - 1;
      if (gps_history[i].timestamp <= timestamp && gps_history[i].is_valid &&
          gps_history[i].antenna_utm.allFinite()) {
        before_index = i;
        break;
      }
    }
  }

  if (before_index == gps_history.size() || after_index == gps_history.size()) {
    return false;
  }
  const auto& before = gps_history[before_index];
  const auto& after = gps_history[after_index];
  if (before_index == after_index) {
    result->success = true;
    result->before_index = before_index;
    result->after_index = after_index;
    result->before_timestamp = before.timestamp;
    result->after_timestamp = after.timestamp;
    result->gap_before_s = timestamp - before.timestamp;
    result->gap_after_s = before.timestamp - timestamp;
    result->alpha = 0.0;
    result->observation = before;
    result->observation.timestamp = timestamp;
    return result->observation.is_valid &&
           result->observation.antenna_utm.allFinite();
  }
  const double dt = after.timestamp - before.timestamp;
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    return false;
  }

  const double alpha = (timestamp - before.timestamp) / dt;
  if (!std::isfinite(alpha)) {
    return false;
  }

  result->success = true;
  result->before_index = before_index;
  result->after_index = after_index;
  result->before_timestamp = before.timestamp;
  result->after_timestamp = after.timestamp;
  result->gap_before_s = timestamp - before.timestamp;
  result->gap_after_s = after.timestamp - timestamp;
  result->alpha = alpha;
  auto& observation = result->observation;
  observation.timestamp = timestamp;
  observation.antenna_utm =
      before.antenna_utm + alpha * (after.antenna_utm - before.antenna_utm);
  observation.imu_utm =
      before.imu_utm + alpha * (after.imu_utm - before.imu_utm);
  observation.heading_rad =
      InterpolateAngleRad(before.heading_rad, after.heading_rad, alpha);
  observation.pitch_rad =
      InterpolateAngleRad(before.pitch_rad, after.pitch_rad, alpha);
  const auto& quality =
      result->gap_before_s <= result->gap_after_s ? before : after;
  observation.position_std_dev = quality.position_std_dev;
  observation.sol_status = quality.sol_status;
  observation.sol_type = quality.sol_type;
  observation.heading_std_dev = quality.heading_std_dev;
  observation.pitch_std_dev = quality.pitch_std_dev;
  observation.satellite_tracked = quality.satellite_tracked;
  observation.has_position = quality.has_position;
  observation.has_heading = quality.has_heading;
  observation.is_valid =
      quality.is_valid && observation.antenna_utm.allFinite();
  return observation.is_valid;
}

void WritePoseCsv(std::ofstream& file, const lightning::SE3& pose) {
  const auto& translation = pose.translation();
  const auto& quaternion = pose.unit_quaternion();
  file << std::setprecision(9) << translation.x() << "," << translation.y()
       << "," << translation.z() << "," << quaternion.x() << ","
       << quaternion.y() << "," << quaternion.z() << "," << quaternion.w();
}

std::string FormatKeyframeCloudName(unsigned long id) {
  std::ostringstream stream;
  stream << std::setw(6) << std::setfill('0') << id << ".pcd";
  return stream.str();
}

std::string FormatKeyframeDataName(unsigned long id,
                                   const std::string& extension) {
  std::ostringstream stream;
  stream << std::setw(6) << std::setfill('0') << id << extension;
  return stream.str();
}

bool WriteMatrix6(const std::filesystem::path& path,
                  const lightning::Mat6d& matrix) {
  std::ofstream file(path);
  if (!file.is_open()) {
    AERROR << "Failed to open covariance file: " << path.string();
    return false;
  }
  file << std::fixed << std::setprecision(12);
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      if (col > 0) {
        file << " ";
      }
      file << matrix(row, col);
    }
    file << "\n";
  }
  return true;
}

void WriteUtmAlignment(
    const std::filesystem::path& path,
    const std::vector<lightning::Keyframe::Ptr>& keyframes,
    const Stage1ZLevelingResult& z_leveling_result) {
  lightning::Vec3d offset_sum = lightning::Vec3d::Zero();
  int gps_count = 0;
  for (const auto& keyframe : keyframes) {
    if (!keyframe) {
      continue;
    }
    const auto gps_data = keyframe->GetGpsData();
    if (!gps_data.has_gps || !gps_data.gps_utm_position.allFinite()) {
      continue;
    }
    const lightning::Vec3d local_position =
        keyframe->GetOptPose().translation();
    const lightning::Vec3d offset = gps_data.gps_utm_position - local_position;
    if (!offset.allFinite()) {
      continue;
    }
    offset_sum += offset;
    ++gps_count;
  }

  lightning::Vec3d offset = lightning::Vec3d::Zero();
  if (gps_count > 0) {
    offset = offset_sum / static_cast<double>(gps_count);
  }

  std::ofstream file(path);
  if (!file.is_open()) {
    AERROR << "Failed to write UTM alignment file: " << path.string();
    return;
  }
  file << "# UTM Alignment File\n";
  file << "# Stage1 keyframe poses and PCD maps stay in local LIO "
          "coordinates.\n";
  file
      << "# GPS heading may align the local axes with ENU/UTM direction, but\n";
  file << "# absolute UTM translation is stored here only and is not baked "
          "into PCD.\n";
  file << "# p_utm = p_local + offset\n";
  file << "# gps_anchor_count: " << gps_count << "\n\n";
  file << "# zleveling_applied: "
       << (z_leveling_result.applied ? "true" : "false") << "\n";
  file << "# zleveling_height_prior_edge_count: "
       << z_leveling_result.height_prior_edge_count << "\n";
  file << "# zleveling_max_abs_height_after_m: "
       << z_leveling_result.max_abs_height_after_m << "\n\n";
  file << std::fixed << std::setprecision(6);
  file << "offset_x: " << offset.x() << "\n";
  file << "offset_y: " << offset.y() << "\n";
  file << "offset_z: " << offset.z() << "\n";
  file << "yaw_deg: 0.0\n";
}

}  // namespace

bool Stage1ArtifactWriter::Write(
    const Stage1Config& config,
    const std::vector<lightning::Keyframe::Ptr>& keyframes,
    const std::vector<lightning::GpsFullObservation>& gps_history,
    const std::vector<Stage1LoopConstraint>& loop_constraints,
    const Stage1LoopSummary& loop_summary,
    const Stage1ZLevelingResult& z_leveling_result,
    lightning::CloudPtr preview_map) const {
  const std::filesystem::path output_dir(config.output.directory);
  const std::filesystem::path keyframe_dir = output_dir / "keyframes";
  const std::filesystem::path cloud_dir = keyframe_dir / "clouds";
  const std::filesystem::path covariance_dir = keyframe_dir / "covariance";
  const std::filesystem::path gps_dir = output_dir / "gps";
  const std::filesystem::path diagnostics_dir = output_dir / "diagnostics";
  const std::filesystem::path preview_dir = output_dir / "preview";

  if (!EnsureDirectory(keyframe_dir) || !EnsureDirectory(gps_dir) ||
      !EnsureDirectory(diagnostics_dir)) {
    return false;
  }
  if (config.output.save_keyframe_clouds && !EnsureDirectory(cloud_dir)) {
    return false;
  }
  if (!EnsureDirectory(covariance_dir)) {
    return false;
  }
  if (config.output.save_preview_map && !EnsureDirectory(preview_dir)) {
    return false;
  }

  {
    std::ofstream manifest(output_dir / "manifest.yaml");
    if (!manifest.is_open()) {
      AERROR << "Failed to write manifest";
      return false;
    }
    manifest << "stage: stage1_pure_lio_opt\n";
    manifest << "generated_at: " << YamlQuote(CurrentIso8601Utc()) << "\n";
    manifest << "map_name: " << YamlQuote(config.map_name) << "\n";
    manifest << "vehicle_name: " << YamlQuote(config.vehicle_name) << "\n";
    manifest << "active_vehicle: " << YamlQuote(config.vehicle_name) << "\n";
    manifest << "run_config_path: " << YamlQuote(config.run_config_path)
             << "\n";
    manifest << "vehicle_config_path: " << YamlQuote(config.vehicle_config_path)
             << "\n";
    manifest << "module_root: " << YamlQuote(config.module_root) << "\n";
    manifest << "data_root: " << YamlQuote(config.data_root) << "\n";
    manifest << "debug_root: " << YamlQuote(config.debug_root) << "\n";
    manifest << "algorithm_config_path: "
             << YamlQuote(config.algorithm_config_path) << "\n";
    manifest << "dataset:\n";
    manifest << "  vehicle_name: " << YamlQuote(config.vehicle_name) << "\n";
    manifest << "  source_count: " << config.dataset_sources.size() << "\n";
    manifest << "  sources:\n";
    for (const auto& source : config.dataset_sources) {
      manifest << "    - " << YamlQuote(source) << "\n";
    }
    manifest << "  expanded_record_count: " << config.records.size() << "\n";
    manifest << "  expanded_records:\n";
    for (const auto& record : config.records) {
      manifest << "    - " << YamlQuote(record) << "\n";
    }
    manifest << "dual_lidar_enabled: "
             << (config.dual_lidar.enable ? "true" : "false") << "\n";
    manifest << "dual_lidar_primary_channel: "
             << YamlQuote(config.dual_lidar.primary_channel) << "\n";
    manifest << "dual_lidar_secondary_channel: "
             << YamlQuote(config.dual_lidar.secondary_channel) << "\n";
    manifest << "keyframe_count: " << keyframes.size() << "\n";
    manifest << "gps_full_count: " << gps_history.size() << "\n";
    manifest << "loop_closure_enabled: "
             << (loop_summary.enabled ? "true" : "false") << "\n";
    manifest << "loop_constraint_count: " << loop_constraints.size() << "\n";
    manifest << "loop_outlier_edge_count: "
             << loop_summary.loop_outlier_edge_count << "\n";
    manifest << "zleveling:\n";
    manifest << "  enabled: "
             << (z_leveling_result.enabled ? "true" : "false") << "\n";
    manifest << "  applied: "
             << (z_leveling_result.applied ? "true" : "false") << "\n";
    manifest << "  status: " << YamlQuote(z_leveling_result.status_message)
             << "\n";
    manifest << "  height_prior_applied: "
             << (z_leveling_result.height_prior_applied ? "true" : "false")
             << "\n";
    manifest << "  height_prior_status: "
             << YamlQuote(z_leveling_result.height_prior_status_message)
             << "\n";
    manifest << "  height_prior_edge_count: "
             << z_leveling_result.height_prior_edge_count << "\n";
    manifest << "  optimizer_iterations: "
             << loop_summary.zleveling_optimizer_iterations << "\n";
    manifest << "  chi2_before: " << std::fixed << std::setprecision(9)
             << loop_summary.zleveling_chi2_before << "\n";
    manifest << "  chi2_after: " << std::fixed << std::setprecision(9)
             << loop_summary.zleveling_chi2_after << "\n";
    manifest << "  height_noise_m: " << std::fixed
             << std::setprecision(9) << config.zleveling.height_noise_m
             << "\n";
    manifest << "  height_measurement_m: 0.000000000\n";
    manifest << "  max_abs_height_before_m: " << std::fixed
             << std::setprecision(9)
             << z_leveling_result.max_abs_height_before_m << "\n";
    manifest << "  max_abs_height_after_m: " << std::fixed
             << std::setprecision(9)
             << z_leveling_result.max_abs_height_after_m << "\n";
    manifest << "  lio_mean_z_before: "
             << z_leveling_result.lio_mean_z_before << "\n";
    manifest << "  lio_mean_z_after: "
             << z_leveling_result.lio_mean_z_after << "\n";
    manifest << "coordinate_frame: local_lio_enu_aligned\n";
    manifest << "relative_edges: optimized_adjacent_pose_deltas\n";
    manifest
        << "relative_edges_lio_raw: keyframes/relative_edges_lio_raw.csv\n";
    manifest << "utm_alignment_path: utm_alignment.txt\n";
    manifest << "output_format_version: 3\n";
  }

  WriteUtmAlignment(output_dir / "utm_alignment.txt", keyframes,
                    z_leveling_result);

  {
    std::ofstream poses_compat(keyframe_dir / "poses_lio.tum");
    std::ofstream poses_raw(keyframe_dir / "poses_lio_raw.tum");
    std::ofstream poses_opt(keyframe_dir / "poses_lio_opt.tum");
    if (!poses_compat.is_open() || !poses_raw.is_open() ||
        !poses_opt.is_open()) {
      AERROR << "Failed to write stage1 pose TUM outputs";
      return false;
    }
    poses_compat << lightning::DebugUtils::GenerateTUMHeader(
        "air_mapping stage1 optimized LIO poses");
    poses_raw << lightning::DebugUtils::GenerateTUMHeader(
        "air_mapping stage1 raw LIO poses");
    poses_opt << lightning::DebugUtils::GenerateTUMHeader(
        "air_mapping stage1 optimized LIO poses");

    std::ofstream keyframe_csv(keyframe_dir / "keyframes.csv");
    if (!keyframe_csv.is_open()) {
      AERROR << "Failed to write keyframes.csv";
      return false;
    }
    keyframe_csv << "id,timestamp,x,y,z,qx,qy,qz,qw,cloud_path,covariance_path,"
                 << "has_gps,raw_x,raw_y,raw_z,raw_qx,raw_qy,raw_qz,raw_qw\n";

    std::ofstream relative_csv(keyframe_dir / "relative_edges.csv");
    if (!relative_csv.is_open()) {
      AERROR << "Failed to write relative_edges.csv";
      return false;
    }
    relative_csv << "from_id,to_id,tx,ty,tz,qx,qy,qz,qw\n";

    std::ofstream relative_raw_csv(keyframe_dir / "relative_edges_lio_raw.csv");
    if (!relative_raw_csv.is_open()) {
      AERROR << "Failed to write relative_edges_lio_raw.csv";
      return false;
    }
    relative_raw_csv << "from_id,to_id,tx,ty,tz,qx,qy,qz,qw\n";

    std::ofstream loop_csv(keyframe_dir / "loop_edges.csv");
    if (!loop_csv.is_open()) {
      AERROR << "Failed to write loop_edges.csv";
      return false;
    }
    loop_csv << "target_id,source_id,tx,ty,tz,qx,qy,qz,qw,ndt_score,"
             << "icp_fitness,used_icp,delta_from_initial_translation_m,"
             << "delta_from_initial_rotation_deg\n";

    for (size_t index = 0; index < keyframes.size(); ++index) {
      const auto& keyframe = keyframes[index];
      if (!keyframe) {
        continue;
      }
      const std::string cloud_name = FormatKeyframeCloudName(keyframe->GetID());
      const std::string cov_name =
          FormatKeyframeDataName(keyframe->GetID(), ".txt");
      const std::filesystem::path cov_path = covariance_dir / cov_name;
      const lightning::SE3 raw_pose = keyframe->GetLIOPose();
      const lightning::SE3 opt_pose = keyframe->GetOptPose();
      const auto gps_data = keyframe->GetGpsData();

      poses_compat << lightning::DebugUtils::SE3ToTUMString(
                          keyframe->GetTimestamp(), opt_pose)
                   << "\n";
      poses_raw << lightning::DebugUtils::SE3ToTUMString(
                       keyframe->GetTimestamp(), raw_pose)
                << "\n";
      poses_opt << lightning::DebugUtils::SE3ToTUMString(
                       keyframe->GetTimestamp(), opt_pose)
                << "\n";

      keyframe_csv << keyframe->GetID() << "," << std::fixed
                   << std::setprecision(9) << keyframe->GetTimestamp() << ",";
      WritePoseCsv(keyframe_csv, opt_pose);
      keyframe_csv << ",keyframes/clouds/" << cloud_name
                   << ",keyframes/covariance/" << cov_name << ","
                   << (gps_data.has_gps ? 1 : 0) << ",";
      WritePoseCsv(keyframe_csv, raw_pose);
      keyframe_csv << "\n";

      WriteMatrix6(cov_path, keyframe->GetCovariance());

      if (config.output.save_keyframe_clouds && keyframe->GetCloud()) {
        const std::filesystem::path cloud_path = cloud_dir / cloud_name;
        if (pcl::io::savePCDFileBinaryCompressed(cloud_path.string(),
                                                 *keyframe->GetCloud()) < 0) {
          AERROR << "Failed to save keyframe cloud: " << cloud_path.string();
          return false;
        }
      }

      if (index > 0) {
        const auto& previous = keyframes[index - 1];
        if (previous) {
          relative_csv << previous->GetID() << "," << keyframe->GetID() << ",";
          WritePoseCsv(relative_csv, previous->GetOptPose().inverse() *
                                         keyframe->GetOptPose());
          relative_csv << "\n";

          relative_raw_csv << previous->GetID() << "," << keyframe->GetID()
                           << ",";
          WritePoseCsv(relative_raw_csv, keyframe->GetRelativeMotion());
          relative_raw_csv << "\n";
        }
      }
    }

    for (const auto& loop : loop_constraints) {
      loop_csv << loop.target_id << "," << loop.source_id << ",";
      WritePoseCsv(loop_csv, loop.measurement_target_to_source);
      loop_csv << "," << std::fixed << std::setprecision(9) << loop.ndt_score
               << "," << loop.icp_fitness << "," << (loop.used_icp ? 1 : 0)
               << "," << loop.delta_from_initial_translation_m << ","
               << loop.delta_from_initial_rotation_deg << "\n";
    }
  }

  {
    std::ofstream summary(diagnostics_dir / "loop_summary.csv");
    if (!summary.is_open()) {
      AERROR << "Failed to write loop_summary.csv";
      return false;
    }
    summary << "enabled,keyframe_count,query_count,coarse_candidate_count,"
            << "accepted_loop_count,lio_edge_count,loop_edge_count,"
            << "loop_outlier_edge_count,optimizer_iterations,chi2_before,"
            << "chi2_after\n";
    summary << (loop_summary.enabled ? 1 : 0) << ","
            << loop_summary.keyframe_count << "," << loop_summary.query_count
            << "," << loop_summary.coarse_candidate_count << ","
            << loop_summary.accepted_loop_count << ","
            << loop_summary.lio_edge_count << ","
            << loop_summary.loop_edge_count << ","
            << loop_summary.loop_outlier_edge_count << ","
            << loop_summary.optimizer_iterations << "," << std::fixed
            << std::setprecision(9) << loop_summary.chi2_before << ","
            << loop_summary.chi2_after << "\n";
  }

  {
    std::ofstream summary(diagnostics_dir / "zleveling_summary.csv");
    if (!summary.is_open()) {
      AERROR << "Failed to write zleveling_summary.csv";
      return false;
    }
    summary << "enabled,applied,status_message,height_prior_applied,"
            << "height_prior_status,height_prior_edge_count,"
            << "optimizer_iterations,chi2_before,chi2_after,"
            << "height_noise_m,height_measurement_m,"
            << "max_abs_height_before_m,max_abs_height_after_m,"
            << "lio_mean_z_before,"
            << "lio_mean_z_after\n";
    summary << (z_leveling_result.enabled ? 1 : 0) << ","
            << (z_leveling_result.applied ? 1 : 0) << ","
            << z_leveling_result.status_message << ","
            << (z_leveling_result.height_prior_applied ? 1 : 0) << ","
            << z_leveling_result.height_prior_status_message << ","
            << z_leveling_result.height_prior_edge_count << ","
            << loop_summary.zleveling_optimizer_iterations << ","
            << std::fixed << std::setprecision(9)
            << loop_summary.zleveling_chi2_before << ","
            << loop_summary.zleveling_chi2_after << ","
            << std::setprecision(9) << config.zleveling.height_noise_m << ","
            << 0.0 << "," << z_leveling_result.max_abs_height_before_m << ","
            << z_leveling_result.max_abs_height_after_m << ","
            << z_leveling_result.lio_mean_z_before << ","
            << z_leveling_result.lio_mean_z_after << "\n";

    std::ofstream samples(diagnostics_dir / "zleveling_samples.csv");
    if (!samples.is_open()) {
      AERROR << "Failed to write zleveling_samples.csv";
      return false;
    }
    samples << "keyframe_id,timestamp,selected,reject_reason,"
            << "z_before,z_after\n";
    for (const auto& sample : z_leveling_result.samples) {
      samples << sample.keyframe_id << "," << std::fixed
              << std::setprecision(9) << sample.timestamp << ","
              << (sample.selected ? 1 : 0) << "," << sample.reject_reason
              << "," << sample.lio_z_before << ","
              << sample.lio_z_after << "\n";
    }
  }

  {
    std::ofstream keyframe_diag(diagnostics_dir / "keyframe_diagnostics.csv");
    if (!keyframe_diag.is_open()) {
      AERROR << "Failed to write keyframe_diagnostics.csv";
      return false;
    }
    keyframe_diag
        << "id,timestamp,cloud_point_count,has_gps,lio_raw_x,lio_raw_y,"
        << "lio_raw_z,lio_raw_qx,lio_raw_qy,lio_raw_qz,lio_raw_qw,"
        << "lio_opt_x,lio_opt_y,lio_opt_z,lio_opt_qx,lio_opt_qy,"
        << "lio_opt_qz,lio_opt_qw,delta_x,delta_y,delta_z,"
        << "delta_rotation_deg,raw_roll_rad,raw_pitch_rad,raw_yaw_rad,"
        << "opt_roll_rad,opt_pitch_rad,opt_yaw_rad,cov_xx,cov_yy,cov_zz,"
        << "cov_rollroll,cov_pitchpitch,cov_yawyaw,"
        << "relative_tx,relative_ty,relative_tz,relative_qx,relative_qy,"
        << "relative_qz,relative_qw\n";
    for (const auto& keyframe : keyframes) {
      if (!keyframe) {
        continue;
      }
      const lightning::SE3 raw_pose = keyframe->GetLIOPose();
      const lightning::SE3 opt_pose = keyframe->GetOptPose();
      const lightning::SE3 pose_delta = raw_pose.inverse() * opt_pose;
      const lightning::SE3 relative_motion = keyframe->GetRelativeMotion();
      const lightning::Mat6d covariance = keyframe->GetCovariance();
      const size_t cloud_point_count =
          keyframe->GetCloud() ? keyframe->GetCloud()->size() : 0;
      keyframe_diag << keyframe->GetID() << "," << std::fixed
                    << std::setprecision(9) << keyframe->GetTimestamp() << ","
                    << cloud_point_count << ","
                    << (keyframe->GetGpsData().has_gps ? 1 : 0) << ",";
      WritePoseCsv(keyframe_diag, raw_pose);
      keyframe_diag << ",";
      WritePoseCsv(keyframe_diag, opt_pose);
      keyframe_diag << ","
                    << opt_pose.translation().x() - raw_pose.translation().x()
                    << ","
                    << opt_pose.translation().y() - raw_pose.translation().y()
                    << ","
                    << opt_pose.translation().z() - raw_pose.translation().z()
                    << "," << pose_delta.so3().log().norm() * kRadToDeg << ","
                    << PoseRollRad(raw_pose) << "," << PosePitchRad(raw_pose)
                    << "," << PoseYawRad(raw_pose) << ","
                    << PoseRollRad(opt_pose) << "," << PosePitchRad(opt_pose)
                    << "," << PoseYawRad(opt_pose) << "," << covariance(0, 0)
                    << "," << covariance(1, 1) << "," << covariance(2, 2) << ","
                    << covariance(3, 3) << "," << covariance(4, 4) << ","
                    << covariance(5, 5) << ",";
      WritePoseCsv(keyframe_diag, relative_motion);
      keyframe_diag << "\n";
    }
  }

  {
    lightning::Vec3d configured_lever_arm = lightning::Vec3d::Zero();
    if (!LoadGpsLeverArm(config.algorithm_config_path, &configured_lever_arm)) {
      return false;
    }

    std::ofstream gps_full(gps_dir / "gps_full.csv");
    if (!gps_full.is_open()) {
      AERROR << "Failed to write gps_full.csv";
      return false;
    }
    gps_full << "timestamp,antenna_x,antenna_y,antenna_z,imu_x,imu_y,imu_z,"
             << "heading_rad,pitch_rad,std_x,std_y,std_z,sol_status,sol_type,"
             << "heading_std_deg,pitch_std_deg,satellite_tracked\n";
    for (const auto& gps : gps_history) {
      gps_full << std::fixed << std::setprecision(9) << gps.timestamp << ","
               << gps.antenna_utm.x() << "," << gps.antenna_utm.y() << ","
               << gps.antenna_utm.z() << "," << gps.imu_utm.x() << ","
               << gps.imu_utm.y() << "," << gps.imu_utm.z() << ","
               << gps.heading_rad << "," << gps.pitch_rad << ","
               << gps.position_std_dev.x() << "," << gps.position_std_dev.y()
               << "," << gps.position_std_dev.z() << "," << gps.sol_status
               << "," << gps.sol_type << "," << gps.heading_std_dev << ","
               << gps.pitch_std_dev << "," << gps.satellite_tracked << "\n";
    }

    std::ofstream gps_assoc(gps_dir / "gps_keyframe_assoc.csv");
    if (!gps_assoc.is_open()) {
      AERROR << "Failed to write gps_keyframe_assoc.csv";
      return false;
    }
    gps_assoc
        << "keyframe_id,timestamp,has_gps,utm_x,utm_y,utm_z,std_x,std_y,std_z,"
        << "heading_deg,heading_std_deg,sol_type\n";
    for (const auto& keyframe : keyframes) {
      if (!keyframe) {
        continue;
      }
      const auto gps = keyframe->GetGpsData();
      gps_assoc << keyframe->GetID() << "," << std::fixed
                << std::setprecision(9) << keyframe->GetTimestamp() << ","
                << (gps.has_gps ? 1 : 0) << "," << gps.gps_utm_position.x()
                << "," << gps.gps_utm_position.y() << ","
                << gps.gps_utm_position.z() << "," << gps.gps_std_dev.x() << ","
                << gps.gps_std_dev.y() << "," << gps.gps_std_dev.z() << ","
                << gps.gps_heading_deg << "," << gps.heading_std_deg << ","
                << gps.sol_type << "\n";
    }

    std::ofstream gps_raw_assoc(gps_dir / "gps_keyframe_raw_assoc.csv");
    if (!gps_raw_assoc.is_open()) {
      AERROR << "Failed to write gps_keyframe_raw_assoc.csv";
      return false;
    }
    gps_raw_assoc << "keyframe_id,timestamp,has_stage1_gps,has_raw_gps,"
                  << "interp_before_time,interp_after_time,interp_alpha,"
                  << "interp_gap_before_s,interp_gap_after_s,"
                  << "antenna_utm_x,antenna_utm_y,antenna_utm_z,"
                  << "stage1_imu_utm_x,stage1_imu_utm_y,stage1_imu_utm_z,"
                  << "gps_heading_imu_utm_x,gps_heading_imu_utm_y,"
                  << "gps_heading_imu_utm_z,lio_raw_yaw_imu_utm_x,"
                  << "lio_raw_yaw_imu_utm_y,lio_raw_yaw_imu_utm_z,"
                  << "lio_opt_yaw_imu_utm_x,lio_opt_yaw_imu_utm_y,"
                  << "lio_opt_yaw_imu_utm_z,"
                  << "lever_arm_config_x,lever_arm_config_y,lever_arm_config_z,"
                  << "lever_arm_gps_heading_utm_x,lever_arm_gps_heading_utm_y,"
                  << "lever_arm_gps_heading_utm_z,lever_arm_lio_raw_yaw_utm_x,"
                  << "lever_arm_lio_raw_yaw_utm_y,lever_arm_lio_raw_yaw_utm_z,"
                  << "lever_arm_lio_opt_yaw_utm_x,lever_arm_lio_opt_yaw_utm_y,"
                  << "lever_arm_lio_opt_yaw_utm_z,"
                  << "lio_raw_x,lio_raw_y,lio_raw_z,lio_raw_qx,lio_raw_qy,"
                  << "lio_raw_qz,lio_raw_qw,lio_opt_x,lio_opt_y,lio_opt_z,"
                  << "lio_opt_qx,lio_opt_qy,lio_opt_qz,lio_opt_qw,"
                  << "lio_raw_roll_rad,lio_raw_pitch_rad,lio_raw_yaw_rad,"
                  << "lio_opt_roll_rad,lio_opt_pitch_rad,lio_opt_yaw_rad,"
                  << "gnss_heading_rad,gnss_pitch_rad,heading_std_deg,"
                  << "pitch_std_deg,std_x,std_y,std_z,sol_status,sol_type,"
                  << "satellite_tracked,stage1_minus_lio_raw_yaw_dx,"
                  << "stage1_minus_lio_raw_yaw_dy,stage1_minus_lio_raw_yaw_dz,"
                  << "stage1_minus_gps_heading_dx,stage1_minus_gps_heading_dy,"
                  << "stage1_minus_gps_heading_dz\n";
    for (const auto& keyframe : keyframes) {
      if (!keyframe) {
        continue;
      }
      const double timestamp = keyframe->GetTimestamp();
      const auto stage1_gps = keyframe->GetGpsData();
      const lightning::SE3 raw_pose = keyframe->GetLIOPose();
      const lightning::SE3 opt_pose = keyframe->GetOptPose();
      const double raw_yaw = PoseYawRad(raw_pose);
      const double opt_yaw = PoseYawRad(opt_pose);

      InterpolatedGpsFullObservation interp;
      const bool has_raw_gps =
          InterpolateGpsHistoryAt(gps_history, timestamp, &interp);
      lightning::Vec3d antenna = lightning::Vec3d::Zero();
      lightning::Vec3d gps_heading_imu = lightning::Vec3d::Zero();
      lightning::Vec3d lio_raw_yaw_imu = lightning::Vec3d::Zero();
      lightning::Vec3d lio_opt_yaw_imu = lightning::Vec3d::Zero();
      lightning::Vec3d lever_gps_heading = lightning::Vec3d::Zero();
      lightning::Vec3d lever_lio_raw_yaw = lightning::Vec3d::Zero();
      lightning::Vec3d lever_lio_opt_yaw = lightning::Vec3d::Zero();
      lightning::Vec3d stage1_minus_lio_raw = lightning::Vec3d::Zero();
      lightning::Vec3d stage1_minus_gps_heading = lightning::Vec3d::Zero();

      if (has_raw_gps) {
        antenna = interp.observation.antenna_utm;
        lever_gps_heading =
            lightning::SO3::rotZ(interp.observation.heading_rad) *
            configured_lever_arm;
        lever_lio_raw_yaw =
            lightning::SO3::rotZ(raw_yaw) * configured_lever_arm;
        lever_lio_opt_yaw =
            lightning::SO3::rotZ(opt_yaw) * configured_lever_arm;
        gps_heading_imu = antenna - lever_gps_heading;
        lio_raw_yaw_imu = antenna - lever_lio_raw_yaw;
        lio_opt_yaw_imu = antenna - lever_lio_opt_yaw;
        if (stage1_gps.has_gps && stage1_gps.gps_utm_position.allFinite()) {
          stage1_minus_lio_raw = stage1_gps.gps_utm_position - lio_raw_yaw_imu;
          stage1_minus_gps_heading =
              stage1_gps.gps_utm_position - gps_heading_imu;
        }
      }

      gps_raw_assoc << keyframe->GetID() << "," << std::fixed
                    << std::setprecision(9) << timestamp << ","
                    << (stage1_gps.has_gps ? 1 : 0) << ","
                    << (has_raw_gps ? 1 : 0) << "," << interp.before_timestamp
                    << "," << interp.after_timestamp << "," << interp.alpha
                    << "," << interp.gap_before_s << "," << interp.gap_after_s
                    << "," << antenna.x() << "," << antenna.y() << ","
                    << antenna.z() << "," << stage1_gps.gps_utm_position.x()
                    << "," << stage1_gps.gps_utm_position.y() << ","
                    << stage1_gps.gps_utm_position.z() << ","
                    << gps_heading_imu.x() << "," << gps_heading_imu.y() << ","
                    << gps_heading_imu.z() << "," << lio_raw_yaw_imu.x() << ","
                    << lio_raw_yaw_imu.y() << "," << lio_raw_yaw_imu.z() << ","
                    << lio_opt_yaw_imu.x() << "," << lio_opt_yaw_imu.y() << ","
                    << lio_opt_yaw_imu.z() << "," << configured_lever_arm.x()
                    << "," << configured_lever_arm.y() << ","
                    << configured_lever_arm.z() << "," << lever_gps_heading.x()
                    << "," << lever_gps_heading.y() << ","
                    << lever_gps_heading.z() << "," << lever_lio_raw_yaw.x()
                    << "," << lever_lio_raw_yaw.y() << ","
                    << lever_lio_raw_yaw.z() << "," << lever_lio_opt_yaw.x()
                    << "," << lever_lio_opt_yaw.y() << ","
                    << lever_lio_opt_yaw.z() << ",";
      WritePoseCsv(gps_raw_assoc, raw_pose);
      gps_raw_assoc << ",";
      WritePoseCsv(gps_raw_assoc, opt_pose);
      gps_raw_assoc << "," << PoseRollRad(raw_pose) << ","
                    << PosePitchRad(raw_pose) << "," << raw_yaw << ","
                    << PoseRollRad(opt_pose) << "," << PosePitchRad(opt_pose)
                    << "," << opt_yaw << "," << interp.observation.heading_rad
                    << "," << interp.observation.pitch_rad << ","
                    << interp.observation.heading_std_dev << ","
                    << interp.observation.pitch_std_dev << ","
                    << interp.observation.position_std_dev.x() << ","
                    << interp.observation.position_std_dev.y() << ","
                    << interp.observation.position_std_dev.z() << ","
                    << interp.observation.sol_status << ","
                    << interp.observation.sol_type << ","
                    << interp.observation.satellite_tracked << ","
                    << stage1_minus_lio_raw.x() << ","
                    << stage1_minus_lio_raw.y() << ","
                    << stage1_minus_lio_raw.z() << ","
                    << stage1_minus_gps_heading.x() << ","
                    << stage1_minus_gps_heading.y() << ","
                    << stage1_minus_gps_heading.z() << "\n";
    }
  }

  if (config.output.save_preview_map && preview_map && !preview_map->empty()) {
    const std::filesystem::path preview_path =
        preview_dir / "lio_global_preview.pcd";
    if (pcl::io::savePCDFileBinaryCompressed(preview_path.string(),
                                             *preview_map) < 0) {
      AERROR << "Failed to save preview map: " << preview_path.string();
      return false;
    }
    const std::filesystem::path opt_preview_path =
        preview_dir / "lio_opt_global_preview.pcd";
    if (pcl::io::savePCDFileBinaryCompressed(opt_preview_path.string(),
                                             *preview_map) < 0) {
      AERROR << "Failed to save optimized preview map: "
             << opt_preview_path.string();
      return false;
    }
  }

  AINFO << "Stage1 artifacts written to " << output_dir.string();
  return true;
}

}  // namespace stage1
}  // namespace air_mapping
}  // namespace apollo
