#include "modules/air_mapping/stage2/stage1_artifact_reader.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <system_error>
#include <unordered_map>
#include <vector>

#include "pcl/io/pcd_io.h"
#include "yaml-cpp/yaml.h"

#include "cyber/common/log.h"
#include "modules/air_mapping/system/common/artifact_utils.h"

namespace apollo {
namespace air_mapping {
namespace stage2 {

namespace {

std::vector<std::string> SplitCsvLine(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

std::filesystem::path ResolveArtifactPath(const std::filesystem::path& root,
                                          const std::string& artifact_path) {
  std::filesystem::path path(artifact_path);
  if (path.is_absolute()) {
    return path;
  }
  return root / path;
}

bool ReadMatrix6(const std::filesystem::path& path, lightning::Mat6d* matrix) {
  if (matrix == nullptr) {
    return false;
  }
  std::ifstream file(path);
  if (!file.is_open()) {
    AERROR << "Failed to open covariance file: " << path.string();
    return false;
  }

  lightning::Mat6d result = lightning::Mat6d::Zero();
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      if (!(file >> result(row, col))) {
        AERROR << "Invalid covariance file: " << path.string();
        return false;
      }
    }
  }
  *matrix = result;
  return true;
}

bool ParsePoseCsvFields(const std::vector<std::string>& fields, size_t offset,
                        lightning::SE3* pose) {
  if (pose == nullptr || fields.size() < offset + 7) {
    return false;
  }
  const lightning::Vec3d translation(std::stod(fields[offset]),
                                     std::stod(fields[offset + 1]),
                                     std::stod(fields[offset + 2]));
  Eigen::Quaterniond quaternion(
      std::stod(fields[offset + 6]), std::stod(fields[offset + 3]),
      std::stod(fields[offset + 4]), std::stod(fields[offset + 5]));
  if (quaternion.norm() < 1e-12 || !translation.allFinite()) {
    return false;
  }
  quaternion.normalize();
  *pose = lightning::SE3(lightning::SO3(quaternion), translation);
  return true;
}

lightning::Vec3d ParseVec3CsvFields(const std::vector<std::string>& fields,
                                    size_t offset) {
  return lightning::Vec3d(std::stod(fields[offset]),
                          std::stod(fields[offset + 1]),
                          std::stod(fields[offset + 2]));
}

std::string Trim(const std::string& value) {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}

std::string UnquoteYamlScalar(std::string value) {
  value = Trim(value);
  if (value.size() >= 2 &&
      ((value.front() == '"' && value.back() == '"') ||
       (value.front() == '\'' && value.back() == '\''))) {
    value = value.substr(1, value.size() - 2);
  }
  std::string result;
  result.reserve(value.size());
  bool escaped = false;
  for (const char c : value) {
    if (escaped) {
      result.push_back(c);
      escaped = false;
      continue;
    }
    if (c == '\\') {
      escaped = true;
      continue;
    }
    result.push_back(c);
  }
  if (escaped) {
    result.push_back('\\');
  }
  return result;
}

bool ReadManifest(const Stage2Config& config, Stage1Dataset* dataset) {
  const std::filesystem::path manifest_path =
      std::filesystem::path(config.input_dir) / "manifest.yaml";
  dataset->stage1_manifest_path = manifest_path.string();
  std::ifstream file(manifest_path);
  if (!file.is_open()) {
    AWARN << "Stage1 manifest not found, continue without provenance: "
          << manifest_path.string();
    return true;
  }

  std::string section;
  std::string list_key;
  bool dataset_seen = false;
  std::string line;
  while (std::getline(file, line)) {
    const std::string trimmed = Trim(line);
    if (trimmed.empty() || trimmed[0] == '#') {
      continue;
    }
    if (trimmed == "dataset:") {
      section = "dataset";
      list_key.clear();
      dataset_seen = true;
      continue;
    }
    if (trimmed == "records:") {
      section.clear();
      list_key = "records";
      continue;
    }
    if (trimmed.rfind("- ", 0) == 0) {
      const std::string value = UnquoteYamlScalar(trimmed.substr(2));
      if (section == "dataset" && list_key == "sources") {
        dataset->dataset_sources.push_back(value);
      } else if (section == "dataset" && list_key == "expanded_records") {
        dataset->expanded_records.push_back(value);
      } else if (list_key == "records" && !dataset_seen) {
        dataset->expanded_records.push_back(value);
      }
      continue;
    }

    const auto separator = trimmed.find(':');
    if (separator == std::string::npos) {
      continue;
    }
    const std::string key = Trim(trimmed.substr(0, separator));
    const std::string value =
        UnquoteYamlScalar(trimmed.substr(separator + 1));
    if (section == "dataset") {
      if (key == "sources" || key == "expanded_records") {
        list_key = key;
      } else {
        list_key.clear();
      }
      continue;
    }
    if (dataset_seen) {
      continue;
    }
    list_key.clear();
    if (key == "generated_at") {
      dataset->stage1_generated_at = value;
    } else if (key == "vehicle_name") {
      dataset->stage1_vehicle_name = value;
    }
  }

  if (!dataset->stage1_vehicle_name.empty() &&
      dataset->stage1_vehicle_name != config.vehicle_name) {
    AERROR << "Stage1 artifact vehicle mismatch: expected "
           << config.vehicle_name << ", manifest has "
           << dataset->stage1_vehicle_name << ". input="
           << config.input_dir;
    return false;
  }
  if (dataset->dataset_sources.empty()) {
    dataset->dataset_sources = dataset->expanded_records;
  }
  return true;
}

bool LoadLidarExtrinsic(const std::string& config_path,
                        lightning::SO3* rotation,
                        lightning::Vec3d* translation) {
  if (rotation == nullptr || translation == nullptr) {
    return false;
  }
  *rotation = lightning::SO3();
  *translation = lightning::Vec3d::Zero();

  std::error_code error;
  if (!std::filesystem::exists(config_path, error)) {
    AWARN << "Source config not found, use identity lidar extrinsic: "
          << config_path;
    return true;
  }

  try {
    YAML::Node yaml = YAML::LoadFile(config_path);
    if (!yaml["fasterlio"]) {
      AWARN << "No fasterlio section in source config, use identity lidar "
               "extrinsic: "
            << config_path;
      return true;
    }
    const auto& fasterlio = yaml["fasterlio"];
    if (fasterlio["extrinsic_R"]) {
      const auto values = fasterlio["extrinsic_R"].as<std::vector<double>>();
      if (values.size() == 9) {
        lightning::Mat3d matrix;
        matrix << values[0], values[1], values[2], values[3], values[4],
            values[5], values[6], values[7], values[8];
        *rotation = lightning::SO3::fitToSO3(matrix);
      }
    }
    if (fasterlio["extrinsic_T"]) {
      const auto values = fasterlio["extrinsic_T"].as<std::vector<double>>();
      if (values.size() == 3) {
        *translation = lightning::Vec3d(values[0], values[1], values[2]);
      }
    }
  } catch (const std::exception& e) {
    AERROR << "Failed to load lidar extrinsic from " << config_path
           << ", error: " << e.what();
    return false;
  }

  AINFO << "[Stage2Reader] Loaded lidar extrinsic from " << config_path
        << ", t=" << translation->transpose();
  return true;
}

bool ReadKeyframes(const Stage2Config& config, bool load_keyframe_clouds,
                   Stage1Dataset* dataset,
                   std::unordered_map<unsigned long, size_t>* id_to_index) {
  const std::filesystem::path input_dir(config.input_dir);
  const std::filesystem::path keyframes_path =
      input_dir / "keyframes" / "keyframes.csv";
  std::ifstream file(keyframes_path);
  if (!file.is_open()) {
    AERROR << "Failed to open keyframes.csv: " << keyframes_path.string();
    return false;
  }

  lightning::SO3 lidar_rotation;
  lightning::Vec3d lidar_translation;
  if (!LoadLidarExtrinsic(config.source_config_path, &lidar_rotation,
                          &lidar_translation)) {
    return false;
  }

  std::string line;
  std::getline(file, line);
  size_t row = 1;
  while (std::getline(file, line)) {
    ++row;
    if (line.empty()) {
      continue;
    }
    const auto fields = SplitCsvLine(line);
    if (fields.size() < 12) {
      AERROR << "Invalid keyframes.csv row " << row << ": " << line;
      return false;
    }

    const unsigned long id = std::stoul(fields[0]);
    const double timestamp = std::stod(fields[1]);
    lightning::SE3 lio_pose;
    if (!ParsePoseCsvFields(fields, 2, &lio_pose)) {
      AERROR << "Invalid pose in keyframes.csv row " << row << ": " << line;
      return false;
    }

    lightning::CloudPtr cloud(new lightning::PointCloudType);
    if (load_keyframe_clouds && !fields[9].empty()) {
      const auto cloud_path = ResolveArtifactPath(input_dir, fields[9]);
      if (pcl::io::loadPCDFile<lightning::PointType>(cloud_path.string(),
                                                     *cloud) < 0) {
        AERROR << "Failed to load keyframe cloud: " << cloud_path.string();
        return false;
      }
    }

    lightning::NavState state;
    state.timestamp_ = timestamp;
    state.SetPose(lio_pose);
    state.offset_R_lidar_ = lidar_rotation;
    state.offset_t_lidar_ = lidar_translation;

    auto keyframe = std::make_shared<lightning::Keyframe>(id, cloud, state);
    keyframe->SetLIOPose(lio_pose);
    keyframe->SetOptPose(lio_pose);

    lightning::Mat6d covariance = lightning::Mat6d::Identity();
    if (!fields[10].empty()) {
      const auto covariance_path = ResolveArtifactPath(input_dir, fields[10]);
      if (!ReadMatrix6(covariance_path, &covariance)) {
        return false;
      }
    }
    keyframe->SetCovariance(covariance);

    (*id_to_index)[id] = dataset->keyframes.size();
    dataset->keyframes.push_back(keyframe);
    dataset->keyframe_cloud_paths.push_back(fields[9]);
    dataset->keyframe_covariance_paths.push_back(fields[10]);
  }

  AINFO << "[Stage2Reader] Loaded keyframes: " << dataset->keyframes.size()
        << ", clouds=" << (load_keyframe_clouds ? "loaded" : "skipped");
  return !dataset->keyframes.empty();
}

bool ReadRelativeEdges(
    const Stage2Config& config, Stage1Dataset* dataset,
    const std::unordered_map<unsigned long, size_t>& id_to_index) {
  const std::filesystem::path relative_path =
      std::filesystem::path(config.input_dir) / "keyframes" /
      "relative_edges.csv";
  std::ifstream file(relative_path);
  if (!file.is_open()) {
    AERROR << "Failed to open relative_edges.csv: " << relative_path.string();
    return false;
  }

  std::string line;
  std::getline(file, line);
  size_t edge_count = 0;
  size_t row = 1;
  while (std::getline(file, line)) {
    ++row;
    if (line.empty()) {
      continue;
    }
    const auto fields = SplitCsvLine(line);
    if (fields.size() < 9) {
      AERROR << "Invalid relative_edges.csv row " << row << ": " << line;
      return false;
    }

    const unsigned long to_id = std::stoul(fields[1]);
    auto it = id_to_index.find(to_id);
    if (it == id_to_index.end()) {
      AWARN << "Relative edge references missing to_id=" << to_id;
      continue;
    }
    lightning::SE3 relative_motion;
    if (!ParsePoseCsvFields(fields, 2, &relative_motion)) {
      AERROR << "Invalid relative edge pose at row " << row << ": " << line;
      return false;
    }
    dataset->keyframes[it->second]->SetRelativeMotion(relative_motion);
    ++edge_count;
  }

  AINFO << "[Stage2Reader] Loaded relative edges: " << edge_count;
  return true;
}

bool ReadGpsKeyframeAssociations(
    const Stage2Config& config, Stage1Dataset* dataset,
    const std::unordered_map<unsigned long, size_t>& id_to_index) {
  const std::filesystem::path gps_assoc_path =
      std::filesystem::path(config.input_dir) / "gps" /
      "gps_keyframe_assoc.csv";
  std::ifstream file(gps_assoc_path);
  if (!file.is_open()) {
    AERROR << "Failed to open gps_keyframe_assoc.csv: "
           << gps_assoc_path.string();
    return false;
  }

  std::string line;
  std::getline(file, line);
  size_t valid_count = 0;
  size_t row = 1;
  while (std::getline(file, line)) {
    ++row;
    if (line.empty()) {
      continue;
    }
    const auto fields = SplitCsvLine(line);
    if (fields.size() < 12) {
      AERROR << "Invalid gps_keyframe_assoc.csv row " << row << ": " << line;
      return false;
    }

    Stage1GpsKeyframeObservation observation;
    observation.keyframe_id = std::stoul(fields[0]);
    observation.timestamp = std::stod(fields[1]);
    observation.has_gps = std::stoi(fields[2]) != 0;
    observation.utm_position = lightning::Vec3d(
        std::stod(fields[3]), std::stod(fields[4]), std::stod(fields[5]));
    observation.std_dev = lightning::Vec3d(
        std::stod(fields[6]), std::stod(fields[7]), std::stod(fields[8]));
    observation.heading_deg = std::stod(fields[9]);
    observation.heading_std_deg = std::stod(fields[10]);
    observation.sol_type = static_cast<uint32_t>(std::stoul(fields[11]));
    dataset->gps_keyframe_observations.push_back(observation);

    if (!observation.has_gps || !observation.utm_position.allFinite()) {
      continue;
    }
    auto it = id_to_index.find(observation.keyframe_id);
    if (it == id_to_index.end()) {
      AWARN << "GPS association references missing keyframe_id="
            << observation.keyframe_id;
      continue;
    }

    lightning::Keyframe::GpsData gps_data;
    gps_data.gps_available = true;
    gps_data.has_gps = true;
    gps_data.gps_utm_position = observation.utm_position;
    gps_data.gps_std_dev = observation.std_dev;
    gps_data.gps_heading_deg = observation.heading_deg;
    gps_data.heading_std_deg = observation.heading_std_deg;
    gps_data.sol_type = observation.sol_type;
    dataset->keyframes[it->second]->SetGpsData(gps_data);
    ++valid_count;
  }

  AINFO << "[Stage2Reader] Loaded GPS keyframe associations: total="
        << dataset->gps_keyframe_observations.size()
        << ", valid=" << valid_count;
  return true;
}

bool ReadGpsFullHistory(const Stage2Config& config, Stage1Dataset* dataset) {
  const std::filesystem::path gps_full_path =
      std::filesystem::path(config.input_dir) / "gps" / "gps_full.csv";
  std::ifstream file(gps_full_path);
  if (!file.is_open()) {
    AWARN << "gps_full.csv not found, continue without raw GPS history: "
          << gps_full_path.string();
    return true;
  }

  std::string line;
  std::getline(file, line);
  size_t row = 1;
  while (std::getline(file, line)) {
    ++row;
    if (line.empty()) {
      continue;
    }
    const auto fields = SplitCsvLine(line);
    if (fields.size() < 17) {
      AERROR << "Invalid gps_full.csv row " << row << ": " << line;
      return false;
    }
    lightning::GpsFullObservation observation;
    observation.timestamp = std::stod(fields[0]);
    observation.antenna_utm = lightning::Vec3d(
        std::stod(fields[1]), std::stod(fields[2]), std::stod(fields[3]));
    observation.imu_utm = lightning::Vec3d(
        std::stod(fields[4]), std::stod(fields[5]), std::stod(fields[6]));
    observation.heading_rad = std::stod(fields[7]);
    observation.pitch_rad = std::stod(fields[8]);
    observation.position_std_dev = lightning::Vec3d(
        std::stod(fields[9]), std::stod(fields[10]), std::stod(fields[11]));
    observation.sol_status = static_cast<uint32_t>(std::stoul(fields[12]));
    observation.sol_type = static_cast<uint32_t>(std::stoul(fields[13]));
    observation.heading_std_dev = std::stod(fields[14]);
    observation.pitch_std_dev = std::stod(fields[15]);
    observation.satellite_tracked =
        static_cast<uint32_t>(std::stoul(fields[16]));
    observation.has_position = true;
    observation.has_heading = true;
    observation.is_valid = observation.imu_utm.allFinite();
    dataset->gps_full_history.push_back(observation);
  }

  AINFO << "[Stage2Reader] Loaded raw GPS history: "
        << dataset->gps_full_history.size();
  return true;
}

bool ReadGpsRawKeyframeAssociations(const Stage2Config& config,
                                    Stage1Dataset* dataset) {
  const std::filesystem::path gps_raw_assoc_path =
      std::filesystem::path(config.input_dir) / "gps" /
      "gps_keyframe_raw_assoc.csv";
  std::ifstream file(gps_raw_assoc_path);
  if (!file.is_open()) {
    AWARN << "gps_keyframe_raw_assoc.csv not found, lever-arm calibration will "
             "be skipped: "
          << gps_raw_assoc_path.string();
    return true;
  }

  std::string line;
  std::getline(file, line);
  size_t row = 1;
  while (std::getline(file, line)) {
    ++row;
    if (line.empty()) {
      continue;
    }
    const auto fields = SplitCsvLine(line);
    if (fields.size() < 72) {
      AERROR << "Invalid gps_keyframe_raw_assoc.csv row " << row << ": "
             << line;
      return false;
    }

    Stage1GpsRawKeyframeObservation observation;
    observation.keyframe_id = std::stoul(fields[0]);
    observation.timestamp = std::stod(fields[1]);
    observation.has_stage1_gps = std::stoi(fields[2]) != 0;
    observation.has_raw_gps = std::stoi(fields[3]) != 0;
    observation.interp_before_time = std::stod(fields[4]);
    observation.interp_after_time = std::stod(fields[5]);
    observation.interp_alpha = std::stod(fields[6]);
    observation.interp_gap_before_s = std::stod(fields[7]);
    observation.interp_gap_after_s = std::stod(fields[8]);
    observation.antenna_utm_position = ParseVec3CsvFields(fields, 9);
    observation.stage1_imu_utm_position = ParseVec3CsvFields(fields, 12);
    observation.gps_heading_imu_utm_position = ParseVec3CsvFields(fields, 15);
    observation.lio_raw_yaw_imu_utm_position = ParseVec3CsvFields(fields, 18);
    observation.lio_opt_yaw_imu_utm_position = ParseVec3CsvFields(fields, 21);
    observation.configured_lever_arm = ParseVec3CsvFields(fields, 24);
    if (!ParsePoseCsvFields(fields, 36, &observation.lio_raw_pose) ||
        !ParsePoseCsvFields(fields, 43, &observation.lio_opt_pose)) {
      AERROR << "Invalid pose in gps_keyframe_raw_assoc.csv row " << row << ": "
             << line;
      return false;
    }
    observation.lio_raw_roll_rad = std::stod(fields[50]);
    observation.lio_raw_pitch_rad = std::stod(fields[51]);
    observation.lio_raw_yaw_rad = std::stod(fields[52]);
    observation.lio_opt_roll_rad = std::stod(fields[53]);
    observation.lio_opt_pitch_rad = std::stod(fields[54]);
    observation.lio_opt_yaw_rad = std::stod(fields[55]);
    observation.gnss_heading_rad = std::stod(fields[56]);
    observation.gnss_pitch_rad = std::stod(fields[57]);
    observation.heading_std_deg = std::stod(fields[58]);
    observation.pitch_std_deg = std::stod(fields[59]);
    observation.std_dev = ParseVec3CsvFields(fields, 60);
    observation.sol_status = static_cast<uint32_t>(std::stoul(fields[63]));
    observation.sol_type = static_cast<uint32_t>(std::stoul(fields[64]));
    observation.satellite_tracked =
        static_cast<uint32_t>(std::stoul(fields[65]));
    dataset->gps_raw_keyframe_observations.push_back(observation);
  }

  AINFO << "[Stage2Reader] Loaded raw GPS keyframe associations: "
        << dataset->gps_raw_keyframe_observations.size();
  return true;
}

}  // namespace

bool Stage1ArtifactReader::Read(const Stage2Config& config,
                                bool load_keyframe_clouds,
                                Stage1Dataset* dataset) const {
  if (dataset == nullptr) {
    return false;
  }
  dataset->keyframes.clear();
  dataset->keyframe_cloud_paths.clear();
  dataset->keyframe_covariance_paths.clear();
  dataset->gps_keyframe_observations.clear();
  dataset->gps_raw_keyframe_observations.clear();
  dataset->gps_full_history.clear();
  dataset->stage1_manifest_path.clear();
  dataset->stage1_generated_at.clear();
  dataset->stage1_vehicle_name.clear();
  dataset->dataset_sources.clear();
  dataset->expanded_records.clear();

  std::unordered_map<unsigned long, size_t> id_to_index;
  if (!ReadManifest(config, dataset)) {
    return false;
  }
  if (!ReadKeyframes(config, load_keyframe_clouds, dataset, &id_to_index)) {
    return false;
  }
  if (!ReadRelativeEdges(config, dataset, id_to_index)) {
    return false;
  }
  if (!ReadGpsKeyframeAssociations(config, dataset, id_to_index)) {
    return false;
  }
  if (!ReadGpsRawKeyframeAssociations(config, dataset)) {
    return false;
  }
  if (!ReadGpsFullHistory(config, dataset)) {
    return false;
  }
  return true;
}

}  // namespace stage2
}  // namespace air_mapping
}  // namespace apollo
