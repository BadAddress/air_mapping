#include "modules/air_mapping/stage3/stage2_artifact_reader.h"

#include <exception>
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

namespace apollo {
namespace air_mapping {
namespace stage3 {

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
                                          const std::string& path) {
  std::filesystem::path artifact_path(path);
  if (artifact_path.is_absolute()) {
    std::error_code error;
    if (std::filesystem::exists(artifact_path, error)) {
      return artifact_path;
    }
  }
  return root / artifact_path;
}

std::filesystem::path FindModuleRoot() {
  std::error_code error;
  std::filesystem::path path = std::filesystem::current_path(error);
  while (!error && !path.empty()) {
    if (std::filesystem::exists(path / "cyberfile.xml", error) &&
        std::filesystem::exists(path / "stage1", error) &&
        std::filesystem::exists(path / "stage2", error)) {
      return path;
    }
    const auto candidate = path / "modules" / "air_mapping";
    if (std::filesystem::exists(candidate / "cyberfile.xml", error)) {
      return candidate;
    }
    if (path == path.root_path()) {
      break;
    }
    path = path.parent_path();
  }
  return std::filesystem::current_path();
}

std::filesystem::path ResolveExistingPath(const std::string& path) {
  std::filesystem::path candidate(path);
  std::error_code error;
  if (std::filesystem::exists(candidate, error)) {
    return candidate;
  }

  const std::string prefix = "/apollo_workspace/modules/air_mapping/";
  if (path.rfind(prefix, 0) == 0) {
    const auto local_candidate = FindModuleRoot() / path.substr(prefix.size());
    if (std::filesystem::exists(local_candidate, error)) {
      return local_candidate;
    }
  }
  return candidate;
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
  if (!translation.allFinite() || quaternion.norm() < 1e-12) {
    return false;
  }
  quaternion.normalize();
  *pose = lightning::SE3(lightning::SO3(quaternion), translation);
  return true;
}

bool ReadUtmOrigin(const Stage3Config& config, Stage2Dataset* dataset) {
  const auto input_dir = ResolveExistingPath(config.input_dir);
  const auto origin_path = input_dir / "alignment" / "utm_origin.txt";
  std::ifstream file(origin_path);
  if (!file.is_open()) {
    AERROR << "Stage2 UTM origin is required for stage3 graph refinement: "
           << origin_path.string();
    return false;
  }

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::stringstream stream(line);
    if (stream >> dataset->utm_origin.x() >> dataset->utm_origin.y() >>
        dataset->utm_origin.z()) {
      dataset->has_utm_origin = true;
      return true;
    }
  }

  AERROR << "Invalid UTM origin file: " << origin_path.string();
  return false;
}

bool ReadManifest(const Stage3Config& config, Stage2Dataset* dataset) {
  const auto input_dir = ResolveExistingPath(config.input_dir);
  const auto manifest_path = input_dir / "manifest.yaml";
  std::ifstream file(manifest_path);
  if (!file.is_open()) {
    AWARN << "Stage2 manifest not found, continue without source metadata: "
          << manifest_path.string();
    return true;
  }

  std::string line;
  while (std::getline(file, line)) {
    const auto separator = line.find(':');
    if (separator == std::string::npos) {
      continue;
    }
    const std::string key = line.substr(0, separator);
    std::string value = line.substr(separator + 1);
    while (!value.empty() && value.front() == ' ') {
      value.erase(value.begin());
    }
    if (key == "source_stage1_dir") {
      dataset->source_stage1_dir = value;
    } else if (key == "source_config_path") {
      dataset->source_config_path = value;
    }
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
  if (config_path.empty()) {
    AWARN << "Stage2 manifest has no source_config_path, use identity lidar "
             "extrinsic";
    return true;
  }

  std::error_code error;
  const auto resolved_config_path = ResolveExistingPath(config_path);
  if (!std::filesystem::exists(resolved_config_path, error)) {
    AWARN << "Source config not found, use identity lidar extrinsic: "
          << config_path;
    return true;
  }

  try {
    YAML::Node yaml = YAML::LoadFile(resolved_config_path.string());
    if (!yaml["fasterlio"]) {
      AWARN << "No fasterlio section in source config, use identity lidar "
               "extrinsic: "
            << resolved_config_path.string();
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
    AERROR << "Failed to load lidar extrinsic from "
           << resolved_config_path.string() << ", error: " << e.what();
    return false;
  }

  AINFO << "[Stage3Reader] Loaded lidar extrinsic from "
        << resolved_config_path.string() << ", t=" << translation->transpose();
  return true;
}

bool ReadKeyframes(const Stage3Config& config, bool load_keyframe_clouds,
                   Stage2Dataset* dataset,
                   std::unordered_map<unsigned long, size_t>* id_to_index) {
  const auto input_dir = ResolveExistingPath(config.input_dir);
  const auto keyframes_path = input_dir / "keyframes" / "keyframes_opt.csv";
  std::ifstream file(keyframes_path);
  if (!file.is_open()) {
    AERROR << "Failed to open stage2 keyframes: " << keyframes_path.string();
    return false;
  }

  lightning::SO3 lidar_rotation;
  lightning::Vec3d lidar_translation;
  if (!LoadLidarExtrinsic(dataset->source_config_path, &lidar_rotation,
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
    if (fields.size() < 27) {
      AERROR << "Invalid keyframes_opt.csv row " << row << ": " << line;
      return false;
    }

    Stage2KeyframeRecord record;
    record.id = std::stoul(fields[0]);
    record.timestamp = std::stod(fields[1]);
    if (!ParsePoseCsvFields(fields, 2, &record.lio_pose) ||
        !ParsePoseCsvFields(fields, 9, &record.stage2_pose) ||
        !ParsePoseCsvFields(fields, 16, &record.utm_pose)) {
      AERROR << "Invalid pose in keyframes_opt.csv row " << row << ": " << line;
      return false;
    }
    record.source_stage1_dir = fields[23];
    record.source_cloud_path = fields[24];
    record.source_covariance_path = fields[25];
    record.has_gps = std::stoi(fields[26]) != 0;

    lightning::CloudPtr cloud(new lightning::PointCloudType);
    if (load_keyframe_clouds && !record.source_cloud_path.empty()) {
      const auto source_stage1_dir =
          ResolveExistingPath(record.source_stage1_dir);
      const auto cloud_path =
          ResolveArtifactPath(source_stage1_dir, record.source_cloud_path);
      if (pcl::io::loadPCDFile<lightning::PointType>(cloud_path.string(),
                                                     *cloud) < 0) {
        AERROR << "Failed to load keyframe cloud: " << cloud_path.string();
        return false;
      }
    }

    lightning::NavState state;
    state.timestamp_ = record.timestamp;
    state.SetPose(record.stage2_pose);
    state.offset_R_lidar_ = lidar_rotation;
    state.offset_t_lidar_ = lidar_translation;
    auto keyframe =
        std::make_shared<lightning::Keyframe>(record.id, cloud, state);
    keyframe->SetLIOPose(record.stage2_pose);
    keyframe->SetOptPose(record.stage2_pose);

    (*id_to_index)[record.id] = dataset->keyframes.size();
    dataset->records.push_back(record);
    dataset->keyframes.push_back(keyframe);
    dataset->keyframe_cloud_paths.push_back(record.source_cloud_path);
    dataset->keyframe_covariance_paths.push_back(record.source_covariance_path);
  }

  AINFO << "[Stage3Reader] Loaded stage2 keyframes: "
        << dataset->keyframes.size()
        << ", clouds=" << (load_keyframe_clouds ? "loaded" : "skipped");
  return !dataset->keyframes.empty();
}

bool ReadRelativeEdges(
    const Stage3Config& config, Stage2Dataset* dataset,
    const std::unordered_map<unsigned long, size_t>& id_to_index) {
  const auto relative_path = ResolveExistingPath(config.input_dir) /
                             "keyframes" / "relative_edges.csv";
  std::ifstream file(relative_path);
  if (!file.is_open()) {
    AERROR << "Failed to open stage2 relative edges: "
           << relative_path.string();
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
    if (fields.size() < 9) {
      AERROR << "Invalid relative_edges.csv row " << row << ": " << line;
      return false;
    }

    Stage2RelativeEdge edge;
    edge.from_id = std::stoul(fields[0]);
    edge.to_id = std::stoul(fields[1]);
    auto from_it = id_to_index.find(edge.from_id);
    auto to_it = id_to_index.find(edge.to_id);
    if (from_it == id_to_index.end() || to_it == id_to_index.end()) {
      AWARN << "Skip relative edge with missing keyframe: " << edge.from_id
            << " -> " << edge.to_id;
      continue;
    }
    edge.from_index = from_it->second;
    edge.to_index = to_it->second;
    if (!ParsePoseCsvFields(fields, 2, &edge.measurement)) {
      AERROR << "Invalid relative edge pose at row " << row << ": " << line;
      return false;
    }
    dataset->relative_edges.push_back(edge);
  }

  AINFO << "[Stage3Reader] Loaded stage2 relative edges: "
        << dataset->relative_edges.size();
  return true;
}

bool ReadGpsAnchors(
    const Stage3Config& config, Stage2Dataset* dataset,
    const std::unordered_map<unsigned long, size_t>& id_to_index) {
  const auto anchors_path = ResolveExistingPath(config.input_dir) /
                            "diagnostics" / "alignment_anchors.csv";
  std::ifstream file(anchors_path);
  if (!file.is_open()) {
    AERROR << "Failed to open stage2 alignment anchors: "
           << anchors_path.string();
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
    if (fields.size() < 18) {
      AERROR << "Invalid alignment_anchors.csv row " << row << ": " << line;
      return false;
    }

    Stage2GpsAnchor anchor;
    anchor.keyframe_id = std::stoul(fields[0]);
    auto id_it = id_to_index.find(anchor.keyframe_id);
    if (id_it == id_to_index.end()) {
      AWARN << "Skip GPS anchor for missing keyframe: " << anchor.keyframe_id;
      continue;
    }
    anchor.keyframe_index = id_it->second;
    anchor.timestamp = std::stod(fields[1]);
    anchor.segment_id = std::stoi(fields[2]);
    anchor.stage2_weight = std::stod(fields[3]);
    anchor.gps_utm_position = lightning::Vec3d(
        std::stod(fields[7]), std::stod(fields[8]), std::stod(fields[9]));
    anchor.gps_smooth_utm_position = lightning::Vec3d(
        std::stod(fields[10]), std::stod(fields[11]), std::stod(fields[12]));
    anchor.gps_std_dev = lightning::Vec3d(
        std::stod(fields[13]), std::stod(fields[14]), std::stod(fields[15]));
    anchor.residual_before_m = std::stod(fields[16]);
    anchor.residual_after_m = std::stod(fields[17]);
    if (!anchor.gps_smooth_utm_position.allFinite() ||
        !anchor.gps_std_dev.allFinite()) {
      AWARN << "Skip non-finite GPS anchor at row " << row;
      continue;
    }
    dataset->gps_anchors.push_back(anchor);
  }

  AINFO << "[Stage3Reader] Loaded stage2 GPS anchors: "
        << dataset->gps_anchors.size();
  return !dataset->gps_anchors.empty();
}

}  // namespace

bool Stage2ArtifactReader::Read(const Stage3Config& config,
                                bool load_keyframe_clouds,
                                Stage2Dataset* dataset) const {
  if (dataset == nullptr) {
    return false;
  }
  dataset->records.clear();
  dataset->keyframes.clear();
  dataset->keyframe_cloud_paths.clear();
  dataset->keyframe_covariance_paths.clear();
  dataset->relative_edges.clear();
  dataset->gps_anchors.clear();
  dataset->source_stage1_dir.clear();
  dataset->source_config_path.clear();
  dataset->utm_origin = lightning::Vec3d::Zero();
  dataset->has_utm_origin = false;

  std::unordered_map<unsigned long, size_t> id_to_index;
  if (!ReadUtmOrigin(config, dataset)) {
    return false;
  }
  if (!ReadManifest(config, dataset)) {
    return false;
  }
  if (!ReadKeyframes(config, load_keyframe_clouds, dataset, &id_to_index)) {
    return false;
  }
  if (!ReadRelativeEdges(config, dataset, id_to_index)) {
    return false;
  }
  if (!ReadGpsAnchors(config, dataset, id_to_index)) {
    return false;
  }
  return true;
}

}  // namespace stage3
}  // namespace air_mapping
}  // namespace apollo
