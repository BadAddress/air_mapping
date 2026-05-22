#include "modules/air_mapping/stage1/dual_lidar_fusion.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "cyber/common/log.h"
#include "yaml-cpp/yaml.h"

namespace apollo {
namespace air_mapping {
namespace stage1 {

namespace {

Eigen::Matrix3d MatrixFromYaml(const YAML::Node& node) {
  const auto values = node.as<std::vector<double>>();
  if (values.size() != 9) {
    throw std::runtime_error("rotation must contain 9 values");
  }
  Eigen::Matrix3d matrix;
  matrix << values[0], values[1], values[2], values[3], values[4], values[5],
      values[6], values[7], values[8];
  return Eigen::Quaterniond(matrix).normalized().toRotationMatrix();
}

Eigen::Vector3d VectorFromYaml(const YAML::Node& node) {
  const auto values = node.as<std::vector<double>>();
  if (values.size() != 3) {
    throw std::runtime_error("translation must contain 3 values");
  }
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

Eigen::Matrix3d QuaternionXyzwFromYaml(const YAML::Node& node) {
  const auto values = node.as<std::vector<double>>();
  if (values.size() != 4) {
    throw std::runtime_error("quaternion must contain 4 values");
  }
  Eigen::Quaterniond quat(values[3], values[0], values[1], values[2]);
  return quat.normalized().toRotationMatrix();
}

Eigen::Isometry3d TransformFromConfigNode(const YAML::Node& node) {
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  if (node["rotation"]) {
    rotation = MatrixFromYaml(node["rotation"]);
  } else if (node["rotation_quaternion_xyzw"]) {
    rotation = QuaternionXyzwFromYaml(node["rotation_quaternion_xyzw"]);
  } else {
    throw std::runtime_error("missing rotation or rotation_quaternion_xyzw");
  }

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = rotation;
  transform.translation() = VectorFromYaml(node["translation"]);
  return transform;
}

}  // namespace

bool DualLidarFusion::Init(const std::string& config_path) {
  try {
    const YAML::Node root = YAML::LoadFile(config_path);
    const YAML::Node dual = root["dual_lidar"];
    if (!dual) {
      AERROR << "[DualLidar] Missing dual_lidar section in " << config_path;
      return false;
    }

    config_.enable = dual["enable"].as<bool>(false);
    config_.primary_lidar = dual["primary_lidar"].as<std::string>("right");
    config_.secondary_lidar = dual["secondary_lidar"].as<std::string>("left");

    const YAML::Node channels = dual["channels"];
    config_.primary_channel =
        channels[config_.primary_lidar].as<std::string>();
    config_.secondary_channel =
        channels[config_.secondary_lidar].as<std::string>();

    const YAML::Node frames = dual["frames"];
    config_.primary_frame =
        frames[config_.primary_lidar + "_lidar"].as<std::string>();
    config_.secondary_frame =
        frames[config_.secondary_lidar + "_lidar"].as<std::string>();

    const YAML::Node sync = dual["sync"];
    config_.max_interval_sec =
        sync["max_interval_ms"].as<double>(50.0) * 1e-3;
    config_.max_cached_frames =
        sync["max_cached_frames"].as<std::size_t>(8);
    config_.allow_primary_only =
        sync["allow_primary_only"].as<bool>(true);

    const bool use_derived =
        (config_.primary_lidar == "right" &&
         config_.secondary_lidar == "left") ||
        (config_.primary_lidar == "left" &&
         config_.secondary_lidar == "right");

    if (use_derived && dual["derived"]) {
      const YAML::Node derived = dual["derived"];
      if (config_.primary_lidar == "right" &&
          config_.secondary_lidar == "left") {
        config_.T_primary_secondary =
            TransformFromConfigNode(derived["left_lidar_to_right_lidar"]);
      } else {
        const Eigen::Isometry3d T_right_left =
            TransformFromConfigNode(derived["left_lidar_to_right_lidar"]);
        config_.T_primary_secondary = T_right_left.inverse();
      }

      if (derived["right_lidar_to_imu"] &&
          config_.primary_lidar == "right") {
        config_.T_imu_primary =
            TransformFromConfigNode(derived["right_lidar_to_imu"]);
      } else if (derived["left_lidar_to_imu"] &&
                 config_.primary_lidar == "left") {
        config_.T_imu_primary =
            TransformFromConfigNode(derived["left_lidar_to_imu"]);
      }
    } else {
      const Eigen::Isometry3d T_imu_main =
          TransformFromConfigNode(dual["main_antenna_to_imu"]);
      const Eigen::Isometry3d T_main_primary =
          TransformFromConfigNode(
              dual[config_.primary_lidar + "_lidar_to_main_antenna"]);
      const Eigen::Isometry3d T_main_secondary =
          TransformFromConfigNode(
              dual[config_.secondary_lidar + "_lidar_to_main_antenna"]);
      config_.T_imu_primary = T_imu_main * T_main_primary;
      config_.T_primary_secondary =
          config_.T_imu_primary.inverse() * T_imu_main * T_main_secondary;
    }
  } catch (const std::exception& e) {
    AERROR << "[DualLidar] Failed to load " << config_path << ": "
           << e.what();
    return false;
  }

  AINFO << "[DualLidar] enable=" << (config_.enable ? "true" : "false")
        << ", primary=" << config_.primary_lidar << "("
        << config_.primary_channel << ")"
        << ", secondary=" << config_.secondary_lidar << "("
        << config_.secondary_channel << ")"
        << ", max_interval=" << config_.max_interval_sec << "s"
        << ", allow_primary_only="
        << (config_.allow_primary_only ? "true" : "false");
  AINFO << "[DualLidar] T_imu_primary t="
        << config_.T_imu_primary.translation().transpose();
  AINFO << "[DualLidar] T_primary_secondary t="
        << config_.T_primary_secondary.translation().transpose();
  return true;
}

void DualLidarFusion::PushPrimary(
    const std::shared_ptr<apollo::drivers::PointCloud>& cloud) {
  if (!cloud || !config_.enable) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  primary_buffer_.push_back(cloud);
  if (!config_.allow_primary_only) {
    TrimBuffersLocked();
  }
}

void DualLidarFusion::PushSecondary(
    const std::shared_ptr<apollo::drivers::PointCloud>& cloud) {
  if (!cloud || !config_.enable) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  secondary_buffer_.push_back(cloud);
  TrimBuffersLocked();
}

std::shared_ptr<apollo::drivers::PointCloud> DualLidarFusion::TryPopFused() {
  if (!config_.enable) {
    return nullptr;
  }

  std::shared_ptr<apollo::drivers::PointCloud> primary;
  std::shared_ptr<apollo::drivers::PointCloud> secondary;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!PopSyncedPairLocked(&primary, &secondary)) {
      return nullptr;
    }
  }

  if (!secondary) {
    return primary;
  }

  auto fused = std::make_shared<apollo::drivers::PointCloud>(*primary);
  fused->set_frame_id(config_.primary_frame);
  if (fused->has_header()) {
    fused->mutable_header()->set_frame_id(config_.primary_frame);
  }
  AppendTransformedCloud(*secondary, config_.T_primary_secondary, fused.get());
  return fused;
}

std::vector<std::shared_ptr<apollo::drivers::PointCloud>>
DualLidarFusion::FlushPrimaryOnly() {
  std::vector<std::shared_ptr<apollo::drivers::PointCloud>> flushed;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!config_.allow_primary_only) {
    primary_buffer_.clear();
    secondary_buffer_.clear();
    return flushed;
  }
  flushed.reserve(primary_buffer_.size());
  while (!primary_buffer_.empty()) {
    flushed.push_back(primary_buffer_.front());
    primary_buffer_.pop_front();
  }
  secondary_buffer_.clear();
  return flushed;
}

bool DualLidarFusion::PopSyncedPairLocked(
    std::shared_ptr<apollo::drivers::PointCloud>* primary,
    std::shared_ptr<apollo::drivers::PointCloud>* secondary) {
  while (!primary_buffer_.empty()) {
    const double primary_time = GetTimestampSec(*primary_buffer_.front());
    if (primary_time <= 0.0) {
      AWARN << "[DualLidar] Drop primary cloud with invalid timestamp.";
      primary_buffer_.pop_front();
      continue;
    }

    std::size_t best_index = secondary_buffer_.size();
    double best_dt = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < secondary_buffer_.size(); ++i) {
      const double secondary_time = GetTimestampSec(*secondary_buffer_[i]);
      if (secondary_time <= 0.0) {
        continue;
      }
      const double dt = std::abs(primary_time - secondary_time);
      if (dt < best_dt) {
        best_dt = dt;
        best_index = i;
      }
    }

    if (best_index != secondary_buffer_.size() &&
        best_dt <= config_.max_interval_sec) {
      *primary = primary_buffer_.front();
      *secondary = secondary_buffer_[best_index];
      primary_buffer_.pop_front();
      secondary_buffer_.erase(secondary_buffer_.begin(),
                              secondary_buffer_.begin() + best_index + 1);
      return true;
    }

    while (!secondary_buffer_.empty() &&
           GetTimestampSec(*secondary_buffer_.front()) <
               primary_time - config_.max_interval_sec) {
      secondary_buffer_.pop_front();
    }

    if (config_.allow_primary_only &&
        (primary_buffer_.size() > config_.max_cached_frames ||
         secondary_buffer_.size() >= config_.max_cached_frames)) {
      AWARN << "[DualLidar] No synced " << config_.secondary_lidar
            << " cloud for primary t=" << primary_time
            << ", best_dt=" << best_dt << ". Use primary only.";
      *primary = primary_buffer_.front();
      *secondary = nullptr;
      primary_buffer_.pop_front();
      return true;
    }

    if (secondary_buffer_.empty()) {
      return false;
    }
    const double newest_secondary_time =
        GetTimestampSec(*secondary_buffer_.back());
    if (newest_secondary_time <= primary_time + config_.max_interval_sec) {
      return false;
    }

    AWARN << "[DualLidar] Drop unmatched primary cloud. t=" << primary_time
          << ", best_dt=" << best_dt;
    primary_buffer_.pop_front();
  }
  return false;
}

void DualLidarFusion::TrimBuffersLocked() {
  while (!config_.allow_primary_only &&
         primary_buffer_.size() > config_.max_cached_frames) {
    AWARN << "[DualLidar] Drop oldest primary cloud because sync buffer is full.";
    primary_buffer_.pop_front();
  }
  while (secondary_buffer_.size() > config_.max_cached_frames) {
    AWARN << "[DualLidar] Drop oldest secondary cloud because sync buffer is full.";
    secondary_buffer_.pop_front();
  }
}

double DualLidarFusion::GetTimestampSec(
    const apollo::drivers::PointCloud& cloud) {
  if (cloud.has_measurement_time() && cloud.measurement_time() > 0.0) {
    return cloud.measurement_time();
  }
  if (cloud.has_header()) {
    return cloud.header().timestamp_sec();
  }
  return 0.0;
}

void DualLidarFusion::AppendTransformedCloud(
    const apollo::drivers::PointCloud& source,
    const Eigen::Isometry3d& transform,
    apollo::drivers::PointCloud* target) {
  if (target == nullptr) {
    return;
  }

  for (const auto& point : source.point()) {
    auto* output = target->add_point();
    output->set_intensity(point.intensity());
    output->set_timestamp(point.timestamp());

    if (std::isnan(point.x()) || std::isnan(point.y()) ||
        std::isnan(point.z())) {
      output->set_x(point.x());
      output->set_y(point.y());
      output->set_z(point.z());
      continue;
    }

    const Eigen::Vector3d input(point.x(), point.y(), point.z());
    const Eigen::Vector3d transformed = transform * input;
    output->set_x(static_cast<float>(transformed.x()));
    output->set_y(static_cast<float>(transformed.y()));
    output->set_z(static_cast<float>(transformed.z()));
  }

  target->set_height(1);
  target->set_width(static_cast<uint32_t>(target->point_size()));
  target->set_is_dense(target->is_dense() && source.is_dense());
}

}  // namespace stage1
}  // namespace air_mapping
}  // namespace apollo
