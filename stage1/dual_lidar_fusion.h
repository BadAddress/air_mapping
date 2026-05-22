#pragma once

#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

namespace apollo {
namespace air_mapping {
namespace stage1 {

class DualLidarFusion {
 public:
  struct Config {
    bool enable = false;
    std::string primary_lidar = "right";
    std::string secondary_lidar = "left";
    std::string primary_channel;
    std::string secondary_channel;
    std::string primary_frame = "rslidar_right";
    std::string secondary_frame = "rslidar_left";
    double max_interval_sec = 0.05;
    std::size_t max_cached_frames = 8;
    bool allow_primary_only = true;
    Eigen::Isometry3d T_primary_secondary = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_imu_primary = Eigen::Isometry3d::Identity();
  };

  bool Init(const std::string& config_path);
  bool enabled() const { return config_.enable; }
  const Config& config() const { return config_; }

  void PushPrimary(const std::shared_ptr<apollo::drivers::PointCloud>& cloud);
  void PushSecondary(const std::shared_ptr<apollo::drivers::PointCloud>& cloud);
  std::shared_ptr<apollo::drivers::PointCloud> TryPopFused();
  std::vector<std::shared_ptr<apollo::drivers::PointCloud>> FlushPrimaryOnly();

 private:
  bool PopSyncedPairLocked(
      std::shared_ptr<apollo::drivers::PointCloud>* primary,
      std::shared_ptr<apollo::drivers::PointCloud>* secondary);
  void TrimBuffersLocked();
  static double GetTimestampSec(const apollo::drivers::PointCloud& cloud);
  static void AppendTransformedCloud(const apollo::drivers::PointCloud& source,
                                     const Eigen::Isometry3d& transform,
                                     apollo::drivers::PointCloud* target);

  Config config_;
  std::mutex mutex_;
  std::deque<std::shared_ptr<apollo::drivers::PointCloud>> primary_buffer_;
  std::deque<std::shared_ptr<apollo::drivers::PointCloud>> secondary_buffer_;
};

}  // namespace stage1
}  // namespace air_mapping
}  // namespace apollo
