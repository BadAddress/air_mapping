#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include "modules/air_mapping/stage1/stage1_config.h"
#include "modules/air_mapping/system/common/debug_utils.h"
#include "modules/air_mapping/system/core/system/slam.h"
#include "modules/common_msgs/localization_msgs/gps.pb.h"
#include "modules/common_msgs/localization_msgs/imu.pb.h"
#include "modules/common_msgs/sensor_msgs/gnss_best_pose.pb.h"
#include "modules/common_msgs/sensor_msgs/heading.pb.h"
#include "modules/common_msgs/sensor_msgs/imu.pb.h"
#include "modules/common_msgs/sensor_msgs/ins.pb.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

namespace apollo {
namespace air_mapping {
namespace stage1 {

class Stage1Runner {
 public:
  bool Run(const Stage1Config& config);

 private:
  bool ProcessRecord(const std::string& record_path);
  void ProcessCorrectedImu(const apollo::localization::CorrectedImu& imu);
  void ProcessRawImu(const apollo::drivers::gnss::Imu& imu);
  void ProcessHeading(const apollo::drivers::gnss::Heading& heading);
  void ProcessBestPose(const apollo::drivers::gnss::GnssBestPose& best_pose);
  void ProcessInsStat(const apollo::drivers::gnss::InsStat& ins_stat);
  void ProcessGpsOdom(const apollo::localization::Gps& gps);

  bool IsGpsInsValid() const;
  bool IsGpsSolutionValid(const apollo::drivers::gnss::GnssBestPose& msg) const;
  bool IsAcceptedRtkSolutionType(uint32_t type) const;
  double GetHeadingTimestampSec(const apollo::drivers::gnss::Heading& msg) const;

  Stage1Config config_;
  std::shared_ptr<lightning::SlamSystem> slam_system_;

  std::atomic<uint32_t> latest_ins_status_{0};
  std::atomic<uint32_t> latest_pos_type_{0};
  std::atomic<bool> has_ins_stat_{false};
  std::unique_ptr<lightning::GpsOdomRecorder> gps_odom_recorder_;
};

}  // namespace stage1
}  // namespace air_mapping
}  // namespace apollo
