#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "modules/air_mapping/stage1/stage1_config.h"
#include "modules/air_mapping/stage1/stage1_loop_optimizer.h"
#include "modules/air_mapping/system/common/gps_data.h"
#include "modules/air_mapping/system/common/keyframe.h"
#include "modules/air_mapping/system/common/point_def.h"

namespace apollo {
namespace air_mapping {
namespace stage1 {

struct Stage1GpsZLevelingSample {
  unsigned long keyframe_id = 0;
  double timestamp = 0.0;
  bool selected = false;
  std::string reject_reason;
  double gps_z = 0.0;
  double lio_z_before = 0.0;
  double lio_z_after = 0.0;
  double std_x = 0.0;
  double std_y = 0.0;
  double std_z = 0.0;
  uint32_t sol_type = 0;
};

struct Stage1GpsZLevelingResult {
  bool enabled = false;
  bool applied = false;
  std::string status_message;
  size_t candidate_count = 0;
  size_t selected_count = 0;
  double gps_mean_z = 0.0;
  double lio_mean_z_before = 0.0;
  double lio_mean_z_after = 0.0;
  double z_offset_m = 0.0;
  std::vector<Stage1GpsZLevelingSample> samples;
};

class Stage1ArtifactWriter {
 public:
  bool Write(const Stage1Config& config,
             const std::vector<lightning::Keyframe::Ptr>& keyframes,
             const std::vector<lightning::GpsFullObservation>& gps_history,
             const std::vector<Stage1LoopConstraint>& loop_constraints,
             const Stage1LoopSummary& loop_summary,
             const Stage1GpsZLevelingResult& z_leveling_result,
             lightning::CloudPtr preview_map) const;
};

}  // namespace stage1
}  // namespace air_mapping
}  // namespace apollo
