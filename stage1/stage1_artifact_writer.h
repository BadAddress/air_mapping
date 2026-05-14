#pragma once

#include <string>
#include <vector>

#include "modules/air_mapping/stage1/stage1_config.h"
#include "modules/air_mapping/system/common/gps_data.h"
#include "modules/air_mapping/system/common/keyframe.h"
#include "modules/air_mapping/system/common/point_def.h"

namespace apollo {
namespace air_mapping {
namespace stage1 {

class Stage1ArtifactWriter {
 public:
  bool Write(const Stage1Config& config,
             const std::vector<lightning::Keyframe::Ptr>& keyframes,
             const std::vector<lightning::GpsFullObservation>& gps_history,
             lightning::CloudPtr preview_map) const;
};

}  // namespace stage1
}  // namespace air_mapping
}  // namespace apollo

