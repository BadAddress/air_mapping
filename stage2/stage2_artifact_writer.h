#pragma once

#include <vector>

#include "modules/air_mapping/stage2/stage2_config.h"
#include "modules/air_mapping/stage2/stage2_types.h"
#include "modules/air_mapping/system/common/point_def.h"

namespace apollo {
namespace air_mapping {
namespace stage2 {

class Stage2ArtifactWriter {
 public:
  bool Write(const Stage2Config& config, const Stage1Dataset& dataset,
             const std::vector<AlignmentAnchor>& anchors,
             const std::vector<GpsSegmentSummary>& segments,
             const Stage2AlignmentResult& result,
             const LeverArmCalibrationResult& lever_arm_result,
             lightning::CloudPtr preview_map) const;
};

}  // namespace stage2
}  // namespace air_mapping
}  // namespace apollo
