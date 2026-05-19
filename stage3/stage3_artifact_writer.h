#pragma once

#include <vector>

#include "modules/air_mapping/stage3/stage3_config.h"
#include "modules/air_mapping/stage3/stage3_types.h"
#include "modules/air_mapping/system/common/point_def.h"

namespace apollo {
namespace air_mapping {
namespace stage3 {

class Stage3ArtifactWriter {
 public:
  bool Write(const Stage3Config& config, const Stage2Dataset& dataset,
             const Stage3RefineSummary& summary,
             const std::vector<Stage3OutageBlock>& outage_blocks,
             lightning::CloudPtr preview_map) const;
};

}  // namespace stage3
}  // namespace air_mapping
}  // namespace apollo
