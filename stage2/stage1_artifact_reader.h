#pragma once

#include <string>

#include "modules/air_mapping/stage2/stage2_config.h"
#include "modules/air_mapping/stage2/stage2_types.h"

namespace apollo {
namespace air_mapping {
namespace stage2 {

class Stage1ArtifactReader {
 public:
  bool Read(const Stage2Config& config, bool load_keyframe_clouds,
            Stage1Dataset* dataset) const;
};

}  // namespace stage2
}  // namespace air_mapping
}  // namespace apollo
