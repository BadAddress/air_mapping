#pragma once

#include "modules/air_mapping/stage3/stage3_config.h"
#include "modules/air_mapping/stage3/stage3_types.h"

namespace apollo {
namespace air_mapping {
namespace stage3 {

class Stage2ArtifactReader {
 public:
  bool Read(const Stage3Config& config, bool load_keyframe_clouds,
            Stage2Dataset* dataset) const;
};

}  // namespace stage3
}  // namespace air_mapping
}  // namespace apollo
