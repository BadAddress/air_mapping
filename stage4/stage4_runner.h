#pragma once

#include "modules/air_mapping/stage4/stage4_config.h"

namespace apollo {
namespace air_mapping {
namespace stage4 {

class Stage4Runner {
 public:
  bool Run(const Stage4Config& config) const;
};

}  // namespace stage4
}  // namespace air_mapping
}  // namespace apollo
