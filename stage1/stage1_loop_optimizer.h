#pragma once

#include <cstddef>
#include <vector>

#include "modules/air_mapping/stage1/stage1_config.h"
#include "modules/air_mapping/system/common/keyframe.h"

namespace apollo {
namespace air_mapping {
namespace stage1 {

struct Stage1LoopConstraint {
  size_t target_index = 0;
  size_t source_index = 0;
  unsigned long target_id = 0;
  unsigned long source_id = 0;
  lightning::SE3 measurement_target_to_source;
  double ndt_score = 0.0;
  double icp_fitness = -1.0;
  bool used_icp = false;
  double delta_from_initial_translation_m = 0.0;
  double delta_from_initial_rotation_deg = 0.0;
};

struct Stage1LoopSummary {
  bool enabled = false;
  size_t keyframe_count = 0;
  size_t query_count = 0;
  size_t coarse_candidate_count = 0;
  size_t accepted_loop_count = 0;
  size_t lio_edge_count = 0;
  size_t loop_edge_count = 0;
  size_t loop_outlier_edge_count = 0;
  int optimizer_iterations = 0;
  double chi2_before = 0.0;
  double chi2_after = 0.0;
  bool zleveling_enabled = false;
  size_t zleveling_height_prior_edge_count = 0;
  int zleveling_optimizer_iterations = 0;
  double zleveling_chi2_before = 0.0;
  double zleveling_chi2_after = 0.0;
  double zleveling_max_abs_height_before_m = 0.0;
  double zleveling_max_abs_height_after_m = 0.0;
  double zleveling_mean_z_before = 0.0;
  double zleveling_mean_z_after = 0.0;
};

class Stage1LoopOptimizer {
 public:
  bool Optimize(const Stage1Config& config,
                const std::vector<lightning::Keyframe::Ptr>& keyframes,
                std::vector<Stage1LoopConstraint>* loop_constraints,
                Stage1LoopSummary* summary) const;
};

}  // namespace stage1
}  // namespace air_mapping
}  // namespace apollo
