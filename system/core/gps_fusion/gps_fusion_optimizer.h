//
// GPS Fusion Optimizer - Backend factor graph optimizer for LIO + GPS fusion
//

#ifndef LIGHTNING_GPS_FUSION_OPTIMIZER_H
#define LIGHTNING_GPS_FUSION_OPTIMIZER_H

#include <vector>
#include <memory>
#include <atomic>
#include <cstdint>
#include <limits>
#include <mutex>
#include <unordered_set>
#include "../../common/keyframe.h"
#include "../../common/gps_data.h"
#include "../../common/eigen_types.h"
#include "../system/async_message_process.h"

namespace lightning {

enum class LoopConstraintOrigin : uint8_t {
    kAsync = 0,
    kFinalCatchup = 1,
    kOutageSupportRematch = 2,
};

struct GpsFusionOptimizerOptions {
    bool enable_gps_position = true;
    bool enable_gps_heading = true;
    bool enable_gps_height = false;
    
    double weight_lio_relative = 1.0;
    double weight_height_smooth = 10.0;
    double weight_gps_position = 100.0;
    double weight_gps_heading = 10.0;
    double weight_gps_height = 0.5;

    double min_gps_std_xy = 0.05;
    double max_gps_info_xy = 400.0;
    double gps_anchor_min_spacing_m = 8.0;
    double gps_segment_break_distance_m = 25.0;
    double gps_boundary_ramp_distance_m = 15.0;
    int gps_segment_min_points = 3;
    double gps_height_trend_anchor_spacing_m = 15.0;
    int gps_height_trend_min_points = 5;
    double gps_height_trend_smoothing_lambda = 200.0;
    double gps_height_trend_min_std_z = 0.05;
    bool enable_outage_rigid_segments = true;
    bool enable_outage_block_scan_to_map = true;
    int outage_rigid_min_keyframes = 8;
    double outage_rigid_min_length_m = 20.0;
    double max_outage_rigid_block_length_m = 100.0;
    int outage_overlap_keyframes = 5;
    double outage_block_target_radius_m = 200.0;
    double outage_block_source_window_radius_m = 60.0;
    double outage_block_window_spacing_m = 80.0;
    int outage_block_min_source_keyframes = 8;
    int outage_block_target_sample_step = 2;
    double outage_block_max_coarse_z_search_m = 12.0;
    double outage_block_coarse_z_step_m = 4.0;
    double outage_block_prior_translation_sigma_m = 0.6;
    double outage_block_prior_rotation_sigma_deg = 4.0;
    bool disable_gps_height_on_long_outage = false;
    double long_outage_threshold_m = 100.0;
    
    double huber_lio_delta = 1.0;
    double huber_gps_pos_delta = 2.0;
    double cauchy_heading_delta = 5.0;
    
    int max_iterations = 50;
    bool verbose = false;
};

struct LoopClosureOptions {
    bool enable = false;
    double search_radius = 15.0;
    int min_keyframe_gap = 30;
    int loop_kf_gap = 10;
    int min_id_interval = 20;
    int closest_id_threshold = 50;
    int history_submap_half_range = 40;
    int history_submap_step = 4;
    int icp_max_iterations = 100;
    double icp_max_corr_dist = 2.0;
    double icp_fitness_threshold = 0.3;
    double info_scale = 100.0;
};

class GpsFusionOptimizer {
public:
    explicit GpsFusionOptimizer(const GpsFusionOptimizerOptions& opts);
    ~GpsFusionOptimizer();
    
    void AddKeyframe(const Keyframe::Ptr& kf);
    void SetRawGpsHistory(const std::vector<GpsFullObservation>& gps_history);
    void FinalOptimize();
    
    void SetLoopClosureOptions(const LoopClosureOptions& opts);
    
    int GetTotalKeyframes() const;
    bool IsOptimizing() const { return is_optimizing_.load(); }
    
private:
    struct PrecomputedLoopConstraint {
        size_t target_idx = 0;
        size_t source_idx = 0;
        SE3 measurement_target_to_source;
        double ndt_score = 0.0;
        LoopConstraintOrigin origin = LoopConstraintOrigin::kAsync;
    };

    struct LoopQueryTask {
        size_t query_idx = 0;
        bool force_check = false;
    };

    void Optimize(bool is_final);
    void StartLoopWorkerIfNeeded();
    void StopLoopWorker();
    void EnqueuePendingLoopQueriesBeforeFinalOptimize();
    void ProcessLoopClosureQuery(size_t query_idx, bool force_check);
    std::vector<PrecomputedLoopConstraint> GetLoopConstraintsSnapshot() const;
    
    GpsFusionOptimizerOptions options_;
    LoopClosureOptions loop_opts_;
    
    mutable std::mutex keyframes_mutex_;
    std::vector<Keyframe::Ptr> keyframes_;
    mutable std::mutex raw_gps_mutex_;
    std::vector<GpsFullObservation> raw_gps_history_;
    mutable std::mutex loop_constraints_mutex_;
    std::vector<PrecomputedLoopConstraint> loop_constraints_;
    std::unordered_set<uint64_t> loop_constraint_keys_;
    sys::AsyncMessageProcess<LoopQueryTask> loop_kf_thread_;
    std::atomic<bool> loop_worker_started_{false};
    size_t last_loop_query_idx_ = std::numeric_limits<size_t>::max();
    std::atomic<bool> is_optimizing_{false};
    
    Vec3d utm_to_lio_offset_ = Vec3d::Zero();
    bool has_utm_offset_ = false;
};

}  // namespace lightning

#endif  // LIGHTNING_GPS_FUSION_OPTIMIZER_H
