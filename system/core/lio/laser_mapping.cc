#include "cyber/common/log.h"
#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h> // ADDED
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <limits>

#include "common/options.h"
#include "core/lightning_math.hpp"
#include "laser_mapping.h"
// GPS fusion optimizer removed
// #include "core/gps_fusion/gps_fusion_optimizer.cc"
// #include "ui/pangolin_window.h"
// #include "wrapper/ros_utils.h"

namespace lightning {

bool LaserMapping::Init(const std::string &config_yaml) {
    AINFO << "init laser mapping from " << config_yaml;
    if (!LoadParamsFromYAML(config_yaml)) {
        return false;
    }

    // localmap init (after LoadParams)
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    // esekf init
    ESKF::Options eskf_options;
    eskf_options.max_iterations_ = fasterlio::NUM_MAX_ITERATIONS;
    eskf_options.epsi_ = 1e-3 * Eigen::Matrix<double, 23, 1>::Ones();
    eskf_options.lidar_obs_func_ = [this](NavState &s, ESKF::CustomObservationModel &obs) { ObsModel(s, obs); };
    eskf_options.use_aa_ = use_aa_;
    kf_.Init(eskf_options);

    return true;
}

bool LaserMapping::LoadParamsFromYAML(const std::string &yaml_file) {
    // get params from yaml
    int lidar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_scan;
    Vec3d lidar_T_wrt_IMU;
    Mat3d lidar_R_wrt_IMU;

    auto yaml = YAML::LoadFile(yaml_file);
    try {
        fasterlio::NUM_MAX_ITERATIONS = yaml["fasterlio"]["max_iteration"].as<int>();
        fasterlio::ESTI_PLANE_THRESHOLD = yaml["fasterlio"]["esti_plane_threshold"].as<float>();

        filter_size_scan = yaml["fasterlio"]["filter_size_scan"].as<float>();
        filter_size_map_min_ = yaml["fasterlio"]["filter_size_map"].as<float>();
        keep_first_imu_estimation_ = yaml["fasterlio"]["keep_first_imu_estimation"].as<bool>();
        gyr_cov = yaml["fasterlio"]["gyr_cov"].as<float>();
        acc_cov = yaml["fasterlio"]["acc_cov"].as<float>();
        b_gyr_cov = yaml["fasterlio"]["b_gyr_cov"].as<float>();
        b_acc_cov = yaml["fasterlio"]["b_acc_cov"].as<float>();
        preprocess_->Blind() = yaml["fasterlio"]["blind"].as<double>();
        preprocess_->TimeScale() = yaml["fasterlio"]["time_scale"].as<double>();
        
        if (yaml["fasterlio"]["kf_dist_th"]) {
            options_.kf_dis_th_ = yaml["fasterlio"]["kf_dist_th"].as<double>();
        }
        if (yaml["fasterlio"]["kf_angle_th"]) {
            options_.kf_angle_th_ = yaml["fasterlio"]["kf_angle_th"].as<double>() * M_PI / 180.0;
        }

        lidar_type = yaml["fasterlio"]["lidar_type"].as<int>();
        preprocess_->NumScans() = yaml["fasterlio"]["scan_line"].as<int>();
        preprocess_->PointFilterNum() = yaml["fasterlio"]["point_filter_num"].as<int>();
        if (yaml["fasterlio"]["mapping_point_filter_num"]) {
            preprocess_->MappingPointFilterNum() = yaml["fasterlio"]["mapping_point_filter_num"].as<int>();
        } else {
            // 默认用 point_filter_num / 2，保留比定位多的点但不至于全量
            preprocess_->MappingPointFilterNum() = std::max(1, preprocess_->PointFilterNum() / 2);
        }
        extrinsic_est_en_ = yaml["fasterlio"]["extrinsic_est_en"].as<bool>();
        extrinT_ = yaml["fasterlio"]["extrinsic_T"].as<std::vector<double>>();
        extrinR_ = yaml["fasterlio"]["extrinsic_R"].as<std::vector<double>>();

        ivox_options_.resolution_ = yaml["fasterlio"]["ivox_grid_resolution"].as<float>();
        ivox_nearby_type = yaml["fasterlio"]["ivox_nearby_type"].as<int>();
        use_aa_ = yaml["fasterlio"]["use_aa"].as<bool>();

        skip_lidar_num_ = yaml["fasterlio"]["skip_lidar_num"].as<int>();
        enable_skip_lidar_ = skip_lidar_num_ > 0;
        
        // Load GPS heading initialization parameters
        if (yaml["gps_heading_init"]) {
            use_gps_heading_init_ = yaml["gps_heading_init"]["enable"].as<bool>(false);
            if (use_gps_heading_init_) {
                AINFO << "[GPS_HEADING_INIT] GPS heading initialization enabled";
            } else {
                AINFO << "[GPS_HEADING_INIT] GPS heading initialization disabled";
            }
        } else {
            AINFO << "[GPS_HEADING_INIT] No gps_heading_init config found, feature disabled";
        }

    } catch (...) {
        AERROR << "bad conversion";
        return false;
    }

    AINFO << "lidar_type " << lidar_type;
    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::AVIA);
        AINFO << "Using AVIA Lidar";
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::VELO32);
        AINFO << "Using Velodyne 32 Lidar";
    } else if (lidar_type == 3) {
        preprocess_->SetLidarType(LidarType::OUST64);
        AINFO << "Using OUST 64 Lidar";
    } else {
        AWARN << "unknown lidar_type";
        return false;
    }

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        AWARN << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    voxel_scan_.setLeafSize(filter_size_scan, filter_size_scan, filter_size_scan);

    lidar_T_wrt_IMU = math::VecFromArray<double>(extrinT_);
    lidar_R_wrt_IMU = math::MatFromArray<double>(extrinR_);

    p_imu_->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    p_imu_->SetGyrCov(Vec3d(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(Vec3d(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(Vec3d(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(Vec3d(b_acc_cov, b_acc_cov, b_acc_cov));

    return true;
}

LaserMapping::LaserMapping(Options options) : options_(options) {
    preprocess_.reset(new PointCloudPreprocess());
    p_imu_.reset(new ImuProcess());
}

void LaserMapping::ProcessIMU(const lightning::IMUPtr &imu) {
    publish_count_++;

    double timestamp = imu->timestamp;

    UL lock(mtx_buffer_);
    if (timestamp < last_timestamp_imu_) {
        AWARN << "imu loop back, clear buffer";
        imu_buffer_.clear();
    }

    if (p_imu_->IsIMUInited()) {
        /// 更新最新imu状态
        kf_imu_.Predict(timestamp - last_timestamp_imu_, p_imu_->Q_, imu->angular_velocity, imu->linear_acceleration);

        // AINFO << "newest wrt lidar: " << timestamp - kf_.GetX().timestamp_;

        /// 更新ui
        /*
        if (ui_) {
            ui_->UpdateNavState(kf_imu_.GetX());
        }
        */
    }

    last_timestamp_imu_ = timestamp;

    imu_buffer_.emplace_back(imu);
}

bool LaserMapping::Run() {
    if (!SyncPackages()) {
        return false;
    }

    /// IMU process, kf prediction, undistortion
    p_imu_->Process(measures_, kf_, scan_undistort_);

    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        AWARN << "No point, skip this scan!";
        return false;
    }
    
    /// Apply GPS heading initialization if enabled and not yet applied.
    /// Use the newest valid heading from cache.
    if (use_gps_heading_init_ && !gps_heading_applied_ && p_imu_->IsIMUInited()) {
        std::lock_guard<std::mutex> lock(mtx_heading_);
        
        AINFO << "[GPS_HEADING_INIT] IMU initialized, attempting to apply GPS heading. Cache size: " 
              << heading_cache_.size();
        
        // Pick the newest valid heading
        const HeadingCache* best = nullptr;
        for (auto it = heading_cache_.rbegin(); it != heading_cache_.rend(); ++it) {
            if (it->valid) {
                best = &(*it);
                break;
            }
        }
        
        if (best) {
            AINFO << "[GPS_HEADING_INIT] Applying GPS heading: yaw="
                  << (best->heading_rad * 180.0 / M_PI) << " deg, pitch="
                  << (best->pitch_rad * 180.0 / M_PI) << " deg";
            
            p_imu_->ApplyGPSHeadingInit(best->heading_rad, best->pitch_rad, kf_);
            gps_heading_applied_ = true;
            
            AINFO << "[GPS_HEADING_INIT] ✓ GPS heading initialization complete.";
            return false;
        } else {
            AWARN << "[GPS_HEADING_INIT] ✗ No valid GPS heading in cache";
        }
    }

    /// the first scan
    // GPS heading 初始化完成前，跳过所有点云处理
    // 避免在错误坐标系下添加点到地图或创建关键帧
    if (use_gps_heading_init_ && !gps_heading_applied_) {
        AINFO << "[GPS_HEADING_INIT] Waiting for GPS heading initialization (skipping scan processing)";
        return false;
    }
    
    if (flg_first_scan_) {
        AINFO << "first scan pts: " << scan_undistort_->size();

        state_point_ = kf_.GetX();
        scan_down_world_->resize(scan_undistort_->size());
        for (int i = 0; i < scan_undistort_->size(); i++) {
            PointBodyToWorld(scan_undistort_->points[i], scan_down_world_->points[i]);
        }
        
        // 仅在建图模式下添加第一帧点云到地图
        if (options_.is_in_slam_mode_) {
            ivox_->AddPoints(scan_down_world_->points);
        }

        first_lidar_time_ = measures_.lidar_end_time_;
        state_point_.timestamp_ = lidar_end_time_;
        flg_first_scan_ = false;
        return true;
    }

    if (enable_skip_lidar_) {
        skip_lidar_cnt_++;
        skip_lidar_cnt_ = skip_lidar_cnt_ % skip_lidar_num_;

        if (skip_lidar_cnt_ != 0) {
            /// 更新UI中的内容
            /*
            if (ui_) {
                ui_->UpdateNavState(kf_.GetX());
                ui_->UpdateScan(scan_undistort_, kf_.GetX().GetPose());
            }
            */

            return false;
        }
    }

    // AINFO << "LIO get cloud at beg: " << std::setprecision(14) << measures_.lidar_begin_time_
    //           << ", end: " << measures_.lidar_end_time_;

    double lidar_gap = measures_.lidar_begin_time_ - last_lidar_time_;
    double expected_gap = enable_skip_lidar_ ? (skip_lidar_num_ + 1) * 0.12 : 0.15;
    if (last_lidar_time_ > 0 && lidar_gap > expected_gap + 0.5) {
        AERROR << "检测到雷达断流，时长：" << lidar_gap;
    }

    last_lidar_time_ = measures_.lidar_begin_time_;

    flg_EKF_inited_ = (measures_.lidar_begin_time_ - first_lidar_time_) >= fasterlio::INIT_TIME;

    /// downsample
    voxel_scan_.setInputCloud(scan_undistort_);
    voxel_scan_.filter(*scan_down_body_);

    int cur_pts = scan_down_body_->size();
    if (cur_pts < 5) {
        AWARN << "Too few points, skip this scan!" << scan_undistort_->size() << ", " << scan_down_body_->size();
        return false;
    }
    scan_down_world_->resize(cur_pts);
    nearest_points_.resize(cur_pts);

    Timer::Evaluate(
        [&, this]() {
            // 成员变量预分配
            residuals_.resize(cur_pts, 0);
            point_selected_surf_.resize(cur_pts, true);
            plane_coef_.resize(cur_pts, Vec4f::Zero());
            plane_valid_.assign(cur_pts, false);  // 每帧重置

            auto old_state = kf_.GetX();

            kf_.Update(ESKF::ObsType::LIDAR, 1e-3);
            state_point_ = kf_.GetX();

            if (keep_first_imu_estimation_ && all_keyframes_.size() < 5 &&
                (old_state.rot_.inverse() * state_point_.rot_).log().norm() > 0.3 * M_PI / 180) {
                kf_.ChangeX(old_state);
                state_point_ = old_state;

                AINFO << "set state as prediction";
            }

            // AINFO << "old yaw: " << old_state.rot_.angleZ() << ", new: " << state_point_.rot_.angleZ();

            state_point_.timestamp_ = measures_.lidar_end_time_;
            euler_cur_ = state_point_.rot_;
            pos_lidar_ = state_point_.pos_ + state_point_.rot_ * state_point_.offset_t_lidar_;
        },
        "IEKF Solve and Update");

    // update local map
    Timer::Evaluate([&, this]() { MapIncremental(); }, "    Incremental Mapping");

    AINFO << "[ mapping ]: In num: " << scan_undistort_->points.size() << " down " << cur_pts
              << " Map grid num: " << ivox_->NumValidGrids() << " effect num : " << effect_feat_num_;

    /// keyframes
    if (last_kf_ == nullptr) {
        MakeKF();
    } else {
        SE3 last_pose = last_kf_->GetLIOPose();
        SE3 cur_pose = state_point_.GetPose();
        if ((last_pose.translation() - cur_pose.translation()).norm() > options_.kf_dis_th_ ||
            (last_pose.so3().inverse() * cur_pose.so3()).log().norm() > options_.kf_angle_th_) {
            MakeKF();
        } else if (!options_.is_in_slam_mode_ && (state_point_.timestamp_ - last_kf_->GetState().timestamp_) > 2.0) {
            MakeKF();
        }
    }

    /// 更新kf_for_imu
    kf_imu_ = kf_;
    if (!measures_.imu_.empty()) {
        double t = measures_.imu_.back()->timestamp;
        for (auto &imu : imu_buffer_) {
            double dt = imu->timestamp - t;
            kf_imu_.Predict(dt, p_imu_->Q_, imu->angular_velocity, imu->linear_acceleration);
            t = imu->timestamp;
        }
    }

    /*
    if (ui_) {
        ui_->UpdateScan(scan_undistort_, state_point_.GetPose());
    }
    */

    return true;
}

void LaserMapping::MakeKF() {
    // GPS heading 初始化完成前不创建关键帧
    // 严格保证：只有在 GPS heading 应用后，LIO 坐标系才与 UTM 平行
    // 避免错误坐标系的帧进入优化和地图
    if (use_gps_heading_init_ && !gps_heading_applied_) {
        AINFO << "[GPS_HEADING_INIT] Skipping keyframe creation (GPS heading not applied yet)";
        return;
    }
    
    Keyframe::Ptr kf = std::make_shared<Keyframe>(kf_id_++, scan_undistort_, state_point_);

    if (last_kf_) {
        // 直接用 LIOPose 初始化 OptPose，让优化器去更新
        // 避免基于错误的 OptPose 递推（特别是 anchor 之前的帧）
        kf->SetOptPose(kf->GetLIOPose());
    } else {
        kf->SetOptPose(kf->GetLIOPose());
    }

    kf->SetState(state_point_);

    // Set relative motion and covariance for backend optimization
    if (last_kf_) {
        SE3 last_pose = last_kf_->GetLIOPose();
        SE3 cur_pose = kf->GetLIOPose();
        SE3 relative_motion = last_pose.inverse() * cur_pose;
        kf->SetRelativeMotion(relative_motion);
    } else {
        kf->SetRelativeMotion(SE3());
    }

    // Extract 6x6 pose covariance from ESKF P matrix (23x23).
    // P layout: [pos(0:3), rot(3:6), ...].
    // Key benefit over Identity: P captures ANISOTROPY — in degenerate geometry
    // (long corridors, open areas) the uncertain direction has larger P values,
    // allowing GPS to selectively correct that axis.
    const auto& P_full = kf_.GetP();
    Mat6d cov;
    cov.block<3, 3>(0, 0) = P_full.block<3, 3>(0, 0);
    cov.block<3, 3>(0, 3) = P_full.block<3, 3>(0, 3);
    cov.block<3, 3>(3, 0) = P_full.block<3, 3>(3, 0);
    cov.block<3, 3>(3, 3) = P_full.block<3, 3>(3, 3);

    // ESKF absolute P after LiDAR update is typically very small (1e-5 ~ 1e-3).
    // For relative motion edges in the factor graph, scale by keyframe distance
    // to approximate accumulated process noise over the interval.
    if (last_kf_) {
        SE3 last_pose = last_kf_->GetLIOPose();
        SE3 cur_pose = kf->GetLIOPose();
        double dist = (cur_pose.translation() - last_pose.translation()).norm();
        double scale = std::max(dist, 1.0);
        cov *= scale;
    }

    // Floor: prevent information from becoming unreasonably large.
    // pos floor 1e-3 → max info ≈ 1000; rot floor 1e-4 → max info ≈ 10000.
    for (int d = 0; d < 3; d++) {
        cov(d, d) = std::max(cov(d, d), 1e-3);
    }
    for (int d = 3; d < 6; d++) {
        cov(d, d) = std::max(cov(d, d), 1e-4);
    }
    kf->SetCovariance(cov);

    AINFO << "LIO: create kf " << kf->GetID() << ", state: " << state_point_.pos_.transpose()
              << ", kf opt pose: " << kf->GetOptPose().translation().transpose()
              << ", lio pose: " << kf->GetLIOPose().translation().transpose() << ", time: " << std::setprecision(14)
              << state_point_.timestamp_;

    if (options_.is_in_slam_mode_) {
        all_keyframes_.emplace_back(kf);
    }

    last_kf_ = kf;
}

void LaserMapping::ProcessPointCloud2(const std::shared_ptr<apollo::drivers::PointCloud> &msg) {
    UL lock(mtx_buffer_);
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;

            CloudPtr cloud(new PointCloudType());
            preprocess_->Process(msg, cloud, ProcessMode::MAPPING);

            double lidar_begin_time = static_cast<double>(cloud->header.stamp) * 1e-9;

            // Cyber RT 线程池调度不保证 Proc 回调顺序与消息时间戳一致，
            // 因此使用预处理后的 header.stamp 做回退检测，且仅在大幅回退
            // （>1s，表示 record 重播）时清空缓冲区；小幅乱序直接丢弃。
            if (lidar_begin_time < last_timestamp_lidar_) {
                double dt = last_timestamp_lidar_ - lidar_begin_time;
                if (dt > 1.0) {
                    AWARN << "lidar loop back (" << dt << "s), clear buffer";
                    lidar_buffer_.clear();
                    time_buffer_.clear();
                    last_timestamp_lidar_ = 0;
                } else {
                    return;
                }
            }
            
            AINFO << "get cloud at " << std::setprecision(14) << lidar_begin_time
                      << ", latest imu: " << last_timestamp_imu_;
            
            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(lidar_begin_time);
            last_timestamp_lidar_ = lidar_begin_time;
        },
        "Preprocess (Standard)");
}

void LaserMapping::ProcessPointCloud2(CloudPtr cloud) {
    UL lock(mtx_buffer_);
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;

            double timestamp = math::ToSec(cloud->header.stamp);
            if (timestamp < last_timestamp_lidar_) {
                AERROR << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }

            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

bool LaserMapping::SyncPackages() {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed_) {
        measures_.scan_ = lidar_buffer_.front();
        measures_.lidar_begin_time_ = time_buffer_.front();

        if (measures_.scan_->points.size() <= 1) {
            AWARN << "Too few input point cloud!";
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else if (measures_.scan_->points.back().time / double(1000) < 0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else {
            scan_num_++;
            lidar_end_time_ = measures_.lidar_begin_time_ + measures_.scan_->points.back().time / double(1000);
            lidar_mean_scantime_ +=
                (measures_.scan_->points.back().time / double(1000) - lidar_mean_scantime_) / scan_num_;
        }

        lo::lidar_time_interval = lidar_mean_scantime_;

        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_) {
        return false;
    }

    /*** push imu_ data, and pop from imu_ buffer ***/
    double imu_time = imu_buffer_.front()->timestamp;
    measures_.imu_.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        imu_time = imu_buffer_.front()->timestamp;
        if (imu_time > lidar_end_time_) {
            break;
        }

        measures_.imu_.push_back(imu_buffer_.front());

        imu_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;

    // AINFO << "sync: " << std::setprecision(14) << measures_.lidar_begin_time_ << ", " <<
    // measures_.lidar_end_time_;

    return true;
}

void LaserMapping::MapIncremental() {
    // 定位模式下不更新地图，仅使用先验地图
    if (!options_.is_in_slam_mode_) {
        return;
    }
    
    PointVector points_to_add;
    PointVector point_no_need_downsample;

    size_t cur_pts = scan_down_body_->size();
    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);

    std::vector<size_t> index(cur_pts);
    for (size_t i = 0; i < cur_pts; ++i) {
        index[i] = i;
    }

    std::for_each(index.begin(), index.end(), [&](const size_t &i) {
        /* transform to world frame */
        PointBodyToWorld(scan_down_body_->points[i], scan_down_world_->points[i]);

        /* decide if need add to map */
        PointType &point_world = scan_down_world_->points[i];
        if (!nearest_points_[i].empty() && flg_EKF_inited_) {
            const PointVector &points_near = nearest_points_[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min_).array().floor() + 0.5) * filter_size_map_min_;

            Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

            if (fabs(dis_2_center.x()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.y()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.z()) > 0.5 * filter_size_map_min_) {
                point_no_need_downsample.emplace_back(point_world);
                return;
            }

            bool need_add = true;
            float dist = math::calc_dist(point_world.getVector3fMap(), center);
            if (points_near.size() >= fasterlio::NUM_MATCH_POINTS) {
                for (int readd_i = 0; readd_i < fasterlio::NUM_MATCH_POINTS; readd_i++) {
                    if (math::calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6) {
                        need_add = false;
                        break;
                    }
                }
            }

            if (need_add) {
                points_to_add.emplace_back(point_world);  // FIXME 这并发可能有点问题
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    });

    Timer::Evaluate(
        [&, this]() {
            ivox_->AddPoints(points_to_add);
            ivox_->AddPoints(point_no_need_downsample);
        },
        "    IVox Add Points");
}

/**
 * Lidar point cloud registration
 * will be called by the eskf custom observation model
 * compute point-to-plane residual here
 * @param s kf state
 * @param ekfom_data H matrix
 */
void LaserMapping::ObsModel(NavState &s, ESKF::CustomObservationModel &obs) {
    int cnt_pts = scan_down_body_->size();

    std::vector<size_t> index(cnt_pts);
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    Timer::Evaluate(
        [&, this]() {
            auto R_wl = (s.rot_ * s.offset_R_lidar_).cast<float>();
            // 分步计算 t_wl，避免 Sophus 表达式模板导致 Z 分量丢失
            Vec3d rot_offset = s.rot_ * s.offset_t_lidar_;
            Vec3d t_wl_d = rot_offset + s.pos_;
            Vec3f t_wl = t_wl_d.cast<float>();

            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                PointType &point_body = scan_down_body_->points[i];
                PointType &point_world = scan_down_world_->points[i];

                /* transform to world frame */
                Vec3f p_body = point_body.getVector3fMap();
                point_world.getVector3fMap() = R_wl * p_body + t_wl;
                point_world.intensity = point_body.intensity;

                // 首轮迭代: 搜索最近邻 + 拟合平面
                if (obs.iteration_ == 0) {
                    auto &points_near = nearest_points_[i];
                    points_near.clear();

                    /** Find the closest surfaces in the map **/
                    ivox_->GetClosestPoint(point_world, points_near, fasterlio::NUM_MATCH_POINTS);
                    plane_valid_[i] = points_near.size() >= fasterlio::MIN_NUM_MATCH_POINTS;
                    if (plane_valid_[i]) {
                        plane_valid_[i] =
                            math::esti_plane(plane_coef_[i], points_near, fasterlio::ESTI_PLANE_THRESHOLD);
                    }
                }

                // 所有迭代: 用缓存的平面系数重新计算残差
                point_selected_surf_[i] = false;
                if (plane_valid_[i]) {
                    auto temp = point_world.getVector4fMap();
                    temp[3] = 1.0;
                    float pd2 = plane_coef_[i].dot(temp);

                    bool valid_corr = p_body.norm() > 81 * pd2 * pd2;
                    if (valid_corr) {
                        point_selected_surf_[i] = true;
                        residuals_[i] = pd2;
                    }
                }
            });
        },
        "    ObsModel (Lidar Match)");

    effect_feat_num_ = 0;

    corr_pts_.resize(cnt_pts);
    corr_norm_.resize(cnt_pts);
    for (int i = 0; i < cnt_pts; i++) {
        if (point_selected_surf_[i]) {
            corr_norm_[effect_feat_num_] = plane_coef_[i];
            corr_pts_[effect_feat_num_] = scan_down_body_->points[i].getVector4fMap();
            corr_pts_[effect_feat_num_][3] = residuals_[i];

            effect_feat_num_++;
        }
    }
    corr_pts_.resize(effect_feat_num_);
    corr_norm_.resize(effect_feat_num_);

    if (effect_feat_num_ < 1) {
        obs.valid_ = false;
        AWARN << "No Effective Points!";
        return;
    }

    Timer::Evaluate(
        [&, this]() {
            /*** Computation of Measurement Jacobian matrix H and measurements vector ***/
            obs.h_x_ = Eigen::MatrixXd::Zero(effect_feat_num_, 12);  // 23
            obs.residual_.resize(effect_feat_num_);

            index.resize(effect_feat_num_);
            const Mat3f off_R = s.offset_R_lidar_.matrix().cast<float>();
            const Vec3f off_t = s.offset_t_lidar_.cast<float>();
            const Mat3f Rt = s.rot_.matrix().transpose().cast<float>();

            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                Vec3f point_this_be = corr_pts_[i].head<3>();
                Mat3f point_be_crossmat = math::SKEW_SYM_MATRIX(point_this_be);
                Vec3f point_this = off_R * point_this_be + off_t;
                Mat3f point_crossmat = math::SKEW_SYM_MATRIX(point_this);

                /*** get the normal vector of closest surface/corner ***/
                Vec3f norm_vec = corr_norm_[i].head<3>();

                /*** calculate the Measurement Jacobian matrix H ***/
                Vec3f C(Rt * norm_vec);
                Vec3f A(point_crossmat * C);

                if (extrinsic_est_en_) {
                    Vec3f B(point_be_crossmat * off_R.transpose() * C);
                    obs.h_x_.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], B[0], B[1],
                        B[2], C[0], C[1], C[2];
                } else {
                    obs.h_x_.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0;
                }

                /// 增加了cauchy's robust kernel
                float res = -corr_pts_[i][3];
                float rho, drho;

                const float delta = 2.0;
                const float dsqr = delta * delta;
                const float dsqr_inv = 1.0 / dsqr;

                if (res >= 0) {
                    rho = dsqr * std::log(1 + res * dsqr_inv);
                    drho = 1.0 / (1 + res * dsqr_inv);
                } else {
                    rho = -dsqr * std::log(1 - res * dsqr_inv);
                    drho = 1.0 / (1 - res * dsqr_inv);
                }

                obs.residual_(i) = rho;
                obs.h_x_.block<1, 12>(i, 0) = obs.h_x_.block<1, 12>(i, 0).eval() * drho;

                // obs.residual_(i) = res;
            });
        },
        "    ObsModel (IEKF Build Jacobian)");

    /// 填入中位数平方误差
    std::vector<double> res_sq2;
    for (size_t i = 0; i < cnt_pts; ++i) {
        if (point_selected_surf_[i]) {
            double r = residuals_[i];
            res_sq2.emplace_back(r * r);
        }
    }

    std::sort(res_sq2.begin(), res_sq2.end());
    obs.lidar_residual_mean_ = res_sq2[res_sq2.size() / 2];
    obs.lidar_residual_max_ = res_sq2[res_sq2.size() - 1];
}

///////////////////////////  private method /////////////////////////////////////////////////////////////////////

CloudPtr LaserMapping::GetGlobalMap(bool use_lio_pose, bool use_voxel, float res) {
    CloudPtr global_map(new PointCloudType);

    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(res, res, res);

    for (auto &kf : all_keyframes_) {
        CloudPtr cloud = kf->GetCloud();

        CloudPtr cloud_filter(new PointCloudType);

        if (use_voxel) {
            voxel.setInputCloud(cloud);
            voxel.filter(*cloud_filter);

        } else {
            cloud_filter = cloud;
        }

        CloudPtr cloud_trans(new PointCloudType);

        // 获取外参 T_imu_lidar (lidar坐标系到IMU坐标系的变换)
        NavState state = kf->GetState();
        Eigen::Matrix4d T_imu_lidar = Eigen::Matrix4d::Identity();
        T_imu_lidar.block<3, 3>(0, 0) = state.offset_R_lidar_.matrix();
        T_imu_lidar.block<3, 1>(0, 3) = state.offset_t_lidar_;
        
        // 获取位姿 T_world_imu
        Eigen::Matrix4d T_world_imu;
        if (use_lio_pose) {
            T_world_imu = kf->GetLIOPose().matrix();
        } else {
            T_world_imu = kf->GetOptPose().matrix();
        }
        
        // 完整变换: T_world_lidar = T_world_imu * T_imu_lidar
        // 点云在lidar坐标系下，需要先转换到IMU坐标系，再转换到世界坐标系
        Eigen::Matrix4d T_world_lidar = T_world_imu * T_imu_lidar;
        
        pcl::transformPointCloud(*cloud_filter, *cloud_trans, T_world_lidar);

        *global_map += *cloud_trans;

        AINFO << "kf " << kf->GetID() << ", pose: " << kf->GetOptPose().translation().transpose();
    }

    CloudPtr global_map_filtered(new PointCloudType);
    if (use_voxel) {
        voxel.setInputCloud(global_map);
        voxel.filter(*global_map_filtered);
    } else {
        global_map_filtered = global_map;
    }

    global_map_filtered->is_dense = false;
    global_map_filtered->height = 1;
    global_map_filtered->width = global_map_filtered->size();

    AINFO << "global map: " << global_map_filtered->size();

    return global_map_filtered;
}

CloudPtr LaserMapping::GetGlobalMapFromKeyframes(const std::vector<Keyframe::Ptr>& keyframes, 
                                                  bool use_voxel, float res) {
    CloudPtr global_map(new PointCloudType);

    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(res, res, res);

    for (auto &kf : keyframes) {
        CloudPtr cloud = kf->GetCloud();

        CloudPtr cloud_filter(new PointCloudType);

        if (use_voxel) {
            voxel.setInputCloud(cloud);
            voxel.filter(*cloud_filter);
        } else {
            cloud_filter = cloud;
        }

        CloudPtr cloud_trans(new PointCloudType);

        // 获取外参 T_imu_lidar (lidar坐标系到IMU坐标系的变换)
        NavState state = kf->GetState();
        Eigen::Matrix4d T_imu_lidar = Eigen::Matrix4d::Identity();
        T_imu_lidar.block<3, 3>(0, 0) = state.offset_R_lidar_.matrix();
        T_imu_lidar.block<3, 1>(0, 3) = state.offset_t_lidar_;
        
        // 使用优化后的位姿 (OptPose)
        Eigen::Matrix4d T_world_imu = kf->GetOptPose().matrix();
        
        // 完整变换: T_world_lidar = T_world_imu * T_imu_lidar
        Eigen::Matrix4d T_world_lidar = T_world_imu * T_imu_lidar;
        
        pcl::transformPointCloud(*cloud_filter, *cloud_trans, T_world_lidar);

        *global_map += *cloud_trans;

        AINFO << "kf " << kf->GetID() << ", OptPose: " << kf->GetOptPose().translation().transpose();
    }

    CloudPtr global_map_filtered(new PointCloudType);
    if (use_voxel) {
        voxel.setInputCloud(global_map);
        voxel.filter(*global_map_filtered);
    } else {
        global_map_filtered = global_map;
    }

    global_map_filtered->is_dense = false;
    global_map_filtered->height = 1;
    global_map_filtered->width = global_map_filtered->size();

    AINFO << "global map from optimized keyframes: " << global_map_filtered->size();

    return global_map_filtered;
}

CloudPtr LaserMapping::GetGlobalMapFromKeyframesAndPoses(const std::vector<Keyframe::Ptr>& keyframes,
                                                         const std::vector<SE3>& poses,
                                                         bool use_voxel, float res) {
    CloudPtr global_map(new PointCloudType);

    if (keyframes.size() != poses.size()) {
        AWARN << "GetGlobalMapFromKeyframesAndPoses size mismatch: keyframes=" << keyframes.size()
              << ", poses=" << poses.size();
        return global_map;
    }

    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(res, res, res);

    for (size_t i = 0; i < keyframes.size(); ++i) {
        auto& kf = keyframes[i];
        if (!kf) {
            continue;
        }

        CloudPtr cloud = kf->GetCloud();
        CloudPtr cloud_filter(new PointCloudType);

        if (use_voxel) {
            voxel.setInputCloud(cloud);
            voxel.filter(*cloud_filter);
        } else {
            cloud_filter = cloud;
        }

        CloudPtr cloud_trans(new PointCloudType);

        NavState state = kf->GetState();
        Eigen::Matrix4d T_imu_lidar = Eigen::Matrix4d::Identity();
        T_imu_lidar.block<3, 3>(0, 0) = state.offset_R_lidar_.matrix();
        T_imu_lidar.block<3, 1>(0, 3) = state.offset_t_lidar_;

        Eigen::Matrix4d T_world_imu = poses[i].matrix();
        Eigen::Matrix4d T_world_lidar = T_world_imu * T_imu_lidar;

        pcl::transformPointCloud(*cloud_filter, *cloud_trans, T_world_lidar);
        *global_map += *cloud_trans;
    }

    CloudPtr global_map_filtered(new PointCloudType);
    if (use_voxel) {
        voxel.setInputCloud(global_map);
        voxel.filter(*global_map_filtered);
    } else {
        global_map_filtered = global_map;
    }

    global_map_filtered->is_dense = false;
    global_map_filtered->height = 1;
    global_map_filtered->width = global_map_filtered->size();

    AINFO << "global map from keyframes and supplied poses: " << global_map_filtered->size();
    return global_map_filtered;
}

void LaserMapping::SaveMap() {
    /// 保存地图
    auto global_map = GetGlobalMap(true);

    pcl::io::savePCDFileBinaryCompressed("./data/lio.pcd", *global_map);

    AINFO << "lio map is saved to ./data/lio.pcd";
}

CloudPtr LaserMapping::GetRecentCloud() {
    if (lidar_buffer_.empty()) {
        return nullptr;
    }

    return lidar_buffer_.front();
}

void LaserMapping::ProcessHeadingForInit(const HeadingObservation &heading) {
    if (!use_gps_heading_init_) {
        return;
    }
    
    if (!heading.is_valid) {
        AWARN << "[GPS_HEADING_INIT] Received invalid GPS heading data (timestamp: " 
              << heading.timestamp << ", heading: " << heading.heading << " deg, std_dev: " 
              << heading.heading_std_dev << " deg). Skipping.";
        return;
    }
    
    std::lock_guard<std::mutex> lock(mtx_heading_);
    
    HeadingCache cache;
    cache.timestamp = heading.timestamp;
    cache.heading_rad = heading.GetYawRad();
    cache.pitch_rad = heading.pitch * M_PI / 180.0;
    cache.valid = true;
    
    heading_cache_.push_back(cache);
    
    if (heading_cache_.size() > MAX_HEADING_CACHE_SIZE) {
        heading_cache_.pop_front();
    }
    
    AINFO << "[GPS_HEADING_INIT] ✓ Cached GPS heading: " << heading.heading << " deg → yaw=" 
          << (cache.heading_rad * 180.0 / M_PI) << " deg"
          << " | Satellites: " << heading.satellite_tracked;
}

void LaserMapping::SetPriorMap(CloudPtr map_cloud) {
    if (!map_cloud || map_cloud->empty()) {
        AWARN << "[LaserMapping] Empty prior map provided";
        return;
    }
    
    // 清空当前 IVox
    ClearMap();
    
    // 重新创建 IVox 并加载先验地图
    ivox_ = std::make_shared<IVoxType>(ivox_options_);
    
    // 将地图点云添加到 IVox
    PointVector points_to_add;
    points_to_add.reserve(map_cloud->size());
    
    for (const auto& pt : map_cloud->points) {
        // 过滤无效点
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }
        points_to_add.push_back(pt);
    }
    
    ivox_->AddPoints(points_to_add);
    
    AINFO << "[LaserMapping] Prior map loaded to IVox: " << points_to_add.size() << " points";
}

void LaserMapping::ClearMap() {
    if (ivox_) {
        // 重置 IVox
        ivox_.reset();
        AINFO << "[LaserMapping] IVox map cleared";
    }
}

void LaserMapping::SetInitialPose(const SE3& pose) {
    // 设置 ESKF 的初始状态
    NavState init_state;
    init_state.pos_ = pose.translation();
    init_state.rot_ = pose.so3();
    init_state.vel_ = Vec3d::Zero();
    init_state.bg_ = Vec3d::Zero();
    init_state.ba_ = Vec3d::Zero();
    init_state.pose_is_ok_ = true;
    
    // 设置重力方向（在世界坐标系下指向 -Z）
    init_state.grav_ = S2(Vec3d(0, 0, -1.0) * G_m_s2);
    
    // 保留外参（从配置加载的）
    init_state.offset_R_lidar_ = state_point_.offset_R_lidar_;
    init_state.offset_t_lidar_ = state_point_.offset_t_lidar_;
    
    // 设置到 ESKF
    kf_.ChangeX(init_state);
    state_point_ = init_state;
    
    // 初始化协方差
    auto init_P = kf_.GetP();
    init_P.setIdentity();
    init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;   // vel
    init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;  // bg
    init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001; // ba
    init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;  // grav
    init_P(21, 21) = init_P(22, 22) = 0.00001;  // ext_R
    kf_.ChangeP(init_P);
    
    AINFO << "[LaserMapping] Initial pose set: pos=(" << pose.translation().transpose() 
          << "), yaw=" << (pose.so3().log()[2] * 180.0 / M_PI) << " deg";
}

void LaserMapping::ApplyGPSHeadingForLocalization(double yaw_rad) {
    // 调用 ImuProcess 的 GPS Heading 初始化
    // 这会保留 IMU 初始化的 roll/pitch（重力对齐），只更新 yaw
    if (!p_imu_ || !p_imu_->IsIMUInited()) {
        AWARN << "[LaserMapping] Cannot apply GPS heading: IMU not initialized";
        return;
    }
    
    // pitch 设为 0（定位时假设地面平坦）
    p_imu_->ApplyGPSHeadingInit(yaw_rad, 0.0, kf_);
    
    // 同步状态
    state_point_ = kf_.GetX();
    gps_heading_applied_ = true;
    
    AINFO << "[LaserMapping] GPS heading applied for localization: yaw=" 
          << (yaw_rad * 180.0 / M_PI) << " deg";
}

void LaserMapping::SetPosition(const Vec3d& pos) {
    // 获取当前状态
    auto current_state = kf_.GetX();
    
    // 只更新位置，保留姿态和其他状态
    current_state.pos_ = pos;
    current_state.vel_ = Vec3d::Zero();  // 定位初始化时速度为 0
    
    // 写回 ESKF
    kf_.ChangeX(current_state);
    state_point_ = current_state;
    
    AINFO << "[LaserMapping] Position set: (" << pos.transpose() << ")";
}

}  // namespace lightning
