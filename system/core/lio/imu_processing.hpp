#pragma once

#ifndef FASTER_LIO_IMU_PROCESSING_H
#define FASTER_LIO_IMU_PROCESSING_H

#include "cyber/common/log.h"
#include <algorithm>
#include <cmath>
#include <deque>
#include <execution>
#include <fstream>
#include <numeric>

#include "common/eigen_types.h"
#include "common/measure_group.h"
#include "common/point_def.h"
#include "core/lio/eskf.hpp"
#include "core/lio/pose6d.h"
#include "utils/timer.h"

namespace lightning {

/// IMU处理类
class ImuProcess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();
    ~ImuProcess();

    void Reset();
    void SetExtrinsic(const Vec3d &transl, const Mat3d &rot);
    void SetGyrCov(const Vec3d &scaler);
    void SetAccCov(const Vec3d &scaler);
    void SetGyrBiasCov(const Vec3d &b_g);
    void SetAccBiasCov(const Vec3d &b_a);

    void Process(const MeasureGroup &meas, ESKF &kf_state, CloudPtr &scan);

    bool IsIMUInited() const { return imu_need_init_ == false; }

    /// 应用GPS heading进行初始化旋转
    /// @param gps_heading_rad GPS航向角(弧度), Apollo规定: 0=东, π/2=北, 逆时针正
    /// @param gps_pitch_rad GPS pitch角(弧度)
    /// @param kf_state ESKF状态
    void ApplyGPSHeadingInit(double gps_heading_rad, double gps_pitch_rad, ESKF &kf_state);

    double GetMeanAccNorm() const { return mean_acc_.norm(); }

    Eigen::Matrix<double, 12, 12> Q_;
    Vec3d cov_acc_;
    Vec3d cov_gyr_;
    Vec3d cov_acc_scale_;
    Vec3d cov_gyr_scale_;
    Vec3d cov_bias_gyr_;
    Vec3d cov_bias_acc_;

   private:
    void IMUInit(const MeasureGroup &meas, ESKF &kf_state, int &N);
    void UndistortPcl(const MeasureGroup &meas, ESKF &kf_state, CloudPtr &pcl_out);

    static inline constexpr int min_init_count_ = 20;
    static inline constexpr double min_init_duration_s_ = 1.0;

    PointCloudType::Ptr cur_pcl_un_ = nullptr;
    lightning::IMUPtr last_imu_ = nullptr;
    std::deque<lightning::IMUPtr> imu_queue_;

    std::vector<Pose6D> imu_pose_;
    Mat3d R_lidar_imu_ = Mat3d ::Identity();
    Vec3d t_lidar_mu_ = Vec3d ::Zero();
    Vec3d mean_acc_ = Vec3d::Zero();
    Vec3d mean_gyr_ = Vec3d::Zero();
    Vec3d mean_euler_angles_ = Vec3d::Zero();
    Vec3d angvel_last_ = Vec3d ::Zero();
    Vec3d acc_s_last_ = Vec3d ::Zero();

    double last_lidar_end_time_ = 0;
    double init_start_time_ = -1.0;
    int init_iter_num_ = 1;
    int init_euler_count_ = 0;
    bool b_first_frame_ = true;
    bool imu_need_init_ = true;
};

inline ImuProcess::ImuProcess() : b_first_frame_(true), imu_need_init_(true) {
    init_iter_num_ = 1;
    Q_.setZero();
    Q_.diagonal() << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5;
    cov_acc_ = Vec3d(0.1, 0.1, 0.1);
    cov_gyr_ = Vec3d(0.1, 0.1, 0.1);
    cov_bias_gyr_ = Vec3d(0.0001, 0.0001, 0.0001);
    cov_bias_acc_ = Vec3d(0.0001, 0.0001, 0.0001);
    mean_acc_ = Vec3d(0, 0, -1.0);
    mean_gyr_ = Vec3d(0, 0, 0);
    last_imu_.reset(new lightning::IMU());
}

inline ImuProcess::~ImuProcess() {}

inline void ImuProcess::Reset() {
    mean_acc_ = Vec3d(0, 0, -1.0);
    mean_gyr_ = Vec3d(0, 0, 0);
    mean_euler_angles_.setZero();
    cov_acc_ = Vec3d(0.0, 0.0, 0.0);
    cov_gyr_ = Vec3d(0.0, 0.0, 0.0);
    angvel_last_.setZero();

    imu_need_init_ = true;
    init_iter_num_ = 0;
    init_euler_count_ = 0;
    init_start_time_ = -1.0;
    imu_queue_.clear();
    imu_pose_.clear();
    last_imu_.reset(new lightning::IMU());
    cur_pcl_un_.reset(new PointCloudType());
}

inline void ImuProcess::SetExtrinsic(const Vec3d &transl, const Mat3d &rot) {
    t_lidar_mu_ = transl;
    // Normalize rotation matrix to ensure orthogonality (numerical solution may have errors)
    Eigen::Quaterniond quat(rot);
    quat.normalize();
    R_lidar_imu_ = quat.toRotationMatrix();
}

inline void ImuProcess::SetGyrCov(const Vec3d &scaler) { cov_gyr_scale_ = scaler; }

inline void ImuProcess::SetAccCov(const Vec3d &scaler) { cov_acc_scale_ = scaler; }

inline void ImuProcess::SetGyrBiasCov(const Vec3d &b_g) { cov_bias_gyr_ = b_g; }

inline void ImuProcess::SetAccBiasCov(const Vec3d &b_a) { cov_bias_acc_ = b_a; }

inline void ImuProcess::IMUInit(const MeasureGroup &meas, ESKF &kf_state, int &N) {
    /** 1. initializing the gravity_, gyro bias, acc and gyro covariance
     ** 2. normalize the acceleration measurenments to unit gravity_ **/

    Vec3d cur_acc, cur_gyr;

    if (b_first_frame_) {
        Reset();
        N = 0;
        b_first_frame_ = false;
    }

    for (const auto &imu : meas.imu_) {
        const auto &imu_acc = imu->linear_acceleration;
        const auto &gyr_acc = imu->angular_velocity;
        cur_acc = imu_acc;
        cur_gyr = gyr_acc;

        if (N == 0 || init_start_time_ < 0.0) {
            mean_acc_ = cur_acc;
            mean_gyr_ = cur_gyr;
            cov_acc_.setZero();
            cov_gyr_.setZero();
            if (imu->has_euler_angles) {
                mean_euler_angles_ = imu->euler_angles;
                init_euler_count_ = 1;
            } else {
                mean_euler_angles_.setZero();
                init_euler_count_ = 0;
            }
            init_start_time_ = imu->timestamp;
            N = 1;
            continue;
        }

        const int next_n = N + 1;
        mean_acc_ += (cur_acc - mean_acc_) / next_n;
        mean_gyr_ += (cur_gyr - mean_gyr_) / next_n;

        cov_acc_ = cov_acc_ * (N - 1.0) / N +
                   (cur_acc - mean_acc_).cwiseProduct(cur_acc - mean_acc_) * (N - 1.0) / (N * N);
        cov_gyr_ = cov_gyr_ * (N - 1.0) / N +
                   (cur_gyr - mean_gyr_).cwiseProduct(cur_gyr - mean_gyr_) * (N - 1.0) / (N * N);

        if (imu->has_euler_angles) {
            const int next_euler_n = init_euler_count_ + 1;
            mean_euler_angles_ += (imu->euler_angles - mean_euler_angles_) / next_euler_n;
            init_euler_count_ = next_euler_n;
        }

        N = next_n;
    }

    if (N == 0 || init_start_time_ < 0.0) {
        AINFO << "[IMU_INIT] Waiting for initial IMU samples...";
        return;
    }

    auto init_state = kf_state.GetX();
    init_state.timestamp_ = meas.imu_.back()->timestamp;
    
    init_state.grav_ = S2(Vec3d(0, 0, -G_m_s2));

    Vec3d acc_up = mean_acc_.normalized();
    Vec3d world_up(0, 0, 1);
    Eigen::Quaterniond q_align = Eigen::Quaterniond::FromTwoVectors(acc_up, world_up);
    q_align.normalize();
    init_state.rot_ = SO3(q_align);

    const Mat3d R_init = q_align.toRotationMatrix();
    const double gravity_pitch = std::asin(std::max(-1.0, std::min(1.0, -R_init(2, 0))));
    const double gravity_roll = std::atan2(R_init(2, 1), R_init(2, 2));
    const double gravity_yaw = std::atan2(R_init(1, 0), R_init(0, 0));
    const double tilt_deg = std::acos(std::min(acc_up.dot(world_up), 1.0)) * 180.0 / M_PI;

    AINFO << "[IMU_INIT] Gravity alignment applied:";
    AINFO << "[IMU_INIT]   mean_acc (IMU):  " << mean_acc_.transpose() << " (norm: " << mean_acc_.norm() << ")";
    AINFO << "[IMU_INIT]   acc_up (IMU):    " << acc_up.transpose();
    AINFO << "[IMU_INIT]   R_align (quat):  " << q_align.coeffs().transpose();
    AINFO << "[IMU_INIT]   gravity_roll:    " << (gravity_roll * 180.0 / M_PI) << " deg";
    AINFO << "[IMU_INIT]   gravity_pitch:   " << (gravity_pitch * 180.0 / M_PI) << " deg";
    AINFO << "[IMU_INIT]   gravity_yaw:     " << (gravity_yaw * 180.0 / M_PI) << " deg";
    AINFO << "[IMU_INIT]   Initial tilt:    " << tilt_deg << " deg (road crown / slope)";

    if (init_euler_count_ > 0) {
        const double euler_roll = mean_euler_angles_.x();
        const double euler_pitch = mean_euler_angles_.y();
        const double euler_yaw = mean_euler_angles_.z();

        AINFO << "[IMU_INIT] corrected_imu euler_angles comparison (not used for init):";
        AINFO << "[IMU_INIT]   mean_euler(roll,pitch,yaw): " << mean_euler_angles_.transpose();
        AINFO << "[IMU_INIT]   euler_roll:     " << (euler_roll * 180.0 / M_PI) << " deg";
        AINFO << "[IMU_INIT]   euler_pitch:    " << (euler_pitch * 180.0 / M_PI) << " deg";
        AINFO << "[IMU_INIT]   euler_yaw:      " << (euler_yaw * 180.0 / M_PI) << " deg";
        AINFO << "[IMU_INIT]   delta_roll:     " << ((gravity_roll - euler_roll) * 180.0 / M_PI) << " deg";
        AINFO << "[IMU_INIT]   delta_pitch:    " << ((gravity_pitch - euler_pitch) * 180.0 / M_PI) << " deg";
    }
    
    init_state.bg_ = mean_gyr_;
    init_state.offset_t_lidar_ = t_lidar_mu_;
    Eigen::Quaterniond lidar_quat(R_lidar_imu_);
    lidar_quat.normalize();
    init_state.offset_R_lidar_ = SO3(lidar_quat);
    kf_state.ChangeX(init_state);

    auto init_P = kf_state.GetP();
    init_P.setIdentity();
    init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
    init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
    init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
    init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
    init_P(21, 21) = init_P(22, 22) = 0.00001;
    kf_state.ChangeP(init_P);

    last_imu_ = meas.imu_.back();
}

inline void ImuProcess::UndistortPcl(const MeasureGroup &meas, ESKF &kf_state, CloudPtr &pcl_out) {
    /*** add the imu_ of the last frame-tail to the of current frame-head ***/
    auto v_imu = meas.imu_;
    v_imu.push_front(last_imu_);
    const double &imu_end_time = v_imu.back()->timestamp;

    const double &pcl_beg_time = meas.lidar_begin_time_;
    const double &pcl_end_time = meas.lidar_end_time_;

    /*** Initialize IMU pose ***/
    auto imu_state = kf_state.GetX();
    imu_pose_.clear();
    imu_pose_.emplace_back(0.0, acc_s_last_, angvel_last_, imu_state.vel_, imu_state.pos_, imu_state.rot_.matrix());

    /*** forward propagation at each imu_ point ***/
    Vec3d angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    Mat3d R_imu;

    double dt = 0;
    Vec3d acc = Vec3d::Zero();
    Vec3d gyro = Vec3d::Zero();

    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
        auto &&head = *(it_imu);
        auto &&tail = *(it_imu + 1);

        if (tail->timestamp < last_lidar_end_time_) {
            continue;
        }

        angvel_avr = .5 * (head->angular_velocity + tail->angular_velocity);
        acc_avr = .5 * (head->linear_acceleration + tail->linear_acceleration);

        acc_avr = acc_avr * G_m_s2 / mean_acc_.norm();  // - state_inout.ba;

        if (head->timestamp < last_lidar_end_time_) {
            dt = tail->timestamp - last_lidar_end_time_;
        } else {
            dt = tail->timestamp - head->timestamp;
        }

        acc = acc_avr;
        gyro = angvel_avr;

        if (dt > 0.1) {
            AERROR << "get abnormal dt: " << dt;
            kf_state.SetTime((*it_imu)->timestamp);
            break;
        }

        Q_.block<3, 3>(0, 0).diagonal() = cov_gyr_;
        Q_.block<3, 3>(3, 3).diagonal() = cov_acc_;
        Q_.block<3, 3>(6, 6).diagonal() = cov_bias_gyr_;
        Q_.block<3, 3>(9, 9).diagonal() = cov_bias_acc_;
        kf_state.Predict(dt, Q_, gyro, acc);

        // AINFO << "gyro: " << gyro.transpose() << ", dt: " << dt;

        // AINFO << "acc: " << acc.transpose() << " grav: " << kf_state.GetX().grav_.vec_.norm()
        //           << ", vel: " << kf_state.GetX().vel_.transpose() << ", dt: " << dt;

        /* save the poses at each IMU measurements */
        imu_state = kf_state.GetX();
        angvel_last_ = angvel_avr - imu_state.bg_;
        acc_s_last_ = imu_state.rot_ * (acc_avr - imu_state.ba_);
        for (int i = 0; i < 3; i++) {
            acc_s_last_[i] += imu_state.grav_[i];
        }

        double &&offs_t = tail->timestamp - pcl_beg_time;
        imu_pose_.emplace_back(
            Pose6D(offs_t, acc_s_last_, angvel_last_, imu_state.vel_, imu_state.pos_, imu_state.rot_.matrix()));
    }

    /*** calculated the pos and attitude prediction at the frame-end ***/
    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
    dt = note * (pcl_end_time - imu_end_time);
    kf_state.Predict(dt, Q_, gyro, acc);

    imu_state = kf_state.GetX();
    last_imu_ = meas.imu_.back();
    last_lidar_end_time_ = pcl_end_time;

    /*** sort point clouds by offset time (parallel) ***/
    pcl_out = meas.scan_;
    std::sort(std::execution::par_unseq,
              pcl_out->points.begin(), pcl_out->points.end(),
              [](const PointType &p1, const PointType &p2) { return p1.time < p2.time; });

    /*** undistort each lidar point (parallel) ***/
    if (pcl_out->empty() || imu_pose_.size() < 2) {
        return;
    }

    // Pre-compute constant matrices for end-frame transform
    const Mat3d R_end_inv = imu_state.rot_.inverse().matrix();
    const Mat3d R_lidar_mat = imu_state.offset_R_lidar_.matrix();
    const Mat3d R_lidar_inv = R_lidar_mat.transpose();
    const Vec3d t_lidar_vec = imu_state.offset_t_lidar_;
    const Vec3d state_pos_end = imu_state.pos_;

    // Build offset_time array for binary search
    const size_t n_poses = imu_pose_.size();
    std::vector<double> pose_times(n_poses);
    for (size_t k = 0; k < n_poses; k++) {
        pose_times[k] = imu_pose_[k].offset_time;
    }

    // Parallel undistortion: each point independently finds its IMU interval
    const size_t n_pts = pcl_out->size();
    std::vector<size_t> pt_indices(n_pts);
    std::iota(pt_indices.begin(), pt_indices.end(), 0);

    std::for_each(std::execution::par_unseq, pt_indices.begin(), pt_indices.end(), [&](size_t i) {
        auto& pt = pcl_out->points[i];
        const double pt_offset = pt.time / 1000.0;

        // Find interval: largest k where pose_times[k] <= pt_offset
        auto it = std::upper_bound(pose_times.begin(), pose_times.end(), pt_offset);
        size_t k = 0;
        if (it != pose_times.begin()) {
            k = static_cast<size_t>(std::distance(pose_times.begin(), it)) - 1;
        }
        if (k + 1 >= n_poses) k = n_poses - 2;

        const auto& head = imu_pose_[k];
        const auto& tail = imu_pose_[k + 1];
        const double dt_pt = pt_offset - head.offset_time;

        Mat3d R_i = head.rot * math::exp(tail.gyr, dt_pt).matrix();
        Vec3d P_i(pt.x, pt.y, pt.z);
        Vec3d T_ei = head.pos + head.vel * dt_pt + 0.5 * tail.acc * dt_pt * dt_pt - state_pos_end;

        Vec3d p_compensate = R_lidar_inv *
            (R_end_inv * (R_i * (R_lidar_mat * P_i + t_lidar_vec) + T_ei) - t_lidar_vec);

        pt.x = p_compensate(0);
        pt.y = p_compensate(1);
        pt.z = p_compensate(2);
    });
}

inline void ImuProcess::Process(const MeasureGroup &meas, ESKF &kf_state, CloudPtr &scan) {
    if (meas.imu_.empty()) {
        return;
    }

    if (imu_need_init_) {
        /// The very first lidar frame
        IMUInit(meas, kf_state, init_iter_num_);

        imu_need_init_ = true;

        last_imu_ = meas.imu_.back();

        auto imu_state = kf_state.GetX();
        double init_elapsed = 0.0;
        if (init_start_time_ > 0.0) {
            init_elapsed = std::max(0.0, meas.imu_.back()->timestamp - init_start_time_);
        }

        if (init_iter_num_ >= min_init_count_ && init_elapsed >= min_init_duration_s_) {
            cov_acc_ *= pow(G_m_s2 / mean_acc_.norm(), 2);
            imu_need_init_ = false;

            cov_acc_ = cov_acc_scale_;
            cov_gyr_ = cov_gyr_scale_;
            AINFO << "[IMU_INIT] ✓ IMU initialization complete (" << init_iter_num_
                  << " samples, " << init_elapsed << " s)";
            AINFO << "[IMU_INIT]   bg (gyro bias): " << imu_state.bg_.transpose();
            AINFO << "[IMU_INIT]   ba (accel bias): " << imu_state.ba_.transpose();
            AINFO << "[IMU_INIT]   Gravity direction: " << imu_state.grav_.vec_.transpose();
            AINFO << "[IMU_INIT]   Mean acceleration: " << mean_acc_.transpose() << " (norm: " << mean_acc_.norm() << ")";
            AINFO << "[IMU_INIT]   Initial rotation (quat): " << imu_state.rot_.unit_quaternion().coeffs().transpose();
            AINFO << "[IMU_INIT] Ready for GPS heading initialization (if enabled)";
        } else {
            AINFO << "[IMU_INIT] Waiting for IMU initialization... samples=" << init_iter_num_
                  << " elapsed=" << init_elapsed << " s / " << min_init_duration_s_ << " s";
        }

        return;
    }

    Timer::Evaluate([&, this]() { UndistortPcl(meas, kf_state, scan); }, "Undistort Pcl");
}

inline void ImuProcess::ApplyGPSHeadingInit(double gps_heading_rad, double gps_pitch_rad, ESKF &kf_state) {
    AINFO << "[GPS_HEADING_INIT] ========================================";
    AINFO << "[GPS_HEADING_INIT] Applying GPS heading and pitch to LIO coordinate frame";
    AINFO << "[GPS_HEADING_INIT] GPS Heading: " << (gps_heading_rad * 180.0 / M_PI) << " deg (" 
          << gps_heading_rad << " rad)";
    AINFO << "[GPS_HEADING_INIT] GPS Pitch: " << (gps_pitch_rad * 180.0 / M_PI) << " deg (" 
          << gps_pitch_rad << " rad)";
    
    // GPS Heading convention (Apollo 规定, 当前 host 驱动已适配):
    //   - 0 rad = East (ENU +X axis)
    //   - π/2 rad = North (ENU +Y axis)
    //   - 逆时针为正, 从 East 起算
    //   - gps_heading_rad 即为标准 ENU yaw 角
    //
    // Target: Rotate LIO frame to align with UTM/ENU frame
    //
    // Step 2: Get current IMU state
    auto current_state = kf_state.GetX();
    
    AINFO << "[GPS_HEADING_INIT] Current state before modification:";
    AINFO << "[GPS_HEADING_INIT]   Position: " << current_state.pos_.transpose();
    AINFO << "[GPS_HEADING_INIT]   Rotation (quat): " << current_state.rot_.unit_quaternion().coeffs().transpose();
    
    // Extract roll/pitch from current rotation using proper ZYX Euler decomposition.
    // (SO3::log() returns rotation vector, NOT Euler angles — using it as Euler is wrong
    //  for large angles like GPS heading.)
    Mat3d R = current_state.rot_.matrix();
    double pitch = std::asin(std::max(-1.0, std::min(1.0, -R(2, 0))));
    double roll  = std::atan2(R(2, 1), R(2, 2));
    double yaw_old = std::atan2(R(1, 0), R(0, 0));
    
    AINFO << "[GPS_HEADING_INIT] ZYX Euler decomposition:";
    AINFO << "[GPS_HEADING_INIT]   Roll:      " << (roll * 180.0 / M_PI) << " deg";
    AINFO << "[GPS_HEADING_INIT]   Pitch:     " << (pitch * 180.0 / M_PI) << " deg";
    AINFO << "[GPS_HEADING_INIT]   Yaw (old): " << (yaw_old * 180.0 / M_PI) << " deg";
    AINFO << "[GPS_HEADING_INIT]   Yaw (new): " << (gps_heading_rad * 180.0 / M_PI) << " deg";
    
    // Reconstruct rotation: Rz(gps_yaw) * Ry(pitch) * Rx(roll)
    // Preserves gravity-aligned roll/pitch, replaces yaw with GPS heading.
    SO3 Rx = SO3::exp(Vec3d(roll, 0, 0));
    SO3 Ry = SO3::exp(Vec3d(0, pitch, 0));
    SO3 Rz = SO3::exp(Vec3d(0, 0, gps_heading_rad));
    SO3 new_rot = Rz * Ry * Rx;
    
    // Rotate pos and vel accumulated during the wait period (t_imu_init → t_gps_heading).
    // Without this, position/velocity are in the old (pre-heading) coordinate frame.
    SO3 R_delta = new_rot * current_state.rot_.inverse();
    current_state.pos_ = R_delta * current_state.pos_;
    current_state.vel_ = R_delta * current_state.vel_;
    current_state.rot_ = new_rot;
    
    AINFO << "[GPS_HEADING_INIT] Updated state after modification:";
    AINFO << "[GPS_HEADING_INIT]   Position: " << current_state.pos_.transpose() << " (rotated)";
    AINFO << "[GPS_HEADING_INIT]   Velocity: " << current_state.vel_.transpose() << " (rotated)";
    AINFO << "[GPS_HEADING_INIT]   Rotation (quat): " << current_state.rot_.unit_quaternion().coeffs().transpose();
    
    // Update state and reset covariance to be consistent with the new frame.
    kf_state.ChangeX(current_state);
    
    auto P = kf_state.GetP();
    P.setIdentity();
    P(6, 6) = P(7, 7) = P(8, 8) = 0.00001;    // vel
    P(9, 9) = P(10, 10) = P(11, 11) = 0.00001;  // bg
    P(12, 12) = P(13, 13) = P(14, 14) = 0.00001; // ba
    kf_state.ChangeP(P);
    
    AINFO << "[GPS_HEADING_INIT] ✓ GPS heading and pitch applied successfully";
    AINFO << "[GPS_HEADING_INIT]   LIO frame is now aligned with UTM/ENU frame";
    AINFO << "[GPS_HEADING_INIT]   - LIO X-axis → UTM East";
    AINFO << "[GPS_HEADING_INIT]   - LIO Y-axis → UTM North";
    AINFO << "[GPS_HEADING_INIT]   - LIO Z-axis → UTM Up";
    AINFO << "[GPS_HEADING_INIT] ========================================";
}

}  // namespace lightning

#endif
