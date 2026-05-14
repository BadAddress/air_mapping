#ifndef LIGHTNING_COMMON_MATH_UTILS_H
#define LIGHTNING_COMMON_MATH_UTILS_H

#include <cmath>
#include <numeric>
#include <utility>
#include <vector>
#include <boost/array.hpp>
#include <boost/math/tools/precision.hpp>

#include "common/eigen_types.h"

namespace lightning::math {

template <typename T>
inline Eigen::Matrix<T, 3, 3> SKEW_SYM_MATRIX(const Eigen::Matrix<T, 3, 1>& v) {
    Eigen::Matrix<T, 3, 3> m;
    m << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
    return m;
}

template <typename T>
inline Eigen::Matrix<T, 3, 3> SKEW_SYM_MATRIX(const T& v1, const T& v2, const T& v3) {
    Eigen::Matrix<T, 3, 3> m;
    m << 0.0, -v3, v2, v3, 0.0, -v1, -v2, v1, 0.0;
    return m;
}

template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1>&& ang) {
    T ang_norm = ang.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (ang_norm > 0.0000001) {
        Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
        Eigen::Matrix<T, 3, 3> K;
        K = SKEW_SYM_MATRIX(r_axis);

        return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
    } else {
        return Eye3;
    }
}

template <typename T, typename Ts>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1>& ang_vel, const Ts& dt) {
    T ang_vel_norm = ang_vel.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

    if (ang_vel_norm > 0.0000001) {
        Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
        Eigen::Matrix<T, 3, 3> K;

        K = SKEW_SYM_MATRIX(r_axis);

        T r_ang = ang_vel_norm * dt;

        /// Roderigous Tranformation
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    } else {
        return Eye3;
    }
}

template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const T& v1, const T& v2, const T& v3) {
    T&& norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (norm > 0.00001) {
        Eigen::Matrix<T, 3, 3> K;
        K = SKEW_SYM_MATRIX(v1 / norm, v2 / norm, v3 / norm);

        /// Roderigous Tranformation
        return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
    } else {
        return Eye3;
    }
}

/* Logrithm of a Rotation Matrix */
template <typename T>
Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3>& R) {
    T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
    Eigen::Matrix<T, 3, 1> K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
    return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

template <typename T>
Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3>& rot) {
    T sy = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
    bool singular = sy < 1e-6;
    T x, y, z;
    if (!singular) {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);
        z = atan2(rot(1, 0), rot(0, 0));
    } else {
        x = atan2(-rot(1, 2), rot(1, 1));
        y = atan2(-rot(2, 0), sy);
        z = 0;
    }
    Eigen::Matrix<T, 3, 1> ang(x, y, z);
    return ang;
}

/// xt: 经验证，此转换与ros中tf::createQuaternionFromRPY结果一致
template <typename T>
Eigen::Matrix<T, 3, 3> RpyToRotM2(const T r, const T p, const T y) {
    using AA = Eigen::AngleAxis<T>;
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    return Eigen::Matrix<T, 3, 3>(AA(y, Vec3::UnitZ()) * AA(p, Vec3::UnitY()) * AA(r, Vec3::UnitX()));
}

template <typename S>
inline Eigen::Matrix<S, 3, 1> VecFromArray(const std::vector<double>& v) {
    return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
}

template <typename S>
inline Eigen::Matrix<S, 3, 1> VecFromArray(const boost::array<S, 3>& v) {
    return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
}

template <typename S>
inline Eigen::Matrix<S, 3, 3> MatFromArray(const std::vector<double>& v) {
    Eigen::Matrix<S, 3, 3> m;
    m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    return m;
}

template <typename S>
inline Eigen::Matrix<S, 3, 3> MatFromArray(const boost::array<S, 9>& v) {
    Eigen::Matrix<S, 3, 3> m;
    m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    return m;
}

template <typename S>
inline Eigen::Matrix<S, 4, 4> Mat4FromArray(const std::vector<double>& v) {
    Eigen::Matrix<S, 4, 4> m;
    for (int i = 0; i < m.size(); ++i) m(i / 4, i % 4) = v[i];
    return m;
}

template <typename T>
T rad2deg(const T& radians) {
    return radians * 180.0 / M_PI;
}

template <typename T>
T deg2rad(const T& degrees) {
    return degrees * M_PI / 180.0;
}

template <typename T, typename T2>
void limit_in_range(T&& num, T2&& min_limit, T2&& max_limit) {
    if (num < min_limit) {
        num = min_limit;
    }
    if (num >= max_limit) {
        num = max_limit;
    }
}

/// 将角度保持在正负PI以内
inline void KeepAngleInPI(double& angle) {
    while (angle < -M_PI) {
        angle = angle + 2 * M_PI;
    }
    while (angle > M_PI) {
        angle = angle - 2 * M_PI;
    }
}

inline void KeepAngleIn2PI(double& angle) {
    while (angle <= 0) {
        angle = angle + 2 * M_PI;
    }
    while (angle > 2 * M_PI) {
        angle = angle - 2 * M_PI;
    }
}

} // namespace lightning::math

#endif // LIGHTNING_COMMON_MATH_UTILS_H

