//
// GPS position constraint with optimizable yaw alignment
//

#ifndef MIAO_EDGE_GPS_WITH_YAW_H
#define MIAO_EDGE_GPS_WITH_YAW_H

#include "core/common/math.h"
#include "core/graph/base_binary_edge.h"
#include "core/types/vertex_se3.h"
#include "core/types/vertex_yaw.h"
#include <cmath>

namespace lightning::miao {

/**
 * \brief GPS position constraint with yaw rotation alignment
 *
 * 这是一个二元边，连接：
 * - Vertex 0: VertexSE3 (关键帧位姿)
 * - Vertex 1: VertexYaw (LIO 到 UTM 的 yaw 角)
 *
 * 测量值: GPS 位置（已减去 offset，在 UTM 坐标系下）
 * 误差 = R(yaw) * gps_position_utm - kf_position_lio
 *
 * 其中 R(yaw) 是绕 Z 轴的旋转矩阵，将 UTM 坐标转换到 LIO 坐标系
 */
class EdgeGPSWithYaw : public BaseBinaryEdge<2, Vec2d, VertexSE3, VertexYaw> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeGPSWithYaw() = default;

    /// 设置 GPS 在 UTM 坐标系下的位置（已减去 offset）
    void SetGpsPositionUtm(const Vec2d& pos) { gps_position_utm_ = pos; }

    /// 计算误差
    void ComputeError() override {
        auto* v0 = static_cast<VertexSE3*>(vertices_[0]);  // 关键帧位姿 (LIO 坐标系)
        auto* v1 = static_cast<VertexYaw*>(vertices_[1]);  // yaw 角

        SE3 pose = v0->Estimate();
        double yaw = v1->Estimate();

        // R(yaw) 将 UTM 坐标转换到 LIO 坐标系
        double c = cos(yaw);
        double s = sin(yaw);

        // GPS 位置在 LIO 坐标系下
        Vec2d gps_in_lio;
        gps_in_lio.x() = c * gps_position_utm_.x() - s * gps_position_utm_.y();
        gps_in_lio.y() = s * gps_position_utm_.x() + c * gps_position_utm_.y();

        // 误差 = GPS 位置 - 关键帧位置 (在 LIO 坐标系下)
        Vec2d kf_position = pose.translation().head<2>();
        error_ = gps_in_lio - kf_position;
    }

    /// 计算雅可比矩阵（解析雅可比）
    void LinearizeOplus() override {
        auto* v1 = static_cast<VertexYaw*>(vertices_[1]);
        double yaw = v1->Estimate();
        double c = cos(yaw);
        double s = sin(yaw);

        // 对 SE3 位姿的雅可比 (只有平移部分 x, y)
        // d_error / d_pose = [-I, 0] (误差对 x, y 的导数是 -1)
        // jacobian_oplus_xi_ 是 2x6 的矩阵
        jacobian_oplus_xi_.setZero();
        jacobian_oplus_xi_(0, 0) = -1.0;  // d_error_x / d_x
        jacobian_oplus_xi_(1, 1) = -1.0;  // d_error_y / d_y

        // 对 yaw 的雅可比
        // gps_in_lio = [c*gps_x - s*gps_y, s*gps_x + c*gps_y]
        // d_gps_in_lio / d_yaw = [-s*gps_x - c*gps_y, c*gps_x - s*gps_y]
        // jacobian_oplus_xj_ 是 2x1 的矩阵
        jacobian_oplus_xj_(0, 0) = -s * gps_position_utm_.x() - c * gps_position_utm_.y();
        jacobian_oplus_xj_(1, 0) = c * gps_position_utm_.x() - s * gps_position_utm_.y();
    }

   private:
    Vec2d gps_position_utm_ = Vec2d::Zero();  // GPS 位置（UTM 坐标系，已减 offset）
};

}  // namespace lightning::miao

#endif  // MIAO_EDGE_GPS_WITH_YAW_H
