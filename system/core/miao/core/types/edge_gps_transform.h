//
// GPS fusion edge with optimizable UTM-to-LIO transformation
// The transformation T_utm2lio = [R(yaw), t] is jointly optimized with keyframe poses
//

#ifndef MIAO_EDGE_GPS_TRANSFORM_H
#define MIAO_EDGE_GPS_TRANSFORM_H

#include "core/common/math.h"
#include "core/graph/base_binary_edge.h"
#include "core/types/vertex_se3.h"
#include "core/types/vertex_se2.h"
#include <cmath>

namespace lightning::miao {

/**
 * \brief GPS 位置约束边，包含可优化的 UTM-to-LIO 变换
 *
 * 这是一个二元边，连接：
 * - Vertex 0: VertexSE3 (关键帧位姿, 在 LIO 坐标系下)
 * - Vertex 1: VertexSE2 (UTM 到 LIO 的 2D 变换: [tx, ty, yaw])
 *
 * 变换模型:
 *   p_lio = R(yaw) * p_utm + t
 *   其中 R(yaw) = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)]
 *        t = [tx, ty]
 *
 * 测量值: GPS 位置 p_utm (UTM 坐标系，已减去 offset)
 * 
 * 误差 (2D):
 *   e = R(yaw) * p_utm + t - kf_position_lio[0:2]
 *
 * 信息矩阵: 根据 GPS 标准差设置，约束 x, y 方向
 *
 * 雅可比矩阵:
 *   对 SE3 位姿: ∂e/∂pose = [-I_{2x2}, 0_{2x4}]  (只对平移 x,y 有导数)
 *   对 SE2 变换: ∂e/∂[tx, ty, yaw] = [1, 0, -sin*px - cos*py]
 *                                    [0, 1,  cos*px - sin*py]
 */
class EdgeGPSTransform : public BaseBinaryEdge<2, Vec2d, VertexSE3, VertexSE2> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeGPSTransform() = default;

    /// 设置 GPS 在 UTM 坐标系下的位置（已减去 offset）
    void SetGpsPositionUtm(const Vec2d& pos) { gps_position_utm_ = pos; }
    
    /// 获取 GPS 位置
    const Vec2d& GetGpsPositionUtm() const { return gps_position_utm_; }

    /// 计算误差
    void ComputeError() override {
        const auto* v_pose = static_cast<const VertexSE3*>(vertices_[0]);  // 关键帧位姿 (LIO 坐标系)
        const auto* v_trans = static_cast<const VertexSE2*>(vertices_[1]); // UTM-to-LIO 变换

        SE3 pose = v_pose->Estimate();
        SE2 trans = v_trans->Estimate();

        // 从 SE2 提取变换参数
        double yaw = trans.so2().log();  // yaw 角
        Vec2d t = trans.translation();   // 平移 [tx, ty]

        double c = cos(yaw);
        double s = sin(yaw);

        // GPS 位置变换到 LIO 坐标系: p_lio = R(yaw) * p_utm + t
        Vec2d gps_in_lio;
        gps_in_lio.x() = c * gps_position_utm_.x() - s * gps_position_utm_.y() + t.x();
        gps_in_lio.y() = s * gps_position_utm_.x() + c * gps_position_utm_.y() + t.y();

        // 误差 = 变换后的 GPS 位置 - 关键帧位置
        Vec2d kf_position = pose.translation().head<2>();
        error_ = gps_in_lio - kf_position;
    }

    /// 计算雅可比矩阵（解析雅可比）
    void LinearizeOplus() override {
        const auto* v_trans = static_cast<const VertexSE2*>(vertices_[1]);
        SE2 trans = v_trans->Estimate();
        double yaw = trans.so2().log();
        double c = cos(yaw);
        double s = sin(yaw);

        // 对 SE3 位姿的雅可比 (2x6)
        // 误差只与位姿的平移部分 (x, y) 有关
        // ∂e/∂pose = [-I, 0]
        // VertexSE3 参数顺序: [tx, ty, tz, rx, ry, rz]
        jacobian_oplus_xi_.setZero();
        jacobian_oplus_xi_(0, 0) = -1.0;  // ∂e_x/∂tx
        jacobian_oplus_xi_(1, 1) = -1.0;  // ∂e_y/∂ty

        // 对 SE2 变换的雅可比 (2x3)
        // VertexSE2 参数顺序: [tx, ty, yaw]
        // gps_in_lio = [c*px - s*py + tx, s*px + c*py + ty]
        // ∂e/∂tx = [1, 0]
        // ∂e/∂ty = [0, 1]
        // ∂e/∂yaw = [-s*px - c*py, c*px - s*py]
        jacobian_oplus_xj_(0, 0) = 1.0;  // ∂e_x/∂tx
        jacobian_oplus_xj_(0, 1) = 0.0;
        jacobian_oplus_xj_(0, 2) = -s * gps_position_utm_.x() - c * gps_position_utm_.y();

        jacobian_oplus_xj_(1, 0) = 0.0;
        jacobian_oplus_xj_(1, 1) = 1.0;  // ∂e_y/∂ty
        jacobian_oplus_xj_(1, 2) = c * gps_position_utm_.x() - s * gps_position_utm_.y();
    }

   private:
    Vec2d gps_position_utm_ = Vec2d::Zero();  // GPS 位置（UTM 坐标系，已减 offset）
};

}  // namespace lightning::miao

#endif  // MIAO_EDGE_GPS_TRANSFORM_H
