//
// GPS fusion edge with optimizable UTM-to-LIO transformation (3D version with elevation)
// The transformation T_utm2lio = [R(yaw), t] is jointly optimized with keyframe poses
//

#ifndef MIAO_EDGE_GPS_TRANSFORM_3D_H
#define MIAO_EDGE_GPS_TRANSFORM_3D_H

#include "core/common/math.h"
#include "core/graph/base_binary_edge.h"
#include "core/types/vertex_se3.h"
#include "core/types/vertex_se2.h"
#include <cmath>

namespace lightning::miao {

/**
 * \brief GPS 位置约束边（3D 版本，包含高程约束）
 *
 * 这是一个二元边，连接：
 * - Vertex 0: VertexSE3 (关键帧位姿, 在 LIO 坐标系下)
 * - Vertex 1: VertexSE2 (UTM 到 LIO 的 2D 变换: [tx, ty, yaw])
 *
 * 变换模型 (平面):
 *   p_lio_xy = R(yaw) * p_utm_xy + t
 *   其中 R(yaw) = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)]
 *        t = [tx, ty]
 *
 * 高程变换:
 *   p_lio_z = p_utm_z + tz_offset
 *   其中 tz_offset 是固定的高程偏移（不参与优化）
 *
 * 测量值: GPS 位置 p_utm (UTM 坐标系，已减去 offset)
 * 
 * 误差 (3D):
 *   e_xy = R(yaw) * p_utm_xy + t - kf_position_lio[0:2]
 *   e_z  = p_utm_z + tz_offset - kf_position_lio[2]
 *
 * 信息矩阵: 根据 GPS 标准差设置，约束 x, y, z 方向
 *
 * 雅可比矩阵:
 *   对 SE3 位姿: ∂e/∂pose = [-I_{3x3}, 0_{3x3}]  (对平移 x,y,z 有导数)
 *   对 SE2 变换: ∂e/∂[tx, ty, yaw] = [1, 0, -sin*px - cos*py]
 *                                    [0, 1,  cos*px - sin*py]
 *                                    [0, 0,  0]
 */
class EdgeGPSTransform3D : public BaseBinaryEdge<3, Vec3d, VertexSE3, VertexSE2> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeGPSTransform3D() = default;

    /// 设置 GPS 在 UTM 坐标系下的位置（已减去 offset，包含高程）
    void SetGpsPositionUtm(const Vec3d& pos) { gps_position_utm_ = pos; }
    
    /// 获取 GPS 位置
    const Vec3d& GetGpsPositionUtm() const { return gps_position_utm_; }
    
    /// 设置高程偏移（UTM 高程到 LIO 高程的固定偏移）
    void SetHeightOffset(double offset) { height_offset_ = offset; }
    
    /// 获取高程偏移
    double GetHeightOffset() const { return height_offset_; }

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

        // GPS 位置变换到 LIO 坐标系
        // p_lio_xy = R(yaw) * p_utm_xy + t
        // p_lio_z = p_utm_z + height_offset
        Vec3d gps_in_lio;
        gps_in_lio.x() = c * gps_position_utm_.x() - s * gps_position_utm_.y() + t.x();
        gps_in_lio.y() = s * gps_position_utm_.x() + c * gps_position_utm_.y() + t.y();
        gps_in_lio.z() = gps_position_utm_.z() + height_offset_;

        // 误差 = 变换后的 GPS 位置 - 关键帧位置
        Vec3d kf_position = pose.translation();
        error_ = gps_in_lio - kf_position;
    }

    /// 计算雅可比矩阵（解析雅可比）
    void LinearizeOplus() override {
        const auto* v_trans = static_cast<const VertexSE2*>(vertices_[1]);
        SE2 trans = v_trans->Estimate();
        double yaw = trans.so2().log();
        double c = cos(yaw);
        double s = sin(yaw);

        // 对 SE3 位姿的雅可比 (3x6)
        // 误差与位姿的平移部分 (x, y, z) 有关
        // ∂e/∂pose = [-I, 0]
        // VertexSE3 参数顺序: [tx, ty, tz, rx, ry, rz]
        jacobian_oplus_xi_.setZero();
        jacobian_oplus_xi_(0, 0) = -1.0;  // ∂e_x/∂tx
        jacobian_oplus_xi_(1, 1) = -1.0;  // ∂e_y/∂ty
        jacobian_oplus_xi_(2, 2) = -1.0;  // ∂e_z/∂tz

        // 对 SE2 变换的雅可比 (3x3)
        // VertexSE2 参数顺序: [tx, ty, yaw]
        // gps_in_lio = [c*px - s*py + tx, s*px + c*py + ty, pz + offset]
        // ∂e/∂tx = [1, 0, 0]
        // ∂e/∂ty = [0, 1, 0]
        // ∂e/∂yaw = [-s*px - c*py, c*px - s*py, 0]
        jacobian_oplus_xj_(0, 0) = 1.0;   // ∂e_x/∂tx
        jacobian_oplus_xj_(0, 1) = 0.0;
        jacobian_oplus_xj_(0, 2) = -s * gps_position_utm_.x() - c * gps_position_utm_.y();

        jacobian_oplus_xj_(1, 0) = 0.0;
        jacobian_oplus_xj_(1, 1) = 1.0;   // ∂e_y/∂ty
        jacobian_oplus_xj_(1, 2) = c * gps_position_utm_.x() - s * gps_position_utm_.y();

        jacobian_oplus_xj_(2, 0) = 0.0;
        jacobian_oplus_xj_(2, 1) = 0.0;
        jacobian_oplus_xj_(2, 2) = 0.0;   // 高程与 yaw 无关
    }

   private:
    Vec3d gps_position_utm_ = Vec3d::Zero();  // GPS 位置（UTM 坐标系，已减 offset，含高程）
    double height_offset_ = 0.0;              // 高程偏移（UTM -> LIO）
};

}  // namespace lightning::miao

#endif  // MIAO_EDGE_GPS_TRANSFORM_3D_H
