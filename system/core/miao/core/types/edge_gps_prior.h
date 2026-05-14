//
// Created for GPS fusion in map_builder
// GPS position prior edge (unary edge)
//

#ifndef MIAO_EDGE_GPS_PRIOR_H
#define MIAO_EDGE_GPS_PRIOR_H

#include "core/common/math.h"
#include "core/graph/base_unary_edge.h"
#include "core/types/vertex_se3.h"

namespace lightning::miao {

/**
 * GPS 位置先验边（一元边）
 * 
 * 约束关键帧位姿的平移部分与 GPS 观测一致
 * 
 * 顶点：SE3 位姿 T = [R | t]
 * 测量值：GPS 位置 p_gps = [x, y, z] (在局部坐标系下)
 * 误差：e = p_gps - t (3维)
 * 
 * 注意：
 * - 这是一个只约束平移的一元边
 * - 不约束旋转，因为 GPS 不提供姿态信息
 * - 信息矩阵应根据 GPS 的标准差设置
 */
class EdgeGPSPrior : public BaseUnaryEdge<3, Vec3d, VertexSE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    EdgeGPSPrior() = default;
    
    /**
     * 计算误差
     * 误差 = 测量的 GPS 位置 - 估计位姿的平移部分
     */
    void ComputeError() override {
        const VertexSE3* v = static_cast<const VertexSE3*>(vertices_[0]);
        SE3 pose = v->Estimate();
        
        // 误差 = GPS测量位置 - 当前估计位置
        error_ = measurement_ - pose.translation();
    }
    
    /**
     * 计算雅可比矩阵 (解析雅可比)
     * 
     * 误差 e = p_gps - t
     * 对平移 t 的雅可比: ∂e/∂t = -I (3x3)
     * 对旋转的雅可比: ∂e/∂θ = 0 (3x3)
     * 
     * VertexSE3 的参数顺序是 [translation, rotation]
     * 所以完整雅可比是 [3x6]: [-I, 0]
     */
    void LinearizeOplus() override {
        // jacobian_oplus_xi_ 是 3x6 的矩阵
        // 前3列是对平移的雅可比，后3列是对旋转的雅可比
        jacobian_oplus_xi_.setZero();
        jacobian_oplus_xi_.block<3, 3>(0, 0) = -Mat3d::Identity();
        // 对旋转的雅可比为 0，已在 setZero 中设置
    }
};

}  // namespace lightning::miao

#endif  // MIAO_EDGE_GPS_PRIOR_H

