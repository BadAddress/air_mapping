//
// Yaw angle prior edge for GPS rotation alignment
//

#ifndef MIAO_EDGE_YAW_PRIOR_H
#define MIAO_EDGE_YAW_PRIOR_H

#include "core/common/math.h"
#include "core/graph/base_unary_edge.h"
#include "core/types/vertex_yaw.h"
#include <cmath>

namespace lightning::miao {

/**
 * \brief Yaw angle prior edge (unary edge)
 *
 * 约束 yaw 角在初始估计值附近，防止优化发散
 * 
 * 误差 = yaw - yaw_prior (1维)
 */
class EdgeYawPrior : public BaseUnaryEdge<1, double, VertexYaw> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeYawPrior() = default;

    /// 计算误差
    void ComputeError() override {
        auto* v = static_cast<VertexYaw*>(vertices_[0]);
        double yaw = v->Estimate();
        
        // 误差 = 当前 yaw - 先验 yaw
        double diff = yaw - measurement_;
        
        // 归一化到 [-pi, pi]
        while (diff > M_PI) diff -= 2 * M_PI;
        while (diff < -M_PI) diff += 2 * M_PI;
        
        error_(0) = diff;
    }

    /// 计算雅可比矩阵
    void LinearizeOplus() override {
        // d_error / d_yaw = 1
        jacobian_oplus_xi_(0, 0) = 1.0;
    }
};

}  // namespace lightning::miao

#endif  // MIAO_EDGE_YAW_PRIOR_H

