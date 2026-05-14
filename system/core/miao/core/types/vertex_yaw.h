//
// Yaw angle vertex for GPS rotation alignment optimization
//

#ifndef MIAO_VERTEX_YAW_H
#define MIAO_VERTEX_YAW_H

#include "core/graph/base_vertex.h"
#include <cmath>

namespace lightning::miao {

/**
 * \brief 1D Yaw angle vertex
 *
 * 用于优化 LIO 坐标系到 UTM 坐标系的 yaw 角对齐
 * 估计值是一个 double (弧度)
 * 更新量也是 double
 */
class VertexYaw : public BaseVertex<1, double> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexYaw() { estimate_ = 0.0; }

    /// 覆盖 oplus：yaw = yaw + delta
    void OplusImpl(const double *update) override {
        estimate_ += update[0];
        // 归一化到 [-pi, pi]
        while (estimate_ > M_PI) estimate_ -= 2 * M_PI;
        while (estimate_ < -M_PI) estimate_ += 2 * M_PI;
    }
};

}  // namespace lightning::miao

#endif  // MIAO_VERTEX_YAW_H

