//
// G2P5 Grid Data - 2.5D 栅格数据定义
// 迁移自 lightning-lm，移除 OpenCV 依赖
//

#ifndef AIR_LOCALIZATION_G2P5_GRID_DATA_H
#define AIR_LOCALIZATION_G2P5_GRID_DATA_H

namespace lightning::g2p5 {

/// 2.5D 子地图定义
/// 子网格中的数据内容
struct GridData {
    explicit GridData(unsigned int occupySum = 0, unsigned int visitSum = 0)
        : hit_cnt_(occupySum), visit_cnt_(visitSum) {}

    unsigned int hit_cnt_ = 0;    // 占据的计数
    unsigned int visit_cnt_ = 0;  // 总共的计数
    float height_ = 10000;        // 相对地面的最低高度，可以从上方穿过但不能从下方穿过
};

}  // namespace lightning::g2p5

#endif  // AIR_LOCALIZATION_G2P5_GRID_DATA_H
