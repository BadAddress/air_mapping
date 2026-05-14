//
// G2P5 Map - 2.5D 栅格地图
// 迁移自 lightning-lm，移除 OpenCV 依赖
//

#ifndef AIR_LOCALIZATION_G2P5_MAP_H
#define AIR_LOCALIZATION_G2P5_MAP_H

#include "modules/air_mapping/system/common/eigen_types.h"
#include "modules/air_mapping/system/common/std_types.h"
#include "modules/air_mapping/system/core/g2p5/g2p5_subgrid.h"

#include <bitset>
#include <vector>
#include <fstream>

namespace lightning::g2p5 {

/// 简单的图像数据结构（替代 cv::Mat）
struct ImageData {
    int width = 0;
    int height = 0;
    int channels = 3;  // RGB
    std::vector<uint8_t> data;
    
    ImageData() = default;
    ImageData(int w, int h, int c = 3) : width(w), height(h), channels(c) {
        data.resize(w * h * c, 127);  // 默认灰色
    }
    
    void SetPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        int idx = (y * width + x) * channels;
        data[idx] = r;
        data[idx + 1] = g;
        data[idx + 2] = b;
    }
    
    void GetPixel(int x, int y, uint8_t& r, uint8_t& g, uint8_t& b) const {
        if (x < 0 || x >= width || y < 0 || y >= height) {
            r = g = b = 127;
            return;
        }
        int idx = (y * width + x) * channels;
        r = data[idx];
        g = data[idx + 1];
        b = data[idx + 2];
    }
    
    /// 水平翻转
    void FlipHorizontal() {
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width / 2; ++x) {
                int x2 = width - 1 - x;
                for (int c = 0; c < channels; ++c) {
                    std::swap(data[(y * width + x) * channels + c],
                              data[(y * width + x2) * channels + c]);
                }
            }
        }
    }

    /// 180度旋转
    void Rotate180() {
        int total = width * height;
        for (int i = 0; i < total / 2; ++i) {
            int idx1 = i * channels;
            int idx2 = (total - 1 - i) * channels;
            for (int c = 0; c < channels; ++c) {
                std::swap(data[idx1 + c], data[idx2 + c]);
            }
        }
    }
    
    /// 保存为 PPM 格式（无需外部库）
    bool SavePPM(const std::string& filename) const {
        std::ofstream file(filename, std::ios::binary);
        if (!file.is_open()) return false;
        
        file << "P6\n" << width << " " << height << "\n255\n";
        file.write(reinterpret_cast<const char*>(data.data()), data.size());
        return true;
    }
    
    /// 保存为 PGM 格式（灰度图，无需外部库）
    bool SavePGM(const std::string& filename) const {
        std::ofstream file(filename, std::ios::binary);
        if (!file.is_open()) return false;
        
        file << "P5\n" << width << " " << height << "\n255\n";
        
        // 转换为灰度
        std::vector<uint8_t> gray(width * height);
        for (int i = 0; i < width * height; ++i) {
            // 简单灰度转换
            gray[i] = static_cast<uint8_t>(
                0.299 * data[i * 3] + 0.587 * data[i * 3 + 1] + 0.114 * data[i * 3 + 2]);
        }
        file.write(reinterpret_cast<const char*>(gray.data()), gray.size());
        return true;
    }
};

/**
 * g2p5自定义栅格地图数据
 * 主要用于对外的绘制，并没有匹配部分
 * 数据存储在grids_中，是一个二维数组，内部还有4层subgrids
 * x为行指针，y为列指针，y优先增长
 *
 * 可以通过 ToImage() 转换成图像格式进行显示和存储
 */
class G2P5Map {
   public:
    struct Options {
        float resolution_ = 0.05;      /// 子网格的最高分辨率
        float occupancy_ratio_ = 0.3;  // 占用比例
    };

    G2P5Map(Options options) : options_(options) {
        grid_reso_ = options_.resolution_ * sub_grid_width_;
        grids_ = nullptr;
    }

    ~G2P5Map();

    /// 子网格的大小
    static inline constexpr int SUB_GRID_SIZE = SubGrid::SUB_GRID_SIZE;

    /// 由自身内容创建一个深拷贝
    std::shared_ptr<G2P5Map> MakeDeepCopy();

    /// 转换至图像数据（替代 ToCV）
    ImageData ToImage();
    
    /// 保存地图为 PPM 文件
    bool SavePPM(const std::string& filename);
    
    /// 保存地图为 PGM 文件（灰度图）
    bool SavePGM(const std::string& filename);

    /// 清空内部数据
    void ReleaseResources();

    bool Init(const float &temp_min_x, const float &temp_min_y, const float &temp_max_x, const float &temp_max_y);

    bool Resize(const float &temp_min_x, const float &temp_min_y, const float &temp_max_x, const float &temp_max_y);

    void SetHitPoint(const float &px, const float &py, const bool &if_hit, float height);

    /**
     * 白色区域的直线填充算法
     * 除了2D填充以外，还需要给出雷达高度和目标位置高度
     */
    void SetMissPoint(const float &point_x, const float &point_y, const float &laser_origin_x,
                      const float &laser_origin_y, float height, float lidar_height);

    bool IsObstacle(const Vec2i &point) {
        // 判断点是否属于障碍物区域，可以是通过查找 grid 中是否已经有墙体或物体数据
        int xi = point.x() >> SUB_GRID_SIZE;
        int yi = point.y() >> SUB_GRID_SIZE;
        if (xi < 0 || xi >= grid_size_x_ || yi < 0 || yi >= grid_size_y_) {
            return false;
        }
        return true;
    }

    void UpdateCell(const Vec2i &point_index, const bool &if_hit, float height);

    bool GetDataIndex(const float x, const float y, int &x_index, int &y_index);

    bool IsEmpty() { return (grids_ == nullptr); }

    inline void GetMinAndMax(float &min_x, float &min_y, float &max_x, float &max_y) {
        min_x = min_x_;
        min_y = min_y_;
        max_x = max_x_;
        max_y = max_y_;
    }
    inline void SetMinAndMax(float &min_x, float &min_y, float &max_x, float &max_y) {
        min_x_ = min_x;
        min_y_ = min_y;
        max_x_ = max_x;
        max_y_ = max_y;
    }

    float GetGridResolution() const { return options_.resolution_; }
    
    int GetImageWidth() const { return grid_size_x_ * sub_grid_width_; }
    int GetImageHeight() const { return grid_size_y_ * sub_grid_width_; }
    
    /// 获取原点坐标 (用于生成 map.yaml)
    void GetOrigin(float& origin_x, float& origin_y) const {
        origin_x = min_x_;
        origin_y = min_y_;
    }

   private:
    inline int MapIdx(int sx, int x, int y) { return (sx) * (y) + (x); }

   private:
    float grid_reso_ = 0.0;  /// subgrid的栅格分辨率
    float min_x_ = 0, min_y_ = 0, max_x_ = 0, max_y_ = 0;
    int grid_size_x_ = 0, grid_size_y_ = 0;

    inline static const int sub_grid_width_ = (1 << SUB_GRID_SIZE);

    Options options_;

    SubGrid **grids_ = nullptr;  // 实际存储数据的位置
};

typedef std::shared_ptr<G2P5Map> G2P5MapPtr;

}  // namespace lightning::g2p5

#endif  // AIR_LOCALIZATION_G2P5_MAP_H
