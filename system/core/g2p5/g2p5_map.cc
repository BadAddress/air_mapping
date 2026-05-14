//
// G2P5 Map Implementation
// 迁移自 lightning-lm，移除 OpenCV 依赖
//

#include "modules/air_mapping/system/core/g2p5/g2p5_map.h"

#include <malloc.h>
#include <cstdlib>
#include <execution>

namespace lightning::g2p5 {

bool G2P5Map::Init(const float &temp_min_x, const float &temp_min_y, const float &temp_max_x, const float &temp_max_y) {
    ReleaseResources();
    min_x_ = temp_min_x;
    min_y_ = temp_min_y;
    max_x_ = temp_max_x;
    max_y_ = temp_max_y;

    grid_size_x_ = ceil((max_x_ - min_x_) / grid_reso_);
    grid_size_y_ = ceil((max_y_ - min_y_) / grid_reso_);

    if (grid_size_x_ <= 0 || grid_size_y_ <= 0) {
        return false;
    }

    grids_ = new SubGrid *[grid_size_x_];
    for (int xi = 0; xi < grid_size_x_; ++xi) {
        grids_[xi] = new SubGrid[grid_size_y_];
    }
    return true;
}

std::shared_ptr<G2P5Map> G2P5Map::MakeDeepCopy() {
    std::shared_ptr<G2P5Map> ret(new G2P5Map(options_));
    ret->min_x_ = min_x_;
    ret->min_y_ = min_y_;
    ret->max_x_ = max_x_;
    ret->max_y_ = max_y_;
    ret->grid_size_x_ = grid_size_x_;
    ret->grid_size_y_ = grid_size_y_;

    ret->grids_ = new SubGrid *[grid_size_x_];
    for (int xi = 0; xi < grid_size_x_; ++xi) {
        ret->grids_[xi] = new SubGrid[grid_size_y_];
        for (int yi = 0; yi < grid_size_y_; ++yi) {
            ret->grids_[xi][yi] = this->grids_[xi][yi];
        }
    }

    return ret;
}

bool G2P5Map::Resize(const float &temp_min_x, const float &temp_min_y, const float &temp_max_x,
                     const float &temp_max_y) {
    int temp_grid_size_x = ceil((temp_max_x - temp_min_x) / grid_reso_) + 1;
    int temp_grid_size_y = ceil((temp_max_y - temp_min_y) / grid_reso_) + 1;

    auto **new_grids = new SubGrid *[temp_grid_size_x];
    for (int xi = 0; xi < temp_grid_size_x; ++xi) {
        new_grids[xi] = new SubGrid[temp_grid_size_y];
    }

    int min_grid_x = (int)round((temp_min_x - min_x_) / grid_reso_);
    int min_grid_y = (int)round((temp_min_y - min_y_) / grid_reso_);
    int max_grid_x = (int)ceil((temp_max_x - min_x_) / grid_reso_);
    int max_grid_y = (int)ceil((temp_max_y - min_y_) / grid_reso_);

    int dx = min_grid_x < 0 ? 0 : min_grid_x;
    int dy = min_grid_y < 0 ? 0 : min_grid_y;
    int Dx = max_grid_x < this->grid_size_x_ ? max_grid_x : this->grid_size_x_;
    int Dy = max_grid_y < this->grid_size_y_ ? max_grid_y : this->grid_size_y_;

    for (int x = dx; x < Dx; x++) {
        for (int y = dy; y < Dy; y++) {
            assert((x - min_grid_x) >= 0 && (x - min_grid_x) < temp_grid_size_x);
            assert((y - min_grid_y) >= 0 && (y - min_grid_y) < temp_grid_size_y);

            assert((x) >= 0 && (x) < temp_grid_size_x);
            assert((y) >= 0 && (y) < temp_grid_size_y);

            new_grids[x - min_grid_x][y - min_grid_y] = this->grids_[x][y];
        }
        delete[] this->grids_[x];
    }

    delete[] this->grids_;
    this->grids_ = new_grids;
    this->min_x_ = temp_min_x;
    this->min_y_ = temp_min_y;
    this->max_x_ = temp_max_x;
    this->max_y_ = temp_max_y;
    this->grid_size_x_ = temp_grid_size_x;
    this->grid_size_y_ = temp_grid_size_y;

    return true;
}

G2P5Map::~G2P5Map() {
    if (grids_ != nullptr) {
        for (int xi = 0; xi < grid_size_x_; ++xi) {
            delete[] grids_[xi];
        }
        delete[] grids_;
        grids_ = nullptr;
    }
}

void G2P5Map::SetHitPoint(const float &px, const float &py, const bool &if_hit, float height) {
    if (grids_ == nullptr) {
        return;
    }

    if (px < min_x_ || px > max_x_ || py < min_y_ || py > max_y_) {
        return;
    }

    int x_index = floor((px - min_x_) / options_.resolution_);
    int y_index = floor((py - min_y_) / options_.resolution_);

    UpdateCell(Vec2i(x_index, y_index), if_hit, height);
}

void G2P5Map::UpdateCell(const Vec2i &point_index, const bool &if_hit, float height) {
    if (grids_ == nullptr) {
        return;
    }

    int x_index = point_index.x();
    int y_index = point_index.y();
    int xi = (x_index >> SUB_GRID_SIZE);
    int yi = (y_index >> SUB_GRID_SIZE);

    if (xi < 0 || xi > (grid_size_x_ - 1) || yi < 0 || yi > (grid_size_y_ - 1)) {
        return;
    }

    int sub_index_i = x_index - (xi << SUB_GRID_SIZE);
    int sub_index_j = y_index - (yi << SUB_GRID_SIZE);

    if (sub_index_i < 0 || sub_index_i > (sub_grid_width_ - 1) || sub_index_j < 0 ||
        sub_index_j > (sub_grid_width_ - 1)) {
        return;
    }

    grids_[xi][yi].SetGridHitPoint(if_hit, sub_index_i, sub_index_j, height);
}

void G2P5Map::SetMissPoint(const float &point_x, const float &point_y, const float &laser_origin_x,
                           const float &laser_origin_y, float height, float lidar_height) {
    if (grids_ == nullptr) {
        return;
    }

    int point_x_index = floor(point_x / options_.resolution_);
    int point_y_index = floor(point_y / options_.resolution_);

    int xi_lidar = floor(laser_origin_x / options_.resolution_);
    int yi_lidar = floor(laser_origin_y / options_.resolution_);

    float k = 0;
    int sign = 1;
    int diff_y = point_y_index - yi_lidar;
    int diff_x = point_x_index - xi_lidar;

    /// 整数的直线填充算法
    if (diff_y == 0 && diff_x == 0) {
        return;
    }

    if (!GetDataIndex(laser_origin_x, laser_origin_y, xi_lidar, yi_lidar)) {
        return;
    }

    std::vector<Vec2i> updated_pts;
    std::vector<float> heights;

    if (std::abs(diff_y) > std::abs(diff_x)) {
        if (diff_y == 0) {
            return;
        }

        k = float(diff_x) / diff_y;
        float dh = (lidar_height - height) / diff_y;

        sign = diff_y > 0 ? 1 : -1;
        for (int j = sign; j != diff_y; j += sign) {
            int i = float(j * k);
            int x_index = xi_lidar + i;
            int y_index = yi_lidar + j;

            updated_pts.emplace_back(Vec2i(x_index, y_index));
            heights.emplace_back(lidar_height - j * dh);
        }
    } else {
        if (diff_x == 0) {
            return;
        }

        k = float(diff_y) / diff_x;
        sign = diff_x > 0 ? 1 : -1;

        float dh = (lidar_height - height) / diff_x;

        for (int i = sign; i != diff_x; i += sign) {
            int j = float(i * k);
            int x_index = xi_lidar + i;
            int y_index = yi_lidar + j;

            updated_pts.emplace_back(Vec2i(x_index, y_index));
            heights.emplace_back(lidar_height - i * dh);
        }
    }

    for (size_t i = 0; i < updated_pts.size(); ++i) {
        UpdateCell(updated_pts[i], false, heights[i]);
    }
}

bool G2P5Map::GetDataIndex(const float x, const float y, int &x_index, int &y_index) {
    if (x > max_x_ || x < min_x_ || y > max_y_ || y < min_y_) {
        return false;
    }
    x_index = floor((x - min_x_) / options_.resolution_);
    y_index = floor((y - min_y_) / options_.resolution_);

    return true;
}

void G2P5Map::ReleaseResources() {
    if (grids_ != nullptr) {
        for (int xi = 0; xi < grid_size_x_; ++xi) {
            delete[] grids_[xi];
        }
        delete[] grids_;
        grids_ = nullptr;
    }
    min_x_ = min_y_ = 10000;
    max_x_ = max_y_ = -10000;

    grid_size_x_ = grid_size_y_ = 0;
    malloc_trim(0);
}

ImageData G2P5Map::ToImage() {
    int image_width = grid_size_x_ * sub_grid_width_;
    int image_height = grid_size_y_ * sub_grid_width_;

    // 定义要映射的颜色值
    const uint8_t black_r = 0, black_g = 0, black_b = 0;
    const uint8_t white_r = 255, white_g = 255, white_b = 255;
    const uint8_t gray_r = 127, gray_g = 127, gray_b = 127;

    ImageData image(image_width, image_height, 3);

    int image_height_1 = image_height - 1;
    int image_width_1 = image_width - 1;
    int index_y_min, index_y_max, index_x_min, index_x_max;

    for (int bxi = 0; bxi < grid_size_x_; ++bxi) {
        for (int byi = 0; byi < grid_size_y_; ++byi) {
            if (grids_[bxi][byi].IsEmpty()) {
                continue;
            }

            for (int sxi = 0; sxi < sub_grid_width_; ++sxi) {
                for (int syi = 0; syi < sub_grid_width_; ++syi) {
                    int x = (bxi << SUB_GRID_SIZE) + sxi;
                    int y = (byi << SUB_GRID_SIZE) + syi;

                    if (x >= 0 && x < image_width && y >= 0 && y < image_height) {
                        unsigned int hit_cnt = 0, visit_cnt = 0;
                        grids_[bxi][byi].GetHitAndVisit(sxi, syi, hit_cnt, visit_cnt);

                        float occ = visit_cnt ? (hit_cnt == 0 ? 0 : (float)hit_cnt / (float)visit_cnt) : -1;

                        /// 注意这里有转置符号
                        if (occ < 0) {
                            continue;
                        } else if (occ > options_.occupancy_ratio_) {  // 0.49
                            image.SetPixel(x, y, black_r, black_g, black_b);
                        } else {
                            index_y_min = std::max(0, y - 1);
                            index_y_max = std::min(image_height_1, y + 1);
                            index_x_min = std::max(0, x - 1);
                            index_x_max = std::min(image_width_1, x + 1);
                            for (int extend_y = index_y_min; extend_y <= index_y_max; extend_y++) {
                                for (int extend_x = index_x_min; extend_x <= index_x_max; extend_x++) {
                                    uint8_t r, g, b;
                                    image.GetPixel(extend_x, extend_y, r, g, b);
                                    if (r == gray_r && g == gray_g && b == gray_b) {
                                        image.SetPixel(extend_x, extend_y, white_r, white_g, white_b);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /// 水平翻转以转到俯视视角
    image.FlipHorizontal();

    /// 180度旋转以匹配坐标系
    image.Rotate180();

    // 只需要垂直翻转，让 (0,0) 在左下角 (PGM 标准)
    // image.FlipVertical();

    return image;
}

bool G2P5Map::SavePPM(const std::string& filename) {
    ImageData image = ToImage();
    return image.SavePPM(filename);
}

bool G2P5Map::SavePGM(const std::string& filename) {
    ImageData image = ToImage();
    return image.SavePGM(filename);
}

}  // namespace lightning::g2p5
