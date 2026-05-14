//
// Created for GPS fusion in map_builder
// UTM coordinate converter (Latitude/Longitude to UTM)
//

#ifndef LIGHTNING_UTM_CONVERTER_H
#define LIGHTNING_UTM_CONVERTER_H

#include <cmath>
#include "common/eigen_types.h"

namespace lightning {

/**
 * UTM 坐标转换工具
 * 
 * 将 WGS84 经纬度坐标转换为 UTM (Universal Transverse Mercator) 坐标
 * 
 * UTM 投影参数:
 * - WGS84 椭球: a = 6378137m, f = 1/298.257223563
 * - 比例因子 k0 = 0.9996
 * - 东偏移 500000m, 南半球北偏移 10000000m
 */
class UTMConverter {
public:
    struct UTMCoord {
        double x = 0.0;      // 东向坐标 (米)
        double y = 0.0;      // 北向坐标 (米)
        double z = 0.0;      // 高程 (米)
        int zone = 0;        // UTM 区号
        bool is_north = true; // 是否在北半球
    };
    
    /**
     * 将经纬度转换为 UTM 坐标
     * @param lat_deg 纬度 (度)
     * @param lon_deg 经度 (度)
     * @param height_msl 海拔高度 (米)
     * @return UTM 坐标
     */
    static UTMCoord LatLonToUTM(double lat_deg, double lon_deg, double height_msl = 0.0) {
        UTMCoord result;
        
        // WGS84 椭球参数
        const double a = 6378137.0;           // 长半轴 (米)
        const double f = 1.0 / 298.257223563; // 扁率
        const double b = a * (1.0 - f);       // 短半轴
        const double e2 = (a*a - b*b) / (a*a); // 第一偏心率的平方
        const double ep2 = (a*a - b*b) / (b*b); // 第二偏心率的平方
        const double k0 = 0.9996;             // 比例因子
        
        // 转换为弧度
        double lat_rad = lat_deg * M_PI / 180.0;
        double lon_rad = lon_deg * M_PI / 180.0;
        
        // 计算 UTM 区号
        result.zone = static_cast<int>((lon_deg + 180.0) / 6.0) + 1;
        result.is_north = (lat_deg >= 0);
        
        // 中央子午线经度
        double lon0 = ((result.zone - 1) * 6.0 - 180.0 + 3.0) * M_PI / 180.0;
        
        // 计算辅助量
        double N = a / std::sqrt(1.0 - e2 * std::sin(lat_rad) * std::sin(lat_rad));
        double T = std::tan(lat_rad) * std::tan(lat_rad);
        double C = ep2 * std::cos(lat_rad) * std::cos(lat_rad);
        double A = std::cos(lat_rad) * (lon_rad - lon0);
        
        // 子午线弧长
        double M = a * ((1.0 - e2/4.0 - 3.0*e2*e2/64.0 - 5.0*e2*e2*e2/256.0) * lat_rad
                      - (3.0*e2/8.0 + 3.0*e2*e2/32.0 + 45.0*e2*e2*e2/1024.0) * std::sin(2.0*lat_rad)
                      + (15.0*e2*e2/256.0 + 45.0*e2*e2*e2/1024.0) * std::sin(4.0*lat_rad)
                      - (35.0*e2*e2*e2/3072.0) * std::sin(6.0*lat_rad));
        
        // 计算 UTM 坐标
        result.x = k0 * N * (A + (1.0 - T + C) * A*A*A / 6.0
                           + (5.0 - 18.0*T + T*T + 72.0*C - 58.0*ep2) * A*A*A*A*A / 120.0);
        result.x += 500000.0;  // 东偏移
        
        result.y = k0 * (M + N * std::tan(lat_rad) * (A*A / 2.0
                       + (5.0 - T + 9.0*C + 4.0*C*C) * A*A*A*A / 24.0
                       + (61.0 - 58.0*T + T*T + 600.0*C - 330.0*ep2) * A*A*A*A*A*A / 720.0));
        
        // 南半球北偏移
        if (!result.is_north) {
            result.y += 10000000.0;
        }
        
        result.z = height_msl;
        
        return result;
    }
    
    /**
     * 将经纬度转换为 Vec3d (UTM 坐标)
     */
    static Vec3d LatLonToVec3d(double lat_deg, double lon_deg, double height_msl = 0.0) {
        UTMCoord utm = LatLonToUTM(lat_deg, lon_deg, height_msl);
        return Vec3d(utm.x, utm.y, utm.z);
    }
    
    /**
     * 检查两个位置是否在同一 UTM 区
     */
    static bool SameZone(double lon1_deg, double lon2_deg) {
        int zone1 = static_cast<int>((lon1_deg + 180.0) / 6.0) + 1;
        int zone2 = static_cast<int>((lon2_deg + 180.0) / 6.0) + 1;
        return zone1 == zone2;
    }
};

}  // namespace lightning

#endif  // LIGHTNING_UTM_CONVERTER_H

