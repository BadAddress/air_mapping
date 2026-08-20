#!/usr/bin/env python3
"""Measure the usable Helios-32 LIO work set in Apollo Cyber records.

The PointCloud protobuf used by this project has no ring field.  This tool
therefore reports geometric range/elevation distributions and the retention
ratio of candidate spherical range gates.  It reads records without replaying
them and never modifies mapping artifacts.
"""

import argparse
import collections
import math
import os
import statistics
import sys

from cyber.proto import record_pb2
from modules.common_msgs.sensor_msgs import pointcloud_pb2

from analyze_cyber_record import HEADER_RESERVED_BYTES, SECTION_STRUCT, read_header


TOP_CLOUD = "/apollo/sensor/rslidar/top/PointCloud2"
RANGE_EDGES_M = (0.0, 2.0, 5.0, 10.0, 20.0, 30.0, 40.0, 45.0,
                 50.0, 60.0, 80.0, 100.0, 120.0, float("inf"))
CANDIDATE_MAX_RANGE_M = (30.0, 35.0, 40.0, 45.0, 50.0, 60.0)


def percentile(values, fraction):
    if not values:
        return float("nan")
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


class WorksetStats:
    def __init__(self, scan_step):
        self.scan_step = scan_step
        self.cloud_count = 0
        self.sampled_cloud_count = 0
        self.sampled_point_count = 0
        self.finite_point_count = 0
        self.nonfinite_point_count = 0
        self.ranges = []
        self.horizontal_ranges = []
        self.z_values = []
        self.range_histogram = [0] * (len(RANGE_EDGES_M) - 1)
        self.elevation_histogram = collections.Counter()
        self.candidate_counts = collections.Counter()
        self.high_point_counts = collections.Counter()
        self.per_cloud_retention = {
            limit: [] for limit in CANDIDATE_MAX_RANGE_M
        }

    def add_cloud(self, cloud):
        cloud_index = self.cloud_count
        self.cloud_count += 1
        if cloud_index % self.scan_step != 0:
            return
        self.sampled_cloud_count += 1
        finite_this_cloud = 0
        candidate_this_cloud = collections.Counter()
        for point in cloud.point:
            self.sampled_point_count += 1
            x, y, z = point.x, point.y, point.z
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                self.nonfinite_point_count += 1
                continue
            radius_xy = math.hypot(x, y)
            distance = math.hypot(radius_xy, z)
            if distance <= 1e-6:
                continue
            finite_this_cloud += 1
            self.finite_point_count += 1
            self.ranges.append(distance)
            self.horizontal_ranges.append(radius_xy)
            self.z_values.append(z)
            for index, (lower, upper) in enumerate(
                    zip(RANGE_EDGES_M, RANGE_EDGES_M[1:])):
                if lower <= distance < upper:
                    self.range_histogram[index] += 1
                    break
            elevation_deg = math.degrees(math.atan2(z, radius_xy))
            # 0.25 degree bins reveal the beam pattern even though ring is absent.
            elevation_bin = round(elevation_deg * 4.0) / 4.0
            self.elevation_histogram[elevation_bin] += 1
            for limit in CANDIDATE_MAX_RANGE_M:
                if 2.0 <= distance <= limit:
                    candidate_this_cloud[limit] += 1
                    self.candidate_counts[limit] += 1
            if z > 8.0:
                self.high_point_counts["z_gt_8"] += 1
                if distance > 45.0:
                    self.high_point_counts["z_gt_8_range_gt_45"] += 1
            if z < -4.0:
                self.high_point_counts["z_lt_minus_4"] += 1
        if finite_this_cloud:
            for limit in CANDIDATE_MAX_RANGE_M:
                self.per_cloud_retention[limit].append(
                    candidate_this_cloud[limit] / finite_this_cloud)

    def report(self):
        print("\n=== HELIOS-32 GEOMETRIC WORKSET ===")
        print(f"clouds_total={self.cloud_count} sampled_clouds="
              f"{self.sampled_cloud_count} scan_step={self.scan_step}")
        print(f"sampled_points={self.sampled_point_count} finite="
              f"{self.finite_point_count} nonfinite={self.nonfinite_point_count}")
        for name, values in (("range_3d_m", self.ranges),
                             ("range_xy_m", self.horizontal_ranges),
                             ("z_lidar_m", self.z_values)):
            if values:
                print(f"{name}: min={min(values):.3f} "
                      f"p50={statistics.median(values):.3f} "
                      f"p90={percentile(values, 0.90):.3f} "
                      f"p95={percentile(values, 0.95):.3f} "
                      f"p99={percentile(values, 0.99):.3f} "
                      f"max={max(values):.3f}")
        print("range_histogram:")
        for lower, upper, count in zip(
                RANGE_EDGES_M, RANGE_EDGES_M[1:], self.range_histogram):
            ratio = count / max(self.finite_point_count, 1)
            upper_label = "inf" if math.isinf(upper) else f"{upper:g}"
            print(f"  [{lower:g},{upper_label})m count={count} ratio={ratio:.4%}")
        print("candidate_spherical_gate (min=2m):")
        for limit in CANDIDATE_MAX_RANGE_M:
            count = self.candidate_counts[limit]
            ratios = self.per_cloud_retention[limit]
            print(f"  max={limit:g}m count={count} global="
                  f"{count / max(self.finite_point_count, 1):.4%} "
                  f"per_cloud_p10={percentile(ratios, 0.10):.4%} "
                  f"p50={percentile(ratios, 0.50):.4%} "
                  f"p90={percentile(ratios, 0.90):.4%}")
        print("vertical_extremes:", dict(self.high_point_counts))
        print("elevation_bins_top40 (deg,count):")
        for angle, count in self.elevation_histogram.most_common(40):
            print(f"  {angle:+7.2f} {count}")


def analyze_file(path, stats):
    header = read_header(path)
    if header.compress != record_pb2.COMPRESS_NONE:
        raise RuntimeError("compressed Cyber records are not supported")
    print(f"record={path} duration_sec="
          f"{(header.end_time-header.begin_time)*1e-9:.6f}")
    with open(path, "rb") as stream:
        stream.seek(SECTION_STRUCT.size + HEADER_RESERVED_BYTES)
        while True:
            raw = stream.read(SECTION_STRUCT.size)
            if not raw:
                break
            if len(raw) != SECTION_STRUCT.size:
                raise RuntimeError(f"truncated section header at {stream.tell()}")
            section_type, section_size = SECTION_STRUCT.unpack(raw)
            if section_size < 0:
                raise RuntimeError(f"negative section size {section_size}")
            if section_type != record_pb2.SECTION_CHUNK_BODY:
                stream.seek(section_size, os.SEEK_CUR)
                continue
            payload = stream.read(section_size)
            if len(payload) != section_size:
                raise RuntimeError("truncated chunk body")
            chunk = record_pb2.ChunkBody()
            chunk.ParseFromString(payload)
            for message in chunk.messages:
                if message.channel_name != TOP_CLOUD:
                    continue
                cloud = pointcloud_pb2.PointCloud()
                cloud.ParseFromString(message.content)
                stats.add_cloud(cloud)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("records", nargs="+", help="Cyber record file(s)")
    parser.add_argument("--scan-step", type=int, default=20,
                        help="fully inspect every Nth top-LiDAR cloud")
    args = parser.parse_args()
    stats = WorksetStats(max(args.scan_step, 1))
    for path in args.records:
        analyze_file(path, stats)
    stats.report()
    return 0


if __name__ == "__main__":
    sys.exit(main())
