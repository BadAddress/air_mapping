#!/usr/bin/env python3
"""Offline integrity and sensor-convention checks for Apollo Cyber records.

This reader intentionally parses the Cyber record sections directly instead of
using cyber_py3.record.RecordReader.  The latter is not compatible with the
Python 3.10 runtime shipped by some Apollo 10 package environments.
"""

import argparse
import bisect
import collections
import math
import os
import statistics
import struct
import sys

from cyber.proto import record_pb2
from modules.common_msgs.localization_msgs import gps_pb2
from modules.common_msgs.localization_msgs import imu_pb2 as corrected_imu_pb2
from modules.common_msgs.sensor_msgs import gnss_best_pose_pb2
from modules.common_msgs.sensor_msgs import heading_pb2
from modules.common_msgs.sensor_msgs import imu_pb2
from modules.common_msgs.sensor_msgs import ins_pb2
from modules.common_msgs.sensor_msgs import pointcloud_pb2
from modules.common_msgs.transform_msgs import transform_pb2


SECTION_STRUCT = struct.Struct("<i4xq")
HEADER_RESERVED_BYTES = 2048

BEST_POSE = "/apollo/sensor/gnss/best_pose"
HEADING = "/apollo/sensor/gnss/heading"
CORRECTED_IMU = "/apollo/sensor/gnss/corrected_imu"
RAW_IMU = "/apollo/sensor/gnss/imu"
GPS_ODOMETRY = "/apollo/sensor/gnss/odometry"
INS_STAT = "/apollo/sensor/gnss/ins_stat"
TOP_CLOUD = "/apollo/sensor/rslidar/top/PointCloud2"
FRONT_CLOUD = "/apollo/sensor/rslidar/front/PointCloud2"
TF_STATIC = "/tf_static"


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


def stats(values):
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return "n=0"
    return (f"n={len(finite)} min={min(finite):.6g} "
            f"p50={statistics.median(finite):.6g} "
            f"p95={percentile(finite, 0.95):.6g} "
            f"max={max(finite):.6g}")


def wrap_deg(value):
    return (value + 180.0) % 360.0 - 180.0


def enum_label(message, field_name):
    field = message.DESCRIPTOR.fields_by_name[field_name]
    value = getattr(message, field_name)
    if field.enum_type is None:
        return str(value)
    item = field.enum_type.values_by_number.get(value)
    return f"{item.name if item else 'UNKNOWN'}({value})"


def point_components(point):
    return (point.x, point.y, point.z)


def quaternion_yaw_deg(quaternion):
    siny = 2.0 * (quaternion.qw * quaternion.qz +
                  quaternion.qx * quaternion.qy)
    cosy = 1.0 - 2.0 * (quaternion.qy * quaternion.qy +
                       quaternion.qz * quaternion.qz)
    return math.degrees(math.atan2(siny, cosy))


class Series:
    def __init__(self):
        self.record_ns = []
        self.sensor_sec = []
        self.sequence = []

    def add(self, record_ns, sensor_sec=None, sequence=None):
        self.record_ns.append(record_ns)
        if sensor_sec is not None:
            self.sensor_sec.append(sensor_sec)
        if sequence is not None:
            self.sequence.append(sequence)

    def report(self, name):
        record_dt = [(right - left) * 1e-9 for left, right in
                     zip(self.record_ns, self.record_ns[1:])]
        sensor_dt = [right - left for left, right in
                     zip(self.sensor_sec, self.sensor_sec[1:])]
        print(f"[{name}] count={len(self.record_ns)}")
        if record_dt:
            nonmono = sum(value <= 0.0 for value in record_dt)
            median = statistics.median(record_dt)
            gaps = sum(value > 3.0 * median for value in record_dt)
            print(f"  record_dt_sec: {stats(record_dt)} "
                  f"non_monotonic={nonmono} gaps_gt_3x_median={gaps}")
        if sensor_dt:
            nonmono = sum(value <= 0.0 for value in sensor_dt)
            median = statistics.median(sensor_dt)
            gaps = sum(value > 3.0 * median for value in sensor_dt)
            print(f"  sensor_dt_sec: {stats(sensor_dt)} "
                  f"non_monotonic={nonmono} gaps_gt_3x_median={gaps}")
        if len(self.sensor_sec) == len(self.record_ns):
            delay = [record * 1e-9 - sensor for record, sensor in
                     zip(self.record_ns, self.sensor_sec)]
            print(f"  record_minus_sensor_sec: {stats(delay)}")
        if len(self.sequence) > 1:
            increments = [right - left for left, right in
                          zip(self.sequence, self.sequence[1:])]
            print(f"  sequence: first={self.sequence[0]} "
                  f"last={self.sequence[-1]} "
                  f"non_unit_steps={sum(value != 1 for value in increments)}")


class CloudData:
    def __init__(self):
        self.series = Series()
        self.measurement_sec = []
        self.point_counts = []
        self.width_mismatch = 0
        self.empty = 0
        self.reversed_endpoint_time = 0
        self.endpoint_span_sec = []
        self.measurement_minus_last_sec = []
        self.full_scan_span_sec = []
        self.nonfinite_points = 0
        self.sampled_points = 0
        self.xyz_min = [float("inf")] * 3
        self.xyz_max = [float("-inf")] * 3

    def add(self, record_ns, cloud, index):
        header_time = cloud.header.timestamp_sec
        sequence = cloud.header.sequence_num
        measurement = cloud.measurement_time
        sensor_time = measurement if measurement > 0.0 else header_time
        self.series.add(record_ns, sensor_time, sequence)
        self.measurement_sec.append(sensor_time)
        count = len(cloud.point)
        self.point_counts.append(count)
        if cloud.width and cloud.width * max(cloud.height, 1) != count:
            self.width_mismatch += 1
        if not count:
            self.empty += 1
            return

        first_ns = cloud.point[0].timestamp
        last_ns = cloud.point[-1].timestamp
        if last_ns < first_ns:
            self.reversed_endpoint_time += 1
        self.endpoint_span_sec.append((last_ns - first_ns) * 1e-9)
        self.measurement_minus_last_sec.append(sensor_time - last_ns * 1e-9)

        # A full point traversal every 25 scans catches unsorted timestamps,
        # NaN/Inf coordinates and coordinate-range problems without making the
        # preflight as expensive as map generation.
        if index % 25 != 0:
            return
        min_time = first_ns
        max_time = first_ns
        for point in cloud.point:
            min_time = min(min_time, point.timestamp)
            max_time = max(max_time, point.timestamp)
            xyz = point_components(point)
            self.sampled_points += 1
            if not all(math.isfinite(value) for value in xyz):
                self.nonfinite_points += 1
                continue
            for axis, value in enumerate(xyz):
                self.xyz_min[axis] = min(self.xyz_min[axis], value)
                self.xyz_max[axis] = max(self.xyz_max[axis], value)
        self.full_scan_span_sec.append((max_time - min_time) * 1e-9)

    def report(self, name):
        self.series.report(name)
        print(f"  points_per_cloud: {stats(self.point_counts)} empty={self.empty} "
              f"width_mismatch={self.width_mismatch}")
        print(f"  endpoint_point_span_sec: {stats(self.endpoint_span_sec)} "
              f"reversed={self.reversed_endpoint_time}")
        print(f"  sampled_full_point_span_sec: {stats(self.full_scan_span_sec)}")
        print(f"  measurement_minus_last_point_sec: "
              f"{stats(self.measurement_minus_last_sec)}")
        if self.sampled_points:
            print(f"  sampled_points={self.sampled_points} "
                  f"nonfinite={self.nonfinite_points} "
                  f"xyz_min={self.xyz_min} xyz_max={self.xyz_max}")


class Analyzer:
    def __init__(self):
        self.channel_counts = collections.Counter()
        self.global_record_times = []
        self.parse_failures = collections.Counter()
        self.best_pose_series = Series()
        self.best_poses = []
        self.heading_series = Series()
        self.headings = []
        self.corrected_series = Series()
        self.corrected_accel = [[], [], []]
        self.corrected_gyro = [[], [], []]
        self.corrected_accel_norm = []
        self.corrected_gyro_norm = []
        self.raw_imu_series = Series()
        self.raw_accel_norm = []
        self.raw_gyro_norm = []
        self.gps_odom_series = Series()
        self.gps_odometry = []
        self.ins_stat_series = Series()
        self.ins_status = collections.Counter()
        self.ins_pos_type = collections.Counter()
        self.clouds = {TOP_CLOUD: CloudData(), FRONT_CLOUD: CloudData()}
        self.tf_static = {}

    def parse(self, channel, content, record_ns):
        self.channel_counts[channel] += 1
        try:
            if channel == BEST_POSE:
                msg = gnss_best_pose_pb2.GnssBestPose()
                msg.ParseFromString(content)
                sensor_time = msg.measurement_time or msg.header.timestamp_sec
                self.best_pose_series.add(record_ns, sensor_time,
                                          msg.header.sequence_num)
                self.best_poses.append((sensor_time, msg.latitude, msg.longitude,
                                        msg.height_msl, msg.sol_status,
                                        msg.sol_type, msg.latitude_std_dev,
                                        msg.longitude_std_dev,
                                        msg.height_std_dev,
                                        enum_label(msg, "sol_status"),
                                        enum_label(msg, "sol_type")))
            elif channel == HEADING:
                msg = heading_pb2.Heading()
                msg.ParseFromString(content)
                sensor_time = msg.measurement_time or msg.header.timestamp_sec
                self.heading_series.add(record_ns, sensor_time,
                                        msg.header.sequence_num)
                self.headings.append((sensor_time, msg.heading,
                                      msg.heading_std_dev, msg.solution_status,
                                      msg.position_type))
            elif channel == CORRECTED_IMU:
                msg = corrected_imu_pb2.CorrectedImu()
                msg.ParseFromString(content)
                sensor_time = msg.header.timestamp_sec
                self.corrected_series.add(record_ns, sensor_time,
                                          msg.header.sequence_num)
                accel = point_components(msg.imu.linear_acceleration)
                gyro = point_components(msg.imu.angular_velocity)
                for axis in range(3):
                    self.corrected_accel[axis].append(accel[axis])
                    self.corrected_gyro[axis].append(gyro[axis])
                self.corrected_accel_norm.append(math.sqrt(sum(v*v for v in accel)))
                self.corrected_gyro_norm.append(math.sqrt(sum(v*v for v in gyro)))
            elif channel == RAW_IMU:
                msg = imu_pb2.Imu()
                msg.ParseFromString(content)
                sensor_time = msg.measurement_time or msg.header.timestamp_sec
                self.raw_imu_series.add(record_ns, sensor_time,
                                        msg.header.sequence_num)
                accel = point_components(msg.linear_acceleration)
                gyro = point_components(msg.angular_velocity)
                self.raw_accel_norm.append(math.sqrt(sum(v*v for v in accel)))
                self.raw_gyro_norm.append(math.sqrt(sum(v*v for v in gyro)))
            elif channel == GPS_ODOMETRY:
                msg = gps_pb2.Gps()
                msg.ParseFromString(content)
                sensor_time = msg.header.timestamp_sec
                self.gps_odom_series.add(record_ns, sensor_time,
                                         msg.header.sequence_num)
                pose = msg.localization
                self.gps_odometry.append(
                    (sensor_time, pose.position.x, pose.position.y,
                     pose.position.z, math.degrees(pose.heading),
                     quaternion_yaw_deg(pose.orientation)))
            elif channel == INS_STAT:
                msg = ins_pb2.InsStat()
                msg.ParseFromString(content)
                self.ins_stat_series.add(record_ns, msg.header.timestamp_sec,
                                         msg.header.sequence_num)
                self.ins_status[msg.ins_status] += 1
                self.ins_pos_type[msg.pos_type] += 1
            elif channel in self.clouds:
                msg = pointcloud_pb2.PointCloud()
                msg.ParseFromString(content)
                self.clouds[channel].add(
                    record_ns, msg, self.channel_counts[channel] - 1)
            elif channel == TF_STATIC:
                msg = transform_pb2.TransformStampeds()
                msg.ParseFromString(content)
                for transform in msg.transforms:
                    key = (transform.header.frame_id, transform.child_frame_id)
                    self.tf_static[key] = transform
        except Exception as error:  # continue auditing the rest of the record
            self.parse_failures[(channel, type(error).__name__)] += 1

    def report(self):
        print("\n=== MESSAGE ORDER / PARSING ===")
        nonmono = sum(right < left for left, right in
                      zip(self.global_record_times, self.global_record_times[1:]))
        equal = sum(right == left for left, right in
                    zip(self.global_record_times, self.global_record_times[1:]))
        print(f"selected_messages={len(self.global_record_times)} "
              f"global_record_time_backwards={nonmono} equal={equal}")
        print(f"parse_failures={dict(self.parse_failures)}")

        print("\n=== TIME SERIES ===")
        self.best_pose_series.report("best_pose")
        self.heading_series.report("heading")
        self.corrected_series.report("corrected_imu")
        self.raw_imu_series.report("raw_imu")
        self.gps_odom_series.report("gps_odometry")
        self.ins_stat_series.report("ins_stat")
        self.clouds[TOP_CLOUD].report("top_cloud")
        self.clouds[FRONT_CLOUD].report("front_cloud")

        print("\n=== DUAL LIDAR SYNC ===")
        top = self.clouds[TOP_CLOUD].measurement_sec
        front = self.clouds[FRONT_CLOUD].measurement_sec
        nearest = []
        for stamp in top:
            index = bisect.bisect_left(front, stamp)
            choices = front[max(0, index - 1):min(len(front), index + 1)]
            if choices:
                nearest.append(min(abs(stamp - other) for other in choices))
        paired = sum(value <= 0.05 for value in nearest)
        print(f"top_to_nearest_front_abs_sec: {stats(nearest)} "
              f"within_50ms={paired}/{len(nearest)}")

        print("\n=== GNSS QUALITY ===")
        status = collections.Counter(row[9] for row in self.best_poses)
        sol_type = collections.Counter(row[10] for row in self.best_poses)
        print(f"best_pose_status={dict(status)}")
        print(f"best_pose_type={dict(sol_type)}")
        print("best_pose_lat_std_m:", stats([row[6] for row in self.best_poses]))
        print("best_pose_lon_std_m:", stats([row[7] for row in self.best_poses]))
        print("best_pose_height_std_m:", stats([row[8] for row in self.best_poses]))
        if self.best_poses:
            latitude = [row[1] for row in self.best_poses]
            longitude = [row[2] for row in self.best_poses]
            print(f"lat_range=[{min(latitude):.10f}, {max(latitude):.10f}] "
                  f"lon_range=[{min(longitude):.10f}, {max(longitude):.10f}]")

        print(f"ins_status_counts={dict(self.ins_status)}")
        print(f"ins_pos_type_counts={dict(self.ins_pos_type)}")

        print("\n=== HEADING / COURSE CONVENTION ===")
        heading_status = collections.Counter(row[3] for row in self.headings)
        heading_type = collections.Counter(row[4] for row in self.headings)
        print(f"heading_solution_status={dict(heading_status)}")
        print(f"heading_position_type={dict(heading_type)}")
        print("heading_deg:", stats([row[1] for row in self.headings]))
        print("heading_std_deg:", stats([row[2] for row in self.headings]))
        self.report_heading_models()

        print("\n=== IMU VALUES ===")
        axes = "xyz"
        for axis in range(3):
            print(f"corrected_accel_{axes[axis]}: "
                  f"{stats(self.corrected_accel[axis])}")
        print("corrected_accel_norm:", stats(self.corrected_accel_norm))
        for axis in range(3):
            print(f"corrected_gyro_{axes[axis]}: "
                  f"{stats(self.corrected_gyro[axis])}")
        print("corrected_gyro_norm:", stats(self.corrected_gyro_norm))
        print("raw_accel_norm:", stats(self.raw_accel_norm))
        print("raw_gyro_norm:", stats(self.raw_gyro_norm))

        print("\n=== TF_STATIC (last value per frame pair) ===")
        if not self.tf_static:
            print("none")
        for key in sorted(self.tf_static):
            transform = self.tf_static[key].transform
            translation = transform.translation
            rotation = transform.rotation
            print(f"{key[0]} -> {key[1]}: "
                  f"t=[{translation.x:.9g}, {translation.y:.9g}, "
                  f"{translation.z:.9g}] "
                  f"q_xyzw=[{rotation.qx:.9g}, {rotation.qy:.9g}, "
                  f"{rotation.qz:.9g}, {rotation.qw:.9g}]")

    def report_heading_models(self):
        if len(self.best_poses) < 3 or not self.headings:
            print("insufficient best_pose/heading samples for course comparison")
            return
        heading_times = [row[0] for row in self.headings]
        comparisons = []
        earth_east_per_lon = (111319.49079327358 *
                              math.cos(math.radians(self.best_poses[0][1])))
        earth_north_per_lat = 110574.0
        # Use a centered multi-sample baseline and reject displacements below
        # 1 m, because course from 1 Hz GNSS is unstable while stationary.
        for index in range(1, len(self.best_poses) - 1):
            left = self.best_poses[index - 1]
            right = self.best_poses[index + 1]
            east = (right[2] - left[2]) * earth_east_per_lon
            north = (right[1] - left[1]) * earth_north_per_lat
            distance = math.hypot(east, north)
            if distance < 1.0:
                continue
            time = self.best_poses[index][0]
            heading_index = bisect.bisect_left(heading_times, time)
            heading_index = min(max(heading_index, 0), len(self.headings) - 1)
            if heading_index and abs(heading_times[heading_index - 1] - time) < \
                    abs(heading_times[heading_index] - time):
                heading_index -= 1
            raw_heading = self.headings[heading_index][1]
            course = math.degrees(math.atan2(north, east))
            comparisons.append((course, raw_heading, distance))
        if not comparisons:
            print("no GNSS displacement baseline above 1 m")
            return
        models = {
            "raw_as_ENU_forward(+h)": lambda heading: heading,
            "raw_as_ENU_right(+h-90)": lambda heading: heading - 90.0,
            "NovAtel_forward(90-h)": lambda heading: 90.0 - heading,
            "NovAtel_RFU_right(-h)": lambda heading: -heading,
        }
        print(f"course_comparisons={len(comparisons)} "
              "(target is GNSS motion course, East=0 CCW)")
        for name, model in models.items():
            errors = [abs(wrap_deg(model(heading) - course))
                      for course, heading, _ in comparisons]
            opposite_errors = [abs(wrap_deg(model(heading) + 180.0 - course))
                               for course, heading, _ in comparisons]
            print(f"  {name}: abs_error_deg {stats(errors)}; "
                  f"with_180_flip {stats(opposite_errors)}")
        print("  samples(course_deg, raw_heading_deg, baseline_m):")
        for sample in comparisons[:10]:
            print(f"    ({sample[0]:.3f}, {sample[1]:.3f}, {sample[2]:.3f})")
        if self.gps_odometry:
            odom_times = [row[0] for row in self.gps_odometry]
            raw_vs_odom = []
            raw_vs_quaternion = []
            for time, raw_heading, *_ in self.headings[::50]:
                index = bisect.bisect_left(odom_times, time)
                if index >= len(odom_times):
                    index = len(odom_times) - 1
                row = self.gps_odometry[index]
                raw_vs_odom.append(wrap_deg(row[4] - raw_heading))
                raw_vs_quaternion.append(wrap_deg(row[5] - raw_heading))
            print("  gps_odometry_heading_deg_minus_raw_heading_deg:",
                  stats(raw_vs_odom))
            print("  gps_odometry_quaternion_yaw_deg_minus_raw_heading_deg:",
                  stats(raw_vs_quaternion))


def read_header(path):
    with open(path, "rb") as stream:
        raw = stream.read(SECTION_STRUCT.size)
        if len(raw) != SECTION_STRUCT.size:
            raise RuntimeError("record is shorter than a section header")
        section_type, section_size = SECTION_STRUCT.unpack(raw)
        if section_type != record_pb2.SECTION_HEADER:
            raise RuntimeError(f"first section type is {section_type}, not header")
        payload = stream.read(section_size)
    header = record_pb2.Header()
    header.ParseFromString(payload)
    return header


def analyze_file(path, analyzer):
    header = read_header(path)
    if header.compress != record_pb2.COMPRESS_NONE:
        raise RuntimeError("compressed Cyber records are not supported")
    print(f"record={path}")
    print(f"size_header={header.size} size_actual={os.path.getsize(path)} "
          f"complete={header.is_complete} messages={header.message_number} "
          f"channels={header.channel_number} chunks={header.chunk_number} "
          f"duration_sec={(header.end_time-header.begin_time)*1e-9:.6f}")
    selected = {BEST_POSE, HEADING, CORRECTED_IMU, RAW_IMU, GPS_ODOMETRY,
                INS_STAT, TOP_CLOUD, FRONT_CLOUD, TF_STATIC}
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
                if message.channel_name not in selected:
                    continue
                analyzer.global_record_times.append(message.time)
                analyzer.parse(message.channel_name, message.content,
                               message.time)
    return header


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("records", nargs="+", help="Cyber record file(s)")
    args = parser.parse_args()
    analyzer = Analyzer()
    previous = None
    for path in args.records:
        header = analyze_file(path, analyzer)
        if previous is not None:
            delta = (header.begin_time - previous.end_time) * 1e-9
            print(f"record_boundary_delta_sec={delta:.9f}")
        previous = header
    analyzer.report()


if __name__ == "__main__":
    sys.exit(main())
