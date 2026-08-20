#!/usr/bin/env python3
"""Plot a relative-UTM trajectory and sensor diagnostics from Cyber records.

BestPose latitude/longitude are projected from WGS84 to the UTM zone selected
from the first GNSS sample.  X/Y/Z in the CSV and plots are then translated by
the first BestPose, so the first row is exactly (0, 0, 0).  Z uses height_msl.

The Cyber section parser is used directly because Apollo 10 package images
based on Python 3.10 can fail in cyber_py3.record.RecordReader.
"""

import argparse
import csv
import math
import os
import pathlib
import struct

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pyproj import CRS, Transformer

from cyber.proto import record_pb2
from modules.common_msgs.chassis_msgs import chassis_pb2
from modules.common_msgs.localization_msgs import imu_pb2 as corrected_imu_pb2
from modules.common_msgs.sensor_msgs import gnss_best_pose_pb2
from modules.common_msgs.sensor_msgs import heading_pb2


SECTION_STRUCT = struct.Struct("<i4xq")
HEADER_RESERVED_BYTES = 2048

BEST_POSE_TOPIC = "/apollo/sensor/gnss/best_pose"
HEADING_TOPIC = "/apollo/sensor/gnss/heading"
CHASSIS_TOPIC = "/apollo/canbus/chassis"
CORRECTED_IMU_TOPIC = "/apollo/sensor/gnss/corrected_imu"


def wrap_deg(values):
    return (values + 180.0) % 360.0 - 180.0


def sensor_time(message):
    measurement_time = getattr(message, "measurement_time", 0.0)
    if measurement_time:
        return measurement_time
    return message.header.timestamp_sec


def iter_selected_messages(path, selected_topics):
    """Yield (topic, record_time_ns, serialized_content) from one record."""
    with open(path, "rb") as stream:
        first = stream.read(SECTION_STRUCT.size)
        if len(first) != SECTION_STRUCT.size:
            raise RuntimeError(f"{path}: incomplete record header")
        section_type, section_size = SECTION_STRUCT.unpack(first)
        if section_type != record_pb2.SECTION_HEADER:
            raise RuntimeError(f"{path}: first section is not SECTION_HEADER")
        header = record_pb2.Header()
        header.ParseFromString(stream.read(section_size))
        if header.compress != record_pb2.COMPRESS_NONE:
            raise RuntimeError(f"{path}: compressed record is not supported")
        if not header.is_complete:
            print(f"WARNING: record header marks file incomplete: {path}")
        stream.seek(SECTION_STRUCT.size + HEADER_RESERVED_BYTES)

        while True:
            raw = stream.read(SECTION_STRUCT.size)
            if not raw:
                break
            if len(raw) != SECTION_STRUCT.size:
                raise RuntimeError(f"{path}: truncated section header")
            section_type, section_size = SECTION_STRUCT.unpack(raw)
            if section_size < 0:
                raise RuntimeError(f"{path}: negative section size")
            if section_type != record_pb2.SECTION_CHUNK_BODY:
                stream.seek(section_size, os.SEEK_CUR)
                continue
            payload = stream.read(section_size)
            if len(payload) != section_size:
                raise RuntimeError(f"{path}: truncated chunk body")
            chunk = record_pb2.ChunkBody()
            chunk.ParseFromString(payload)
            for message in chunk.messages:
                if message.channel_name in selected_topics:
                    yield message.channel_name, message.time, message.content


def read_records(paths):
    best_pose = []
    headings = []
    chassis = []
    imu = []
    selected = {BEST_POSE_TOPIC, HEADING_TOPIC, CHASSIS_TOPIC,
                CORRECTED_IMU_TOPIC}
    for path in paths:
        print(f"Reading {path}")
        for topic, record_ns, content in iter_selected_messages(path, selected):
            if topic == BEST_POSE_TOPIC:
                message = gnss_best_pose_pb2.GnssBestPose()
                message.ParseFromString(content)
                best_pose.append({
                    "time": sensor_time(message),
                    "record_time": record_ns * 1e-9,
                    "latitude": message.latitude,
                    "longitude": message.longitude,
                    "height_msl": message.height_msl,
                    "sol_status": message.sol_status,
                    "sol_type": message.sol_type,
                    "lat_std": message.latitude_std_dev,
                    "lon_std": message.longitude_std_dev,
                    "height_std": message.height_std_dev,
                })
            elif topic == HEADING_TOPIC:
                message = heading_pb2.Heading()
                message.ParseFromString(content)
                headings.append({
                    "time": sensor_time(message),
                    "heading": message.heading,
                    "heading_std": message.heading_std_dev,
                    "solution_status": message.solution_status,
                    "position_type": message.position_type,
                })
            elif topic == CHASSIS_TOPIC:
                message = chassis_pb2.Chassis()
                message.ParseFromString(content)
                chassis.append({
                    "time": message.header.timestamp_sec,
                    "speed_mps": message.speed_mps,
                    "gear": message.gear_location,
                    "steering_pct": message.steering_percentage,
                })
            elif topic == CORRECTED_IMU_TOPIC:
                message = corrected_imu_pb2.CorrectedImu()
                message.ParseFromString(content)
                imu.append({
                    "time": message.header.timestamp_sec,
                    "gyro_z": message.imu.angular_velocity.z,
                    "accel_x": message.imu.linear_acceleration.x,
                    "accel_y": message.imu.linear_acceleration.y,
                    "accel_z": message.imu.linear_acceleration.z,
                })
    best_pose.sort(key=lambda row: row["time"])
    headings.sort(key=lambda row: row["time"])
    chassis.sort(key=lambda row: row["time"])
    imu.sort(key=lambda row: row["time"])
    if not best_pose:
        raise RuntimeError(f"no {BEST_POSE_TOPIC} messages found")
    return best_pose, headings, chassis, imu


def project_utm(best_pose):
    first = best_pose[0]
    zone = int(math.floor((first["longitude"] + 180.0) / 6.0)) + 1
    zone = min(max(zone, 1), 60)
    north = first["latitude"] >= 0.0
    epsg = (32600 if north else 32700) + zone
    crs = CRS.from_epsg(epsg)
    transformer = Transformer.from_crs("EPSG:4326", crs, always_xy=True)

    longitude = np.array([row["longitude"] for row in best_pose])
    latitude = np.array([row["latitude"] for row in best_pose])
    zones = np.floor((longitude + 180.0) / 6.0).astype(int) + 1
    if np.any(zones != zone):
        raise RuntimeError("trajectory crosses a UTM zone boundary; one-offset "
                           "UTM output would be ambiguous")
    easting, northing = transformer.transform(longitude, latitude)
    height = np.array([row["height_msl"] for row in best_pose])
    xyz = np.column_stack((easting, northing, height))
    relative = xyz - xyz[0]
    return zone, north, epsg, crs.name, xyz, relative


def interpolate_heading(best_pose, headings):
    count = len(best_pose)
    if not headings:
        return np.full(count, np.nan), np.full(count, np.nan)
    query = np.array([row["time"] for row in best_pose])
    source_time = np.array([row["time"] for row in headings])
    source_heading = np.array([row["heading"] for row in headings])
    source_unwrapped = np.unwrap(np.deg2rad(source_heading))
    interpolated = np.rad2deg(np.interp(query, source_time, source_unwrapped))
    interpolated = wrap_deg(interpolated)
    source_std = np.array([row["heading_std"] for row in headings])
    std = np.interp(query, source_time, source_std)
    return interpolated, std


def motion_course(relative):
    course = np.full(len(relative), np.nan)
    baseline = np.zeros(len(relative))
    if len(relative) < 2:
        return course, baseline
    for index in range(len(relative)):
        left = max(index - 1, 0)
        right = min(index + 1, len(relative) - 1)
        dx = relative[right, 0] - relative[left, 0]
        dy = relative[right, 1] - relative[left, 1]
        distance = math.hypot(dx, dy)
        baseline[index] = distance
        if distance >= 0.5:
            # ENU mathematical yaw: East=0 deg, North=+90 deg, CCW positive.
            course[index] = math.degrees(math.atan2(dy, dx))
    return course, baseline


def write_csv(path, best_pose, zone, hemisphere, xyz, relative,
              heading, heading_std, course, baseline):
    fields = [
        "index", "measurement_time_sec", "elapsed_sec", "latitude_deg",
        "longitude_deg", "height_msl_m", "utm_zone", "utm_hemisphere",
        "utm_easting_m", "utm_northing_m", "utm_z_m", "relative_x_m",
        "relative_y_m", "relative_z_m", "sol_status", "sol_type",
        "latitude_std_dev_m", "longitude_std_dev_m", "height_std_dev_m",
        "raw_heading_deg", "heading_std_dev_deg", "motion_course_deg",
        "course_baseline_m",
    ]
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        first_time = best_pose[0]["time"]
        for index, row in enumerate(best_pose):
            writer.writerow({
                "index": index,
                "measurement_time_sec": f"{row['time']:.9f}",
                "elapsed_sec": f"{row['time'] - first_time:.6f}",
                "latitude_deg": f"{row['latitude']:.10f}",
                "longitude_deg": f"{row['longitude']:.10f}",
                "height_msl_m": f"{row['height_msl']:.6f}",
                "utm_zone": zone,
                "utm_hemisphere": hemisphere,
                "utm_easting_m": f"{xyz[index, 0]:.6f}",
                "utm_northing_m": f"{xyz[index, 1]:.6f}",
                "utm_z_m": f"{xyz[index, 2]:.6f}",
                "relative_x_m": f"{relative[index, 0]:.6f}",
                "relative_y_m": f"{relative[index, 1]:.6f}",
                "relative_z_m": f"{relative[index, 2]:.6f}",
                "sol_status": row["sol_status"],
                "sol_type": row["sol_type"],
                "latitude_std_dev_m": f"{row['lat_std']:.6f}",
                "longitude_std_dev_m": f"{row['lon_std']:.6f}",
                "height_std_dev_m": f"{row['height_std']:.6f}",
                "raw_heading_deg": f"{heading[index]:.6f}",
                "heading_std_dev_deg": f"{heading_std[index]:.6f}",
                "motion_course_deg": f"{course[index]:.6f}",
                "course_baseline_m": f"{baseline[index]:.6f}",
            })


def save_trajectory_plot(path, relative, elapsed, course):
    fig, axis = plt.subplots(figsize=(9, 8), constrained_layout=True)
    scatter = axis.scatter(relative[:, 0], relative[:, 1], c=elapsed,
                           cmap="viridis", s=28, zorder=3)
    axis.plot(relative[:, 0], relative[:, 1], color="0.55", linewidth=1,
              zorder=1)
    axis.scatter(relative[0, 0], relative[0, 1], marker="o", s=100,
                 color="limegreen", edgecolor="black", label="start", zorder=4)
    axis.scatter(relative[-1, 0], relative[-1, 1], marker="X", s=110,
                 color="red", edgecolor="black", label="end", zorder=4)
    valid = np.flatnonzero(np.isfinite(course))[::4]
    if len(valid):
        radians = np.deg2rad(course[valid])
        axis.quiver(relative[valid, 0], relative[valid, 1],
                    np.cos(radians), np.sin(radians), color="tab:orange",
                    angles="xy", scale_units="xy", scale=0.35, width=0.004,
                    alpha=0.75, label="motion direction")
    axis.set_aspect("equal", adjustable="datalim")
    axis.set_xlabel("Relative UTM X / East (m)")
    axis.set_ylabel("Relative UTM Y / North (m)")
    axis.set_title("BestPose trajectory (first UTM sample = origin)")
    axis.grid(True, alpha=0.3)
    axis.legend()
    colorbar = fig.colorbar(scatter, ax=axis)
    colorbar.set_label("Elapsed time (s)")
    fig.savefig(path, dpi=180)
    plt.close(fig)


def save_xyz_plot(path, relative, elapsed):
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True,
                             constrained_layout=True)
    labels = ["X / East", "Y / North", "Z / height_msl"]
    colors = ["tab:blue", "tab:orange", "tab:green"]
    for axis, values, label, color in zip(axes, relative.T, labels, colors):
        axis.plot(elapsed, values, marker=".", markersize=4, color=color)
        axis.set_ylabel(f"{label} (m)")
        axis.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Elapsed time (s)")
    fig.suptitle("Relative UTM coordinates versus time")
    fig.savefig(path, dpi=180)
    plt.close(fig)


def save_heading_plot(path, elapsed, raw_heading, course, heading_std):
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True,
                             constrained_layout=True)
    axes[0].plot(elapsed, raw_heading, "o-", markersize=3,
                 label="raw Heading.heading")
    axes[0].plot(elapsed, wrap_deg(raw_heading + 90.0), "o-", markersize=3,
                 label="raw heading + 90 deg")
    axes[0].plot(elapsed, course, "k.-", markersize=5,
                 label="GNSS motion course")
    axes[0].set_ylabel("Angle (deg, East=0 CCW)")
    axes[0].set_ylim(-190, 190)
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")
    error = wrap_deg(raw_heading + 90.0 - course)
    axes[1].plot(elapsed, error, label="(raw + 90) - motion course")
    axes[1].plot(elapsed, heading_std, label="reported heading std")
    axes[1].axhline(0.0, color="black", linewidth=0.8)
    axes[1].set_ylabel("Difference / std (deg)")
    axes[1].set_xlabel("Elapsed time (s)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best")
    fig.suptitle("Raw heading and motion-course convention check")
    fig.savefig(path, dpi=180)
    plt.close(fig)


def save_quality_plot(path, best_pose, elapsed, headings):
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True,
                             constrained_layout=True)
    axes[0].plot(elapsed, [row["lat_std"] for row in best_pose],
                 label="latitude std")
    axes[0].plot(elapsed, [row["lon_std"] for row in best_pose],
                 label="longitude std")
    axes[0].plot(elapsed, [row["height_std"] for row in best_pose],
                 label="height std")
    axes[0].set_ylabel("BestPose std (m)")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[1].step(elapsed, [row["sol_status"] for row in best_pose],
                 where="mid", label="sol_status")
    axes[1].step(elapsed, [row["sol_type"] for row in best_pose],
                 where="mid", label="sol_type")
    axes[1].set_ylabel("GNSS enum value")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    if headings:
        first_time = best_pose[0]["time"]
        heading_elapsed = np.array([row["time"] - first_time for row in headings])
        axes[2].plot(heading_elapsed,
                     [row["heading_std"] for row in headings],
                     label="heading std")
        axes[2].axhline(1.0, color="red", linestyle="--",
                        label="current Stage1 threshold (1 deg)")
    axes[2].set_ylabel("Heading std (deg)")
    axes[2].set_xlabel("Elapsed time (s)")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    fig.suptitle("GNSS quality fields")
    fig.savefig(path, dpi=180)
    plt.close(fig)


def save_motion_plot(path, first_time, chassis, imu):
    if not chassis and not imu:
        return False
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True,
                             constrained_layout=True)
    if chassis:
        elapsed = np.array([row["time"] - first_time for row in chassis])
        axes[0].plot(elapsed, [row["speed_mps"] for row in chassis])
        axes[0].set_ylabel("Chassis speed (m/s)")
        axes[1].step(elapsed, [row["gear"] for row in chassis], where="post")
        axes[1].set_ylabel("Gear enum\n1=D, 2=R")
    if imu:
        elapsed = np.array([row["time"] - first_time for row in imu])
        axes[2].plot(elapsed, [row["gyro_z"] for row in imu])
        axes[2].set_ylabel("IMU gyro Z (rad/s)")
    for axis in axes:
        axis.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Elapsed time (s)")
    fig.suptitle("Vehicle motion diagnostics")
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("records", nargs="+", help="Cyber record file(s)")
    parser.add_argument("--output-dir", required=True,
                        help="Directory for CSV and PNG outputs")
    args = parser.parse_args()

    output_dir = pathlib.Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    best_pose, headings, chassis, imu = read_records(args.records)
    zone, north, epsg, crs_name, xyz, relative = project_utm(best_pose)
    interpolated_heading, interpolated_std = interpolate_heading(
        best_pose, headings)
    course, baseline = motion_course(relative)
    first_time = best_pose[0]["time"]
    elapsed = np.array([row["time"] - first_time for row in best_pose])
    hemisphere = "N" if north else "S"

    csv_path = output_dir / "utm_relative_trajectory.csv"
    write_csv(csv_path, best_pose, zone, hemisphere, xyz, relative,
              interpolated_heading, interpolated_std, course, baseline)
    save_trajectory_plot(output_dir / "trajectory_xy.png", relative, elapsed,
                         course)
    save_xyz_plot(output_dir / "utm_xyz_vs_time.png", relative, elapsed)
    save_heading_plot(output_dir / "heading_course_comparison.png", elapsed,
                      interpolated_heading, course, interpolated_std)
    save_quality_plot(output_dir / "gnss_quality.png", best_pose, elapsed,
                      headings)
    motion_saved = save_motion_plot(output_dir / "motion_diagnostics.png",
                                    first_time, chassis, imu)

    valid_error = wrap_deg(interpolated_heading + 90.0 - course)
    valid_error = valid_error[np.isfinite(valid_error)]
    print(f"BestPose samples: {len(best_pose)}")
    print(f"Heading samples: {len(headings)}")
    print(f"UTM: zone {zone}{hemisphere}, EPSG:{epsg}, {crs_name}")
    print("First absolute UTM/height_msl: "
          f"[{xyz[0, 0]:.6f}, {xyz[0, 1]:.6f}, {xyz[0, 2]:.6f}]")
    print("Last relative XYZ: "
          f"[{relative[-1, 0]:.6f}, {relative[-1, 1]:.6f}, "
          f"{relative[-1, 2]:.6f}]")
    if len(valid_error):
        print("Median circular error of (raw heading + 90 deg) versus "
              f"motion course: {np.median(valid_error):.3f} deg")
        print("Median absolute circular error: "
              f"{np.median(np.abs(valid_error)):.3f} deg")
    print(f"CSV: {csv_path}")
    print("Plots: trajectory_xy.png, utm_xyz_vs_time.png, "
          "heading_course_comparison.png, gnss_quality.png" +
          (", motion_diagnostics.png" if motion_saved else ""))


if __name__ == "__main__":
    main()
