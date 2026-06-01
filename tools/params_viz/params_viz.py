#!/usr/bin/env python3
"""Visualize vehicle extrinsics from all_params.yaml with Open3D."""

from __future__ import annotations

import argparse
from html import escape
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np

try:
    import yaml
except ImportError as exc:  # pragma: no cover - dependency check path
    raise SystemExit(
        "PyYAML is required to read all_params.yaml. Install package: python3-yaml or pyyaml"
    ) from exc


IDENTITY_ROTATION = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]


@dataclass(frozen=True)
class TransformEntry:
    name: str
    parent: str
    child: str
    matrix: np.ndarray
    source: str


def parse_args() -> argparse.Namespace:
    default_config = Path(__file__).with_name("all_params.yaml")
    parser = argparse.ArgumentParser(
        description="Visualize vehicle sensor extrinsics with Open3D.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--config", default=str(default_config), help="Path to all_params.yaml")
    parser.add_argument("--root", default=None, help="Root frame used as world origin")
    parser.add_argument("--frame-size", type=float, default=None, help="Coordinate-frame axis length")
    parser.add_argument("--origin-radius", type=float, default=None, help="Origin marker sphere radius")
    parser.add_argument("--translation-tolerance", type=float, default=1e-3, help="Reference check tolerance in meters")
    parser.add_argument("--rotation-tolerance-deg", type=float, default=1e-3, help="Reference check tolerance in degrees")
    parser.add_argument("--print-only", action="store_true", help="Only print poses and consistency checks")
    parser.add_argument("--strict", action="store_true", help="Return non-zero if any consistency check fails")
    parser.add_argument("--no-lines", action="store_true", help="Do not draw transform connection lines")
    parser.add_argument("--no-labels", action="store_true", help="Use classic Open3D viewer without 3D labels")
    parser.add_argument("--export-html", default=None, help="Write a static top-down HTML/SVG visualization")
    return parser.parse_args()


def read_config(path: Path) -> dict:
    if not path.exists():
        raise FileNotFoundError(f"Config file does not exist: {path}")
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"Config root must be a mapping: {path}")
    return data


def numeric_list(value: object, length: int, field: str) -> List[float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"{field} must be a list with {length} numeric values")
    if len(value) != length:
        raise ValueError(f"{field} must have {length} values, got {len(value)}")
    try:
        return [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} contains a non-numeric value") from exc


def rotation_from_config(entry: dict, field_prefix: str) -> np.ndarray:
    if "rotation" in entry:
        values = numeric_list(entry["rotation"], 9, f"{field_prefix}.rotation")
        return np.array(values, dtype=float).reshape(3, 3)
    if "rotation_matrix" in entry:
        values = numeric_list(entry["rotation_matrix"], 9, f"{field_prefix}.rotation_matrix")
        return np.array(values, dtype=float).reshape(3, 3)
    if "rotation_quaternion_xyzw" in entry:
        qx, qy, qz, qw = numeric_list(
            entry["rotation_quaternion_xyzw"], 4, f"{field_prefix}.rotation_quaternion_xyzw"
        )
        return rotation_from_quaternion_xyzw(qx, qy, qz, qw)
    if "rotation_quaternion_wxyz" in entry:
        qw, qx, qy, qz = numeric_list(
            entry["rotation_quaternion_wxyz"], 4, f"{field_prefix}.rotation_quaternion_wxyz"
        )
        return rotation_from_quaternion_xyzw(qx, qy, qz, qw)
    values = numeric_list(IDENTITY_ROTATION, 9, f"{field_prefix}.rotation")
    return np.array(values, dtype=float).reshape(3, 3)


def rotation_from_quaternion_xyzw(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        raise ValueError("Quaternion norm must be positive")
    x, y, z, w = qx / norm, qy / norm, qz / norm, qw / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def transform_from_config(entry: dict, field_prefix: str) -> np.ndarray:
    translation = numeric_list(entry.get("translation", [0.0, 0.0, 0.0]), 3, f"{field_prefix}.translation")
    rotation = rotation_from_config(entry, field_prefix)
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.array(translation, dtype=float)
    return transform


def parse_transform_entries(entries: Iterable[dict], section: str) -> List[TransformEntry]:
    parsed: List[TransformEntry] = []
    for index, entry in enumerate(entries):
        if not isinstance(entry, dict):
            raise ValueError(f"{section}[{index}] must be a mapping")
        name = str(entry.get("name", f"{section}_{index}"))
        parent = entry.get("parent")
        child = entry.get("child")
        if not parent or not child:
            raise ValueError(f"{section}.{name} must define parent and child")
        matrix = transform_from_config(entry, f"{section}.{name}")
        parsed.append(
            TransformEntry(
                name=name,
                parent=str(parent),
                child=str(child),
                matrix=matrix,
                source=str(entry.get("source", "")),
            )
        )
    return parsed


def invert_transform(transform: np.ndarray) -> np.ndarray:
    inverse = np.eye(4, dtype=float)
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def rotation_error_deg(actual: np.ndarray, expected: np.ndarray) -> float:
    delta = expected[:3, :3].T @ actual[:3, :3]
    value = (float(np.trace(delta)) - 1.0) * 0.5
    value = max(-1.0, min(1.0, value))
    return math.degrees(math.acos(value))


def transform_error(actual: np.ndarray, expected: np.ndarray) -> Tuple[float, float]:
    translation_error = float(np.linalg.norm(actual[:3, 3] - expected[:3, 3]))
    return translation_error, rotation_error_deg(actual, expected)


def build_frame_poses(
    transforms: Sequence[TransformEntry],
    root_frame: str,
    translation_tolerance: float,
    rotation_tolerance_deg: float,
) -> Tuple[Dict[str, np.ndarray], List[TransformEntry], List[str]]:
    poses: Dict[str, np.ndarray] = {root_frame: np.eye(4, dtype=float)}
    unresolved = list(transforms)
    conflicts: List[str] = []

    made_progress = True
    while unresolved and made_progress:
        made_progress = False
        next_unresolved: List[TransformEntry] = []
        for entry in unresolved:
            parent_known = entry.parent in poses
            child_known = entry.child in poses
            if parent_known and not child_known:
                poses[entry.child] = poses[entry.parent] @ entry.matrix
                made_progress = True
            elif child_known and not parent_known:
                poses[entry.parent] = poses[entry.child] @ invert_transform(entry.matrix)
                made_progress = True
            elif parent_known and child_known:
                predicted_child_pose = poses[entry.parent] @ entry.matrix
                trans_err, rot_err = transform_error(poses[entry.child], predicted_child_pose)
                if trans_err > translation_tolerance or rot_err > rotation_tolerance_deg:
                    conflicts.append(
                        f"{entry.name}: graph conflict, translation_error={trans_err:.6f} m, "
                        f"rotation_error={rot_err:.6f} deg"
                    )
                made_progress = True
            else:
                next_unresolved.append(entry)
        unresolved = next_unresolved

    return poses, unresolved, conflicts


def run_reference_checks(
    references: Sequence[TransformEntry],
    poses: Dict[str, np.ndarray],
    translation_tolerance: float,
    rotation_tolerance_deg: float,
) -> Tuple[List[Tuple[str, str, float, float, str]], bool]:
    rows: List[Tuple[str, str, float, float, str]] = []
    ok = True
    for entry in references:
        if entry.parent not in poses or entry.child not in poses:
            rows.append((entry.name, "MISSING_FRAME", math.nan, math.nan, entry.source))
            ok = False
            continue
        actual = invert_transform(poses[entry.parent]) @ poses[entry.child]
        trans_err, rot_err = transform_error(actual, entry.matrix)
        status = "OK" if trans_err <= translation_tolerance and rot_err <= rotation_tolerance_deg else "FAIL"
        rows.append((entry.name, status, trans_err, rot_err, entry.source))
        ok = ok and status == "OK"
    return rows, ok


def ordered_frame_names(config_frames: dict, poses: Dict[str, np.ndarray]) -> List[str]:
    ordered = [name for name in config_frames.keys() if name in poses]
    extra = sorted(name for name in poses.keys() if name not in config_frames)
    return ordered + extra


def print_summary(
    data: dict,
    root_frame: str,
    poses: Dict[str, np.ndarray],
    unresolved: Sequence[TransformEntry],
    conflicts: Sequence[str],
    reference_rows: Sequence[Tuple[str, str, float, float, str]],
) -> None:
    vehicle = data.get("vehicle", "unknown")
    convention = data.get("frame_convention", {})
    axes = convention.get("axes", {})
    print(f"Vehicle: {vehicle}")
    print(f"Root frame: {root_frame}")
    print(
        "Axes: "
        f"x={axes.get('x', 'unknown')} (red), "
        f"y={axes.get('y', 'unknown')} (green), "
        f"z={axes.get('z', 'unknown')} (blue)"
    )
    print("Transform convention: T_parent_child maps child points into parent coordinates")
    print()

    frames = data.get("frames", {})
    print("Frame poses in root frame:")
    print(f"{'frame':<20} {'x[m]':>11} {'y[m]':>11} {'z[m]':>11}")
    for frame_name in ordered_frame_names(frames, poses):
        translation = poses[frame_name][:3, 3]
        print(f"{frame_name:<20} {translation[0]:>11.6f} {translation[1]:>11.6f} {translation[2]:>11.6f}")

    if unresolved:
        print()
        print("Unresolved active transforms:")
        for entry in unresolved:
            print(f"  {entry.name}: {entry.parent} -> {entry.child}")

    if conflicts:
        print()
        print("Graph conflicts:")
        for conflict in conflicts:
            print(f"  {conflict}")

    if reference_rows:
        print()
        print("Reference checks:")
        print(f"{'status':<8} {'name':<48} {'trans[m]':>12} {'rot[deg]':>12}")
        for name, status, trans_err, rot_err, _source in reference_rows:
            trans_text = "nan" if math.isnan(trans_err) else f"{trans_err:.6f}"
            rot_text = "nan" if math.isnan(rot_err) else f"{rot_err:.6f}"
            print(f"{status:<8} {name:<48} {trans_text:>12} {rot_text:>12}")


def color_from_frame_info(info: dict) -> List[float]:
    color = info.get("color", [0.8, 0.8, 0.8])
    values = numeric_list(color, 3, "frames.*.color")
    return [min(1.0, max(0.0, value)) for value in values]


def make_visual_geometries(
    o3d: object,
    data: dict,
    poses: Dict[str, np.ndarray],
    transforms: Sequence[TransformEntry],
    frame_size: float,
    origin_radius: float,
    draw_lines: bool,
) -> Tuple[List[object], List[Tuple[str, np.ndarray]]]:
    geometries: List[object] = []
    labels: List[Tuple[str, np.ndarray]] = []
    frames = data.get("frames", {})

    for frame_name in ordered_frame_names(frames, poses):
        frame_info = frames.get(frame_name, {})
        pose = poses[frame_name]
        size = float(frame_info.get("size", frame_size))
        frame_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size, origin=[0.0, 0.0, 0.0])
        frame_mesh.transform(pose)
        geometries.append(frame_mesh)

        marker = o3d.geometry.TriangleMesh.create_sphere(radius=origin_radius)
        marker.paint_uniform_color(color_from_frame_info(frame_info))
        marker.translate(pose[:3, 3])
        geometries.append(marker)

        label = str(frame_info.get("label", frame_name))
        labels.append((label, pose[:3, 3].copy()))

    if draw_lines:
        points: List[np.ndarray] = []
        lines: List[List[int]] = []
        colors: List[List[float]] = []
        for entry in transforms:
            if entry.parent not in poses or entry.child not in poses:
                continue
            start_index = len(points)
            points.append(poses[entry.parent][:3, 3].copy())
            points.append(poses[entry.child][:3, 3].copy())
            lines.append([start_index, start_index + 1])
            colors.append([0.45, 0.45, 0.45])
        if lines:
            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector(np.array(points, dtype=float))
            line_set.lines = o3d.utility.Vector2iVector(np.array(lines, dtype=np.int32))
            line_set.colors = o3d.utility.Vector3dVector(np.array(colors, dtype=float))
            geometries.append(line_set)

    return geometries, labels


def number_text(value: float, precision: int = 6) -> str:
    if math.isnan(value):
        return "nan"
    return f"{value:.{precision}f}"


def css_color(color: Sequence[float]) -> str:
    red, green, blue = [int(round(min(1.0, max(0.0, value)) * 255.0)) for value in color]
    return f"rgb({red}, {green}, {blue})"


def matrix_text(values: np.ndarray, precision: int = 10) -> str:
    return "[" + ", ".join(f"{value:.{precision}f}" for value in values.reshape(-1)) + "]"


def derived_transform_rows(poses: Dict[str, np.ndarray]) -> List[Tuple[str, str, str, np.ndarray]]:
    pairs = [
        ("T_imu_main_antenna", "imu", "main_antenna"),
        ("T_imu_rslidar_right", "imu", "rslidar_right"),
        ("T_imu_rslidar_left", "imu", "rslidar_left"),
        ("T_rslidar_right_rslidar_left", "rslidar_right", "rslidar_left"),
        ("T_rear_axle_center_imu", "rear_axle_center", "imu"),
    ]
    rows: List[Tuple[str, str, str, np.ndarray]] = []
    for name, parent, child in pairs:
        if parent in poses and child in poses:
            rows.append((name, parent, child, invert_transform(poses[parent]) @ poses[child]))
    return rows


def make_static_html(
    data: dict,
    root_frame: str,
    poses: Dict[str, np.ndarray],
    transforms: Sequence[TransformEntry],
    reference_rows: Sequence[Tuple[str, str, float, float, str]],
) -> str:
    frames = data.get("frames", {})
    frame_names = ordered_frame_names(frames, poses)
    points = np.array([poses[name][:3, 3] for name in frame_names], dtype=float)
    min_x, min_y = points[:, 0].min(), points[:, 1].min()
    max_x, max_y = points[:, 0].max(), points[:, 1].max()
    span_x = max(max_x - min_x, 1.0)
    span_y = max(max_y - min_y, 1.0)
    padding = max(0.5, 0.18 * max(span_x, span_y))
    min_x -= padding
    max_x += padding
    min_y -= padding
    max_y += padding

    width = 1180
    height = 760
    plot_left = 70
    plot_top = 52
    plot_width = 770
    plot_height = 650
    scale = min(plot_width / (max_x - min_x), plot_height / (max_y - min_y))

    def screen(point: np.ndarray) -> Tuple[float, float]:
        x = plot_left + (float(point[0]) - min_x) * scale
        y = plot_top + (max_y - float(point[1])) * scale
        return x, y

    svg_parts: List[str] = [
        f'<svg viewBox="0 0 {width} {height}" role="img" aria-label="Vehicle transform top-down view">',
        "<defs>",
        '<marker id="arrow-red" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto">'
        '<path d="M0,0 L8,4 L0,8 Z" fill="#d7252f"/></marker>',
        '<marker id="arrow-green" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto">'
        '<path d="M0,0 L8,4 L0,8 Z" fill="#20853b"/></marker>',
        "</defs>",
        '<rect x="0" y="0" width="1180" height="760" fill="#f7f8fa"/>',
        '<rect x="40" y="28" width="830" height="700" rx="8" fill="#ffffff" stroke="#d7dce3"/>',
        '<text x="58" y="54" class="title">Top-down transform graph, root frame: '
        + escape(root_frame)
        + "</text>",
        '<text x="58" y="76" class="caption">X right, Y forward, Z shown in tables. Red/green ticks are local X/Y axes.</text>',
    ]

    meter_min_x = math.floor(min_x)
    meter_max_x = math.ceil(max_x)
    meter_min_y = math.floor(min_y)
    meter_max_y = math.ceil(max_y)
    for meter_x in range(meter_min_x, meter_max_x + 1):
        x1, y1 = screen(np.array([meter_x, min_y, 0.0]))
        x2, y2 = screen(np.array([meter_x, max_y, 0.0]))
        svg_parts.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" class="grid"/>')
    for meter_y in range(meter_min_y, meter_max_y + 1):
        x1, y1 = screen(np.array([min_x, meter_y, 0.0]))
        x2, y2 = screen(np.array([max_x, meter_y, 0.0]))
        svg_parts.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" class="grid"/>')

    for entry in transforms:
        if entry.parent not in poses or entry.child not in poses:
            continue
        start = poses[entry.parent][:3, 3]
        end = poses[entry.child][:3, 3]
        x1, y1 = screen(start)
        x2, y2 = screen(end)
        mx, my = (x1 + x2) * 0.5, (y1 + y2) * 0.5
        svg_parts.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" class="edge"/>')
        svg_parts.append(f'<text x="{mx + 6:.1f}" y="{my - 6:.1f}" class="edge-label">{escape(entry.name)}</text>')

    axis_len_m = 0.38
    label_offsets = {
        "imu": (-38, -18),
        "main_antenna": (12, -18),
        "rear_axle_center": (12, 20),
        "rslidar_right": (12, -12),
        "rslidar_left": (-96, -12),
    }
    for frame_name in frame_names:
        pose = poses[frame_name]
        origin = pose[:3, 3]
        x_axis = origin + pose[:3, :3] @ np.array([axis_len_m, 0.0, 0.0])
        y_axis = origin + pose[:3, :3] @ np.array([0.0, axis_len_m, 0.0])
        ox, oy = screen(origin)
        xx, xy = screen(x_axis)
        yx, yy = screen(y_axis)
        frame_info = frames.get(frame_name, {})
        color = css_color(color_from_frame_info(frame_info))
        label = str(frame_info.get("label", frame_name))
        dx, dy = label_offsets.get(frame_name, (12, -12))
        svg_parts.append(
            f'<line x1="{ox:.1f}" y1="{oy:.1f}" x2="{xx:.1f}" y2="{xy:.1f}" '
            'class="axis-x" marker-end="url(#arrow-red)"/>'
        )
        svg_parts.append(
            f'<line x1="{ox:.1f}" y1="{oy:.1f}" x2="{yx:.1f}" y2="{yy:.1f}" '
            'class="axis-y" marker-end="url(#arrow-green)"/>'
        )
        svg_parts.append(f'<circle cx="{ox:.1f}" cy="{oy:.1f}" r="8" fill="{color}" stroke="#1f2933"/>')
        svg_parts.append(f'<text x="{ox + dx:.1f}" y="{oy + dy:.1f}" class="frame-label">{escape(label)}</text>')
        svg_parts.append(
            f'<text x="{ox + dx:.1f}" y="{oy + dy + 16:.1f}" class="coord-label">'
            f"({origin[0]:.2f}, {origin[1]:.2f}, {origin[2]:.2f})</text>"
        )

    svg_parts.append("</svg>")

    pose_rows = []
    for frame_name in frame_names:
        translation = poses[frame_name][:3, 3]
        frame_info = frames.get(frame_name, {})
        label = str(frame_info.get("label", frame_name))
        pose_rows.append(
            "<tr>"
            f"<td>{escape(frame_name)}</td>"
            f"<td>{escape(label)}</td>"
            f"<td>{translation[0]:.6f}</td>"
            f"<td>{translation[1]:.6f}</td>"
            f"<td>{translation[2]:.6f}</td>"
            "</tr>"
        )

    derived_rows = []
    for name, parent, child, matrix in derived_transform_rows(poses):
        translation = matrix[:3, 3]
        derived_rows.append(
            "<tr>"
            f"<td>{escape(name)}</td>"
            f"<td>{escape(parent)} -> {escape(child)}</td>"
            f"<td>[{translation[0]:.10f}, {translation[1]:.10f}, {translation[2]:.10f}]</td>"
            f"<td><code>{escape(matrix_text(matrix[:3, :3]))}</code></td>"
            "</tr>"
        )

    reference_html_rows = []
    for name, status, trans_err, rot_err, source in reference_rows:
        class_name = "ok" if status == "OK" else "fail"
        reference_html_rows.append(
            "<tr>"
            f'<td class="{class_name}">{escape(status)}</td>'
            f"<td>{escape(name)}</td>"
            f"<td>{number_text(trans_err)}</td>"
            f"<td>{number_text(rot_err)}</td>"
            f"<td>{escape(source)}</td>"
            "</tr>"
        )

    return """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Vehicle Transform Visualization</title>
<style>
body { margin: 0; font: 14px/1.45 -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; color: #17202a; background: #eef1f5; }
main { max-width: 1220px; margin: 0 auto; padding: 24px; }
h1 { margin: 0 0 10px; font-size: 24px; letter-spacing: 0; }
h2 { margin: 26px 0 10px; font-size: 17px; letter-spacing: 0; }
p { margin: 0 0 14px; color: #4d5b6a; }
svg { width: 100%; height: auto; display: block; border-radius: 8px; box-shadow: 0 1px 2px rgba(15, 23, 42, 0.08); }
table { width: 100%; border-collapse: collapse; background: #fff; border: 1px solid #d7dce3; border-radius: 8px; overflow: hidden; }
th, td { padding: 8px 10px; border-bottom: 1px solid #e6eaf0; text-align: left; vertical-align: top; }
th { background: #f7f8fa; font-weight: 650; }
code { white-space: normal; }
.title { font-size: 18px; font-weight: 700; fill: #17202a; }
.caption { font-size: 13px; fill: #5d6b7a; }
.grid { stroke: #e6eaf0; stroke-width: 1; }
.edge { stroke: #657284; stroke-width: 2.2; stroke-dasharray: 6 5; }
.edge-label { font-size: 12px; fill: #465363; }
.axis-x { stroke: #d7252f; stroke-width: 2.4; }
.axis-y { stroke: #20853b; stroke-width: 2.4; }
.frame-label { font-size: 14px; font-weight: 700; fill: #17202a; paint-order: stroke; stroke: #fff; stroke-width: 4px; }
.coord-label { font-size: 12px; fill: #52606d; paint-order: stroke; stroke: #fff; stroke-width: 4px; }
.ok { color: #1f7a3b; font-weight: 700; }
.fail { color: #b42318; font-weight: 700; }
</style>
</head>
<body>
<main>
<h1>Vehicle Transform Visualization</h1>
<p>Convention: T_parent_child maps child-frame points into parent-frame points. Root-frame positions are in meters.</p>
""" + "\n".join(svg_parts) + """
<h2>Frame Poses In Root Frame</h2>
<table>
<thead><tr><th>Frame</th><th>Label</th><th>X</th><th>Y</th><th>Z</th></tr></thead>
<tbody>
""" + "\n".join(pose_rows) + """
</tbody>
</table>
<h2>Derived Active Transforms</h2>
<table>
<thead><tr><th>Name</th><th>Parent -> Child</th><th>Translation</th><th>Rotation Matrix</th></tr></thead>
<tbody>
""" + "\n".join(derived_rows) + """
</tbody>
</table>
<h2>Reference Checks</h2>
<table>
<thead><tr><th>Status</th><th>Name</th><th>Trans Error m</th><th>Rot Error deg</th><th>Source</th></tr></thead>
<tbody>
""" + "\n".join(reference_html_rows) + """
</tbody>
</table>
</main>
</body>
</html>
"""


def export_html(
    output_path: Path,
    data: dict,
    root_frame: str,
    poses: Dict[str, np.ndarray],
    transforms: Sequence[TransformEntry],
    reference_rows: Sequence[Tuple[str, str, float, float, str]],
) -> None:
    output_path = output_path.expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    html = make_static_html(
        data=data,
        root_frame=root_frame,
        poses=poses,
        transforms=transforms,
        reference_rows=reference_rows,
    )
    output_path.write_text(html, encoding="utf-8")
    print(f"HTML visualization: {output_path}")


def draw_with_labels(o3d: object, geometries: Sequence[object], labels: Sequence[Tuple[str, np.ndarray]]) -> bool:
    try:
        import open3d.visualization.gui as gui

        app = gui.Application.instance
        app.initialize()
        viewer = o3d.visualization.O3DVisualizer("air_mapping params_viz", 1280, 800)
        viewer.show_settings = True
        for index, geometry in enumerate(geometries):
            viewer.add_geometry(f"geometry_{index:02d}", geometry)
        for text, position in labels:
            viewer.add_3d_label(position.tolist(), text)
        app.add_window(viewer)
        app.run()
        return True
    except Exception as exc:  # pragma: no cover - interactive viewer path
        print(f"[WARN] Open3D label viewer failed, falling back to classic viewer: {exc}", file=sys.stderr)
        return False


def visualize(
    data: dict,
    poses: Dict[str, np.ndarray],
    transforms: Sequence[TransformEntry],
    args: argparse.Namespace,
) -> None:
    try:
        import open3d as o3d
    except ImportError as exc:  # pragma: no cover - dependency check path
        raise SystemExit("Open3D is required for visualization. Install package: python3-open3d or open3d") from exc

    viz_config = data.get("visualization", {})
    frame_size = args.frame_size if args.frame_size is not None else float(viz_config.get("frame_size", 0.65))
    origin_radius = (
        args.origin_radius if args.origin_radius is not None else float(viz_config.get("origin_radius", 0.055))
    )
    geometries, labels = make_visual_geometries(
        o3d=o3d,
        data=data,
        poses=poses,
        transforms=transforms,
        frame_size=frame_size,
        origin_radius=origin_radius,
        draw_lines=not args.no_lines,
    )
    if not geometries:
        raise RuntimeError("No drawable geometries were generated")
    if not args.no_labels and draw_with_labels(o3d, geometries, labels):
        return
    o3d.visualization.draw_geometries(
        geometries,
        window_name="air_mapping params_viz",
        width=1280,
        height=800,
    )


def main() -> int:
    args = parse_args()
    config_path = Path(args.config).expanduser().resolve()
    data = read_config(config_path)
    root_frame = args.root or str(data.get("root_frame", "imu"))
    transforms = parse_transform_entries(data.get("transforms", []), "transforms")
    references = parse_transform_entries(data.get("reference_transforms", []), "reference_transforms")
    poses, unresolved, conflicts = build_frame_poses(
        transforms=transforms,
        root_frame=root_frame,
        translation_tolerance=args.translation_tolerance,
        rotation_tolerance_deg=args.rotation_tolerance_deg,
    )
    reference_rows, references_ok = run_reference_checks(
        references=references,
        poses=poses,
        translation_tolerance=args.translation_tolerance,
        rotation_tolerance_deg=args.rotation_tolerance_deg,
    )
    print_summary(
        data=data,
        root_frame=root_frame,
        poses=poses,
        unresolved=unresolved,
        conflicts=conflicts,
        reference_rows=reference_rows,
    )

    checks_ok = not unresolved and not conflicts and references_ok
    if args.export_html:
        export_html(
            output_path=Path(args.export_html),
            data=data,
            root_frame=root_frame,
            poses=poses,
            transforms=transforms,
            reference_rows=reference_rows,
        )
    if args.print_only or args.export_html:
        return 0 if checks_ok or not args.strict else 1

    visualize(data=data, poses=poses, transforms=transforms, args=args)
    return 0 if checks_ok or not args.strict else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, ValueError, RuntimeError) as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        raise SystemExit(1)
