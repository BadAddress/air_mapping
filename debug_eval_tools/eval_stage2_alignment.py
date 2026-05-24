#!/usr/bin/env python3
"""
Stage 2 alignment evaluation and visualization tool.

Reads air_mapping Stage 2 artifacts and produces metrics, plots, a text report,
JSON stats, and a compact HTML report. The tool is intentionally read-only: it
does not modify Stage 2 outputs.
"""

import argparse
import csv
import html
import json
import math
import os
from collections import Counter
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import numpy as np

try:
    import yaml

    HAS_YAML = True
except Exception:
    yaml = None
    HAS_YAML = False

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    HAS_MATPLOTLIB = True
except Exception:
    HAS_MATPLOTLIB = False
    plt = None

try:
    import plotly.graph_objects as go
    import plotly.io as pio

    HAS_PLOTLY = True
except Exception:
    HAS_PLOTLY = False
    go = None
    pio = None


def read_top_level_config(config_path: Path) -> Dict[str, str]:
    result = {
        "active_vehicle": "es6",
        "data_root": str(config_path.parent.parent / "data"),
        "debug_root": str(config_path.parent.parent / "data" / "debug"),
    }
    if not config_path.exists():
        return result
    for line in config_path.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip().strip("'\"")
        if key in result and value:
            result[key] = value
    return result


def read_csv_rows(path: Path) -> List[Dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", newline="") as stream:
        return list(csv.DictReader(stream))


def to_float(row: Dict[str, str], key: str, default: float = math.nan) -> float:
    value = row.get(key, "")
    if value is None or value == "":
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def to_int(row: Dict[str, str], key: str, default: int = 0) -> int:
    value = row.get(key, "")
    if value is None or value == "":
        return default
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def parse_vec(row: Dict[str, str], prefix: str) -> np.ndarray:
    return np.array(
        [to_float(row, f"{prefix}_x"), to_float(row, f"{prefix}_y"), to_float(row, f"{prefix}_z")],
        dtype=float,
    )


def finite_array(values: Iterable[float]) -> np.ndarray:
    arr = np.asarray(list(values), dtype=float)
    return arr[np.isfinite(arr)]


def stats(values: Iterable[float]) -> Dict[str, Any]:
    arr = finite_array(values)
    if arr.size == 0:
        return {"count": 0}
    return {
        "count": int(arr.size),
        "rmse": float(np.sqrt(np.mean(arr * arr))),
        "mean": float(np.mean(arr)),
        "median": float(np.median(arr)),
        "std": float(np.std(arr)),
        "min": float(np.min(arr)),
        "max": float(np.max(arr)),
        "p90": float(np.percentile(arr, 90)),
        "p95": float(np.percentile(arr, 95)),
        "p99": float(np.percentile(arr, 99)),
    }


def pct_reduction(before: float, after: float) -> Optional[float]:
    if not np.isfinite(before) or abs(before) < 1e-12 or not np.isfinite(after):
        return None
    return float((1.0 - after / before) * 100.0)


def read_utm_origin(stage2_dir: Path, anchors: List[Dict[str, Any]]) -> np.ndarray:
    origin_path = stage2_dir / "alignment" / "utm_origin.txt"
    if origin_path.exists():
        with origin_path.open("r") as stream:
            for line in stream:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                if len(parts) >= 3:
                    try:
                        return np.array([float(parts[0]), float(parts[1]), float(parts[2])], dtype=float)
                    except ValueError:
                        pass
    if anchors:
        return np.asarray(anchors[0]["gps_smooth_utm"], dtype=float)
    return np.zeros(3, dtype=float)


def load_keyframes(stage2_dir: Path) -> List[Dict[str, Any]]:
    rows = read_csv_rows(stage2_dir / "keyframes" / "keyframes_opt.csv")
    keyframes: List[Dict[str, Any]] = []
    for index, row in enumerate(rows):
        keyframes.append(
            {
                "id": to_int(row, "id"),
                "index": index,
                "timestamp": to_float(row, "timestamp"),
                "lio": parse_vec(row, "lio"),
                "opt": parse_vec(row, "opt"),
                "utm": parse_vec(row, "utm"),
                "has_gps": to_int(row, "has_gps"),
            }
        )
    return keyframes


def load_anchors(stage2_dir: Path) -> List[Dict[str, Any]]:
    rows = read_csv_rows(stage2_dir / "diagnostics" / "alignment_anchors.csv")
    anchors: List[Dict[str, Any]] = []
    for row in rows:
        anchors.append(
            {
                "keyframe_id": to_int(row, "keyframe_id"),
                "timestamp": to_float(row, "timestamp"),
                "segment_id": to_int(row, "segment_id", -1),
                "weight": to_float(row, "weight"),
                "lio": parse_vec(row, "lio"),
                "gps_utm": parse_vec(row, "gps_utm"),
                "gps_smooth_utm": parse_vec(row, "gps_smooth_utm"),
                "std": np.array([to_float(row, "std_x"), to_float(row, "std_y"), to_float(row, "std_z")]),
                "residual_before_reported": to_float(row, "residual_before_m"),
                "residual_after_reported": to_float(row, "residual_after_m"),
            }
        )
    return anchors


def load_pose_delta(stage2_dir: Path) -> List[Dict[str, Any]]:
    rows = read_csv_rows(stage2_dir / "keyframes" / "pose_delta.csv")
    deltas: List[Dict[str, Any]] = []
    for row in rows:
        delta = np.array([to_float(row, "dx"), to_float(row, "dy"), to_float(row, "dz")], dtype=float)
        deltas.append(
            {
                "id": to_int(row, "id"),
                "timestamp": to_float(row, "timestamp"),
                "delta": delta,
                "translation_norm": float(np.linalg.norm(delta)) if np.all(np.isfinite(delta)) else math.nan,
                "rotation_delta_deg": to_float(row, "rotation_delta_deg"),
            }
        )
    return deltas


def load_single_csv(stage2_dir: Path, relative: str) -> Dict[str, str]:
    rows = read_csv_rows(stage2_dir / relative)
    return rows[0] if rows else {}


def load_manifest(stage2_dir: Path) -> Dict[str, Any]:
    manifest_path = stage2_dir / "manifest.yaml"
    if not manifest_path.exists():
        return {"manifest_path": str(manifest_path), "available": False}
    if HAS_YAML:
        loaded = yaml.safe_load(manifest_path.read_text()) or {}
        if not isinstance(loaded, dict):
            loaded = {}
        loaded["manifest_path"] = str(manifest_path)
        loaded["available"] = True
        return loaded

    result: Dict[str, Any] = {"manifest_path": str(manifest_path), "available": True}
    for line in manifest_path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        result[key.strip()] = value.strip().strip("'\"")
    return result


def add_path_distance(keyframes: List[Dict[str, Any]]) -> Dict[int, Dict[str, Any]]:
    distance = 0.0
    previous: Optional[np.ndarray] = None
    id_to_keyframe: Dict[int, Dict[str, Any]] = {}
    for keyframe in keyframes:
        opt = keyframe["opt"]
        if previous is not None and np.all(np.isfinite(opt)) and np.all(np.isfinite(previous)):
            distance += float(np.linalg.norm(opt - previous))
        keyframe["path_m"] = distance
        previous = opt
        id_to_keyframe[keyframe["id"]] = keyframe
    return id_to_keyframe


def enrich_anchors(
    anchors: List[Dict[str, Any]],
    keyframes_by_id: Dict[int, Dict[str, Any]],
    utm_origin: np.ndarray,
) -> None:
    if not anchors:
        return
    first_target = anchors[0]["gps_smooth_utm"] - utm_origin
    baseline_translation = first_target - anchors[0]["lio"]
    for anchor in anchors:
        keyframe = keyframes_by_id.get(anchor["keyframe_id"])
        target_local = anchor["gps_smooth_utm"] - utm_origin
        raw_target_local = anchor["gps_utm"] - utm_origin
        baseline = anchor["lio"] + baseline_translation
        opt = keyframe["opt"] if keyframe is not None else np.full(3, math.nan)
        before_vec = baseline - target_local
        after_vec = opt - target_local
        anchor["keyframe_index"] = keyframe["index"] if keyframe is not None else math.nan
        anchor["path_m"] = keyframe["path_m"] if keyframe is not None else math.nan
        anchor["gps_local"] = target_local
        anchor["gps_raw_local"] = raw_target_local
        anchor["baseline"] = baseline
        anchor["opt"] = opt
        anchor["before_vec"] = before_vec
        anchor["after_vec"] = after_vec
        anchor["before_3d"] = float(np.linalg.norm(before_vec)) if np.all(np.isfinite(before_vec)) else math.nan
        anchor["after_3d"] = float(np.linalg.norm(after_vec)) if np.all(np.isfinite(after_vec)) else math.nan
        anchor["before_xy"] = float(np.linalg.norm(before_vec[:2])) if np.all(np.isfinite(before_vec[:2])) else math.nan
        anchor["after_xy"] = float(np.linalg.norm(after_vec[:2])) if np.all(np.isfinite(after_vec[:2])) else math.nan
        anchor["before_abs_z"] = float(abs(before_vec[2])) if np.isfinite(before_vec[2]) else math.nan
        anchor["after_abs_z"] = float(abs(after_vec[2])) if np.isfinite(after_vec[2]) else math.nan
        anchor["std_xy"] = float(np.sqrt((anchor["std"][0] ** 2 + anchor["std"][1] ** 2) / 2.0))
        anchor["gps_z_smoothing_delta"] = float(anchor["gps_smooth_utm"][2] - anchor["gps_utm"][2])


def array_from_dicts(items: List[Dict[str, Any]], key: str) -> np.ndarray:
    return np.asarray([item.get(key, math.nan) for item in items], dtype=float)


def vecs_from_dicts(items: List[Dict[str, Any]], key: str) -> np.ndarray:
    if not items:
        return np.zeros((0, 3), dtype=float)
    return np.asarray([item.get(key, np.full(3, math.nan)) for item in items], dtype=float)


def compute_segment_metrics(
    stage2_dir: Path,
    anchors: List[Dict[str, Any]],
    keyframes: List[Dict[str, Any]],
) -> Dict[str, Any]:
    segments = read_csv_rows(stage2_dir / "diagnostics" / "gps_segments.csv")
    segment_lengths = [to_float(row, "length_xy_m") for row in segments]
    segment_anchor_counts = [to_int(row, "anchor_count") for row in segments]

    anchor_paths = finite_array(anchor.get("path_m", math.nan) for anchor in anchors)
    anchor_path_gaps = np.diff(anchor_paths) if anchor_paths.size > 1 else np.asarray([], dtype=float)
    anchor_indices = finite_array(anchor.get("keyframe_index", math.nan) for anchor in anchors)
    anchor_index_gaps = np.diff(anchor_indices) if anchor_indices.size > 1 else np.asarray([], dtype=float)

    total_path_m = keyframes[-1]["path_m"] if keyframes else 0.0
    covered_path_m = float(np.sum(finite_array(segment_lengths)))
    uncovered_path_m = max(total_path_m - covered_path_m, 0.0)
    return {
        "segment_count": len(segments),
        "anchor_count": len(anchors),
        "keyframe_count": len(keyframes),
        "anchor_coverage_ratio": float(len(anchors) / max(len(keyframes), 1)),
        "total_keyframe_path_m": float(total_path_m),
        "covered_path_m": covered_path_m,
        "covered_path_ratio": float(covered_path_m / max(total_path_m, 1e-9)),
        "uncovered_path_m": float(uncovered_path_m),
        "uncovered_path_ratio": float(uncovered_path_m / max(total_path_m, 1e-9)),
        "segment_length_m": stats(segment_lengths),
        "segment_anchor_count": stats(segment_anchor_counts),
        "anchor_path_gap_m": stats(anchor_path_gaps),
        "anchor_index_gap": stats(anchor_index_gaps),
        "segments": [
            {
                "segment_id": to_int(row, "segment_id", -1),
                "first_keyframe_id": to_int(row, "first_keyframe_id"),
                "last_keyframe_id": to_int(row, "last_keyframe_id"),
                "anchor_count": to_int(row, "anchor_count"),
                "length_xy_m": to_float(row, "length_xy_m"),
                "mean_abs_z_smoothing_delta_m": to_float(row, "mean_abs_z_smoothing_delta_m"),
                "max_abs_z_smoothing_delta_m": to_float(row, "max_abs_z_smoothing_delta_m"),
            }
            for row in segments
        ],
    }


def build_anchor_segment_ranges(anchors: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    if not anchors:
        return []
    ranges: List[Dict[str, Any]] = []
    start = 0
    current_segment_id = anchors[0]["segment_id"]
    for index in range(1, len(anchors)):
        if anchors[index]["segment_id"] != current_segment_id:
            ranges.append(
                {
                    "segment_id": current_segment_id,
                    "start_anchor_index": start,
                    "end_anchor_index": index - 1,
                    "start_keyframe_index": anchors[start].get("keyframe_index", math.nan),
                    "end_keyframe_index": anchors[index - 1].get("keyframe_index", math.nan),
                }
            )
            start = index
            current_segment_id = anchors[index]["segment_id"]
    ranges.append(
        {
            "segment_id": current_segment_id,
            "start_anchor_index": start,
            "end_anchor_index": len(anchors) - 1,
            "start_keyframe_index": anchors[start].get("keyframe_index", math.nan),
            "end_keyframe_index": anchors[-1].get("keyframe_index", math.nan),
        }
    )
    return ranges


def build_gap_ranges(total_length: int, covered_ranges: List[Dict[str, Any]]) -> List[Tuple[int, int]]:
    if total_length <= 0:
        return []
    gaps: List[Tuple[int, int]] = []
    cursor = 0
    for segment in covered_ranges:
        start_idx = int(segment["start_keyframe_index"])
        end_idx = int(segment["end_keyframe_index"])
        if end_idx < cursor:
            continue
        if start_idx > cursor:
            gaps.append((cursor, start_idx - 1))
        cursor = max(cursor, end_idx + 1)
    if cursor < total_length:
        gaps.append((cursor, total_length - 1))
    return gaps


def compute_lever_metrics(stage2_dir: Path) -> Dict[str, Any]:
    summary = load_single_csv(stage2_dir, "diagnostics/lever_arm_calibration.csv")
    samples_rows = read_csv_rows(stage2_dir / "diagnostics" / "lever_arm_samples.csv")
    samples: List[Dict[str, Any]] = []
    for row in samples_rows:
        selected = to_int(row, "selected") != 0
        samples.append(
            {
                "keyframe_id": to_int(row, "keyframe_id"),
                "timestamp": to_float(row, "timestamp"),
                "selected": selected,
                "reject_reason": row.get("reject_reason", ""),
                "residual_initial_xy_m": to_float(row, "residual_initial_xy_m"),
                "residual_optimized_xy_m": to_float(row, "residual_optimized_xy_m"),
                "std_xy": float(
                    np.sqrt((to_float(row, "std_x") ** 2 + to_float(row, "std_y") ** 2) / 2.0)
                ),
                "max_interp_gap_s": to_float(row, "max_interp_gap_s"),
                "gnss_lio_yaw_diff_deg": to_float(row, "gnss_lio_yaw_diff_deg"),
                "heading_std_deg": to_float(row, "heading_std_deg"),
                "sol_type": to_int(row, "sol_type"),
            }
        )

    selected_samples = [sample for sample in samples if sample["selected"]]
    reject_counts = Counter(sample["reject_reason"] or "unknown" for sample in samples)
    initial_cost = to_float(summary, "weighted_cost_initial")
    optimized_cost = to_float(summary, "weighted_cost_optimized")
    mean_initial = to_float(summary, "mean_residual_initial_xy_m")
    mean_optimized = to_float(summary, "mean_residual_optimized_xy_m")
    initial_lever = parse_vec(summary, "initial")
    optimized_lever = parse_vec(summary, "optimized")
    correction = parse_vec(summary, "correction")

    return {
        "summary": dict(summary),
        "initial_lever_arm": initial_lever,
        "optimized_lever_arm": optimized_lever,
        "correction": correction,
        "correction_norm_m": float(np.linalg.norm(correction)) if np.all(np.isfinite(correction)) else math.nan,
        "heading_bias_deg": to_float(summary, "heading_bias_deg"),
        "samples": samples,
        "candidate_count": len(samples),
        "selected_count": len(selected_samples),
        "selected_ratio": float(len(selected_samples) / max(len(samples), 1)),
        "reject_counts": dict(reject_counts),
        "residual_initial_xy_m": stats(sample["residual_initial_xy_m"] for sample in selected_samples),
        "residual_optimized_xy_m": stats(sample["residual_optimized_xy_m"] for sample in selected_samples),
        "weighted_cost_reduction_pct": pct_reduction(initial_cost, optimized_cost),
        "mean_residual_reduction_pct": pct_reduction(mean_initial, mean_optimized),
    }


def compute_pose_delta_metrics(deltas: List[Dict[str, Any]]) -> Dict[str, Any]:
    return {
        "translation_delta_m": stats(delta["translation_norm"] for delta in deltas),
        "dx_m": stats(delta["delta"][0] for delta in deltas),
        "dy_m": stats(delta["delta"][1] for delta in deltas),
        "dz_m": stats(delta["delta"][2] for delta in deltas),
        "rotation_delta_deg": stats(delta["rotation_delta_deg"] for delta in deltas),
    }


def compute_alignment_metrics(anchors: List[Dict[str, Any]]) -> Dict[str, Any]:
    before_mean = stats(anchor["before_3d"] for anchor in anchors).get("mean", math.nan)
    after_mean = stats(anchor["after_3d"] for anchor in anchors).get("mean", math.nan)
    before_rmse = stats(anchor["before_3d"] for anchor in anchors).get("rmse", math.nan)
    after_rmse = stats(anchor["after_3d"] for anchor in anchors).get("rmse", math.nan)
    return {
        "before_3d_m": stats(anchor["before_3d"] for anchor in anchors),
        "after_3d_m": stats(anchor["after_3d"] for anchor in anchors),
        "before_xy_m": stats(anchor["before_xy"] for anchor in anchors),
        "after_xy_m": stats(anchor["after_xy"] for anchor in anchors),
        "before_abs_z_m": stats(anchor["before_abs_z"] for anchor in anchors),
        "after_abs_z_m": stats(anchor["after_abs_z"] for anchor in anchors),
        "reported_before_3d_m": stats(anchor["residual_before_reported"] for anchor in anchors),
        "reported_after_3d_m": stats(anchor["residual_after_reported"] for anchor in anchors),
        "after_report_delta_m": stats(
            abs(anchor["after_3d"] - anchor["residual_after_reported"]) for anchor in anchors
        ),
        "mean_residual_reduction_pct": pct_reduction(before_mean, after_mean),
        "rmse_residual_reduction_pct": pct_reduction(before_rmse, after_rmse),
    }


def create_output_dir(base_dir: Path, timestamped: bool) -> Path:
    if timestamped:
        output_dir = base_dir / datetime.now().strftime("%Y%m%d_%H%M%S")
    else:
        output_dir = base_dir
    if not timestamped and output_dir.exists():
        import shutil

        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    if timestamped:
        latest = base_dir / "latest"
        try:
            if latest.is_symlink() or latest.exists():
                latest.unlink()
            latest.symlink_to(output_dir.name)
        except OSError:
            pass
    return output_dir


def downsample_indices(length: int, max_points: int) -> np.ndarray:
    if length <= max_points:
        return np.arange(length)
    step = max(1, int(math.ceil(length / max_points)))
    return np.arange(0, length, step)


def save_figure(fig: Any, output_dir: Path, filename: str, dpi: int = 160) -> str:
    path = output_dir / filename
    fig.align_labels()
    if not fig.get_constrained_layout():
        fig.tight_layout(pad=2.2, h_pad=2.6, w_pad=2.2)
    fig.savefig(path, dpi=dpi, bbox_inches="tight", pad_inches=0.18)
    plt.close(fig)
    return filename


def plot_trajectory(output_dir: Path, keyframes: List[Dict[str, Any]], anchors: List[Dict[str, Any]], max_points: int) -> str:
    fig, ax = plt.subplots(figsize=(13, 9.5), constrained_layout=True)
    keyframe_idx = downsample_indices(len(keyframes), max_points)
    opt = np.asarray([keyframes[i]["opt"] for i in keyframe_idx], dtype=float)
    lio = np.asarray([keyframes[i]["lio"] for i in keyframe_idx], dtype=float)

    baseline_translation = np.zeros(3)
    if anchors:
        baseline_translation = anchors[0]["baseline"] - anchors[0]["lio"]
    baseline_lio = lio + baseline_translation

    if baseline_lio.size:
        ax.plot(baseline_lio[:, 0], baseline_lio[:, 1], color="#d62728", linewidth=1.2, label="LIO baseline")
    if opt.size:
        ax.plot(opt[:, 0], opt[:, 1], color="#1f77b4", linewidth=1.5, label="Stage2 optimized")

    if anchors:
        gps = vecs_from_dicts(anchors, "gps_local")
        raw_gps = vecs_from_dicts(anchors, "gps_raw_local")
        segment_ids = np.asarray([anchor["segment_id"] for anchor in anchors], dtype=float)
        scatter = ax.scatter(gps[:, 0], gps[:, 1], c=segment_ids, s=18, cmap="tab20", label="GPS smooth anchors")
        ax.scatter(raw_gps[:, 0], raw_gps[:, 1], color="#2ca02c", s=8, alpha=0.45, label="GPS raw anchors")
        vector_idx = downsample_indices(len(anchors), min(max_points, 600))
        opt_anchor = vecs_from_dicts([anchors[i] for i in vector_idx], "opt")
        gps_anchor = vecs_from_dicts([anchors[i] for i in vector_idx], "gps_local")
        vectors = gps_anchor - opt_anchor
        ax.quiver(
            opt_anchor[:, 0],
            opt_anchor[:, 1],
            vectors[:, 0],
            vectors[:, 1],
            angles="xy",
            scale_units="xy",
            scale=1.0,
            color="#444444",
            alpha=0.25,
            width=0.002,
        )
        cbar = fig.colorbar(scatter, ax=ax, fraction=0.03, pad=0.02)
        cbar.set_label("GPS segment id")

    ax.set_title("Stage2 alignment trajectory in UTM-local frame")
    ax.set_xlabel("X local (m)")
    ax.set_ylabel("Y local (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    return save_figure(fig, output_dir, "stage2_trajectory_alignment.png")


def plot_residuals(output_dir: Path, anchors: List[Dict[str, Any]]) -> str:
    path = array_from_dicts(anchors, "path_m")
    fig, axes = plt.subplots(2, 1, figsize=(13, 8.8), sharex=True, constrained_layout=True)
    axes[0].plot(path, array_from_dicts(anchors, "before_3d"), label="Before 3D", color="#d62728", linewidth=1.1)
    axes[0].plot(path, array_from_dicts(anchors, "after_3d"), label="After 3D", color="#1f77b4", linewidth=1.1)
    axes[0].plot(path, array_from_dicts(anchors, "after_xy"), label="After XY", color="#2ca02c", linewidth=1.0)
    axes[0].set_ylabel("Residual (m)")
    axes[0].set_title("Alignment residuals over path")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")

    axes[1].plot(path, array_from_dicts(anchors, "before_abs_z"), label="Before |Z|", color="#ff7f0e")
    axes[1].plot(path, array_from_dicts(anchors, "after_abs_z"), label="After |Z|", color="#9467bd")
    axes[1].set_xlabel("Path distance (m)")
    axes[1].set_ylabel("Z residual (m)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best")
    return save_figure(fig, output_dir, "stage2_residuals_over_path.png")


def plot_distribution(output_dir: Path, anchors: List[Dict[str, Any]]) -> str:
    before = finite_array(anchor["before_3d"] for anchor in anchors)
    after = finite_array(anchor["after_3d"] for anchor in anchors)
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.8), constrained_layout=True)
    bins = min(50, max(10, int(math.sqrt(max(len(before), len(after), 1)))))
    axes[0].hist(before, bins=bins, alpha=0.55, label="Before 3D", color="#d62728")
    axes[0].hist(after, bins=bins, alpha=0.65, label="After 3D", color="#1f77b4")
    axes[0].set_xlabel("Residual (m)")
    axes[0].set_ylabel("Count")
    axes[0].set_title("Residual histogram")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(loc="best")

    for values, label, color in ((before, "Before 3D", "#d62728"), (after, "After 3D", "#1f77b4")):
        if values.size:
            ordered = np.sort(values)
            cdf = np.arange(1, len(ordered) + 1) / len(ordered)
            axes[1].plot(ordered, cdf, label=label, color=color)
    axes[1].set_xlabel("Residual (m)")
    axes[1].set_ylabel("CDF")
    axes[1].set_title("Residual CDF")
    axes[1].grid(True, alpha=0.25)
    axes[1].legend(loc="best")
    return save_figure(fig, output_dir, "stage2_residual_distribution.png")


def plot_gps_quality(output_dir: Path, anchors: List[Dict[str, Any]], segment_metrics: Dict[str, Any]) -> str:
    path = array_from_dicts(anchors, "path_m")
    std_xy = array_from_dicts(anchors, "std_xy")
    std_z = np.asarray([anchor["std"][2] for anchor in anchors], dtype=float)
    after = array_from_dicts(anchors, "after_3d")

    fig, axes = plt.subplots(2, 2, figsize=(15, 10), constrained_layout=True)
    axes[0, 0].plot(path, std_xy, label="std XY", color="#1f77b4")
    axes[0, 0].plot(path, std_z, label="std Z", color="#ff7f0e")
    axes[0, 0].set_title("GPS standard deviation over path")
    axes[0, 0].set_xlabel("Path distance (m)")
    axes[0, 0].set_ylabel("Std dev (m)")
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend(loc="best")

    axes[0, 1].scatter(std_xy, after, s=14, alpha=0.65, color="#2ca02c")
    axes[0, 1].set_title("Residual vs GPS XY std")
    axes[0, 1].set_xlabel("GPS XY std (m)")
    axes[0, 1].set_ylabel("After residual 3D (m)")
    axes[0, 1].grid(True, alpha=0.3)

    segments = segment_metrics.get("segments", [])
    segment_ids = [segment["segment_id"] for segment in segments]
    lengths = [segment["length_xy_m"] for segment in segments]
    counts = [segment["anchor_count"] for segment in segments]
    axes[1, 0].bar(segment_ids, lengths, color="#9467bd")
    axes[1, 0].set_title("GPS segment length")
    axes[1, 0].set_xlabel("Segment id")
    axes[1, 0].set_ylabel("Length XY (m)")
    axes[1, 0].grid(True, axis="y", alpha=0.3)

    axes[1, 1].bar(segment_ids, counts, color="#8c564b")
    axes[1, 1].set_title("GPS anchors per segment")
    axes[1, 1].set_xlabel("Segment id")
    axes[1, 1].set_ylabel("Anchor count")
    axes[1, 1].grid(True, axis="y", alpha=0.3)
    return save_figure(fig, output_dir, "stage2_gps_segments_quality.png")


def plot_height(output_dir: Path, keyframes: List[Dict[str, Any]], anchors: List[Dict[str, Any]]) -> str:
    key_path = np.asarray([keyframe["path_m"] for keyframe in keyframes], dtype=float)
    opt_z = np.asarray([keyframe["utm"][2] for keyframe in keyframes], dtype=float)
    lio_z = np.asarray([keyframe["lio"][2] for keyframe in keyframes], dtype=float)
    if anchors:
        baseline_translation = anchors[0]["baseline"] - anchors[0]["lio"]
        utm_origin_z = anchors[0]["gps_smooth_utm"][2] - anchors[0]["gps_local"][2]
    else:
        baseline_translation = np.zeros(3)
        utm_origin_z = 0.0
    lio_z = lio_z + baseline_translation[2] + utm_origin_z

    anchor_path = array_from_dicts(anchors, "path_m")
    gps_z = np.asarray([anchor["gps_smooth_utm"][2] for anchor in anchors], dtype=float)
    raw_gps_z = np.asarray([anchor["gps_utm"][2] for anchor in anchors], dtype=float)
    smooth_delta = array_from_dicts(anchors, "gps_z_smoothing_delta")

    fig, axes = plt.subplots(2, 1, figsize=(13, 8.8), sharex=True, constrained_layout=True)
    axes[0].plot(key_path, lio_z, label="LIO baseline Z", color="#d62728", linewidth=1.0)
    axes[0].plot(key_path, opt_z, label="Stage2 aligned Z", color="#1f77b4", linewidth=1.0)
    axes[0].scatter(anchor_path, raw_gps_z, label="GPS raw Z", color="#2ca02c", s=10, alpha=0.45)
    axes[0].scatter(anchor_path, gps_z, label="GPS smooth Z", color="#9467bd", s=13, alpha=0.75)
    axes[0].set_title("Height alignment")
    axes[0].set_ylabel("Z UTM (m)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")

    axes[1].plot(anchor_path, smooth_delta, color="#ff7f0e")
    axes[1].set_title("GPS height smoothing delta")
    axes[1].set_xlabel("Path distance (m)")
    axes[1].set_ylabel("Smooth - raw (m)")
    axes[1].grid(True, alpha=0.3)
    return save_figure(fig, output_dir, "stage2_height_alignment.png")


def plot_lever(output_dir: Path, lever_metrics: Dict[str, Any]) -> Optional[str]:
    samples = lever_metrics.get("samples", [])
    if not samples:
        return None
    selected = [sample for sample in samples if sample["selected"]]
    fig, axes = plt.subplots(2, 2, figsize=(15, 10), constrained_layout=True)
    x = np.arange(len(selected))
    initial = np.asarray([sample["residual_initial_xy_m"] for sample in selected], dtype=float)
    optimized = np.asarray([sample["residual_optimized_xy_m"] for sample in selected], dtype=float)
    axes[0, 0].plot(x, initial, label="Initial", color="#d62728", linewidth=1.0)
    axes[0, 0].plot(x, optimized, label="Optimized", color="#1f77b4", linewidth=1.0)
    axes[0, 0].set_title("Lever-arm selected sample residuals")
    axes[0, 0].set_xlabel("Selected sample index")
    axes[0, 0].set_ylabel("XY residual (m)")
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend(loc="best")

    axes[0, 1].scatter(initial, optimized, s=14, alpha=0.65, color="#2ca02c")
    max_value = max(float(np.nanmax(initial)) if initial.size else 0.0, float(np.nanmax(optimized)) if optimized.size else 0.0)
    axes[0, 1].plot([0, max_value], [0, max_value], color="#444444", linestyle="--", linewidth=1.0)
    axes[0, 1].set_title("Initial vs optimized residual")
    axes[0, 1].set_xlabel("Initial XY residual (m)")
    axes[0, 1].set_ylabel("Optimized XY residual (m)")
    axes[0, 1].grid(True, alpha=0.3)

    axes[1, 0].hist(initial, bins=35, alpha=0.55, label="Initial", color="#d62728")
    axes[1, 0].hist(optimized, bins=35, alpha=0.65, label="Optimized", color="#1f77b4")
    axes[1, 0].set_title("Lever-arm residual distribution")
    axes[1, 0].set_xlabel("XY residual (m)")
    axes[1, 0].set_ylabel("Count")
    axes[1, 0].grid(True, alpha=0.25)
    axes[1, 0].legend(loc="best")

    reject_counts = lever_metrics.get("reject_counts", {})
    labels = list(reject_counts.keys())
    counts = [reject_counts[label] for label in labels]
    order = np.argsort(counts)[::-1]
    labels = [labels[i] for i in order[:10]]
    counts = [counts[i] for i in order[:10]]
    axes[1, 1].barh(labels[::-1], counts[::-1], color="#9467bd")
    axes[1, 1].set_title("Lever-arm sample reject reasons")
    axes[1, 1].set_xlabel("Count")
    axes[1, 1].grid(True, axis="x", alpha=0.25)
    return save_figure(fig, output_dir, "stage2_lever_arm_calibration.png")


def plot_pose_delta(output_dir: Path, deltas: List[Dict[str, Any]], keyframes_by_id: Dict[int, Dict[str, Any]]) -> Optional[str]:
    if not deltas:
        return None
    path = np.asarray([keyframes_by_id.get(delta["id"], {}).get("path_m", math.nan) for delta in deltas], dtype=float)
    delta_vec = np.asarray([delta["delta"] for delta in deltas], dtype=float)
    trans = np.asarray([delta["translation_norm"] for delta in deltas], dtype=float)
    rot = np.asarray([delta["rotation_delta_deg"] for delta in deltas], dtype=float)
    fig, axes = plt.subplots(2, 1, figsize=(13, 8.8), sharex=True, constrained_layout=True)
    axes[0].plot(path, delta_vec[:, 0], label="dx", linewidth=1.0)
    axes[0].plot(path, delta_vec[:, 1], label="dy", linewidth=1.0)
    axes[0].plot(path, delta_vec[:, 2], label="dz", linewidth=1.0)
    axes[0].plot(path, trans, label="translation norm", color="#111111", linewidth=1.2)
    axes[0].set_title("Stage2 pose delta from source LIO")
    axes[0].set_ylabel("Translation delta (m)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")

    axes[1].plot(path, rot, color="#ff7f0e")
    axes[1].set_xlabel("Path distance (m)")
    axes[1].set_ylabel("Rotation delta (deg)")
    axes[1].grid(True, alpha=0.3)
    return save_figure(fig, output_dir, "stage2_pose_delta.png")


def build_3d_comparison_figure(
    keyframes: List[Dict[str, Any]],
    anchors: List[Dict[str, Any]],
    segment_metrics: Dict[str, Any],
) -> Optional[Any]:
    if not HAS_PLOTLY or not keyframes or not anchors:
        return None

    baseline_translation = anchors[0]["baseline"] - anchors[0]["lio"]
    before_positions = np.asarray([keyframe["lio"] + baseline_translation for keyframe in keyframes], dtype=float)
    after_positions = np.asarray([keyframe["opt"] for keyframe in keyframes], dtype=float)
    keyframe_ids = [keyframe["id"] for keyframe in keyframes]
    timestamps = [keyframe["timestamp"] for keyframe in keyframes]
    path_labels = np.asarray(keyframe_ids, dtype=float)
    path_timestamps = np.asarray(timestamps, dtype=float)

    figure = go.Figure()
    figure.add_trace(
        go.Scatter3d(
            x=before_positions[:, 0],
            y=before_positions[:, 1],
            z=before_positions[:, 2],
            mode="lines",
            name="Before alignment",
            line=dict(color="rgba(214,39,40,0.55)", width=4),
            hoverinfo="skip",
        )
    )
    figure.add_trace(
        go.Scatter3d(
            x=after_positions[:, 0],
            y=after_positions[:, 1],
            z=after_positions[:, 2],
            mode="lines",
            name="After alignment (full rigid trajectory)",
            line=dict(color="rgba(31,119,180,0.78)", width=5),
            hovertemplate=(
                "keyframe=%{customdata[0]:.0f}<br>"
                "timestamp=%{customdata[1]:.3f}<br>"
                "x=%{x:.3f}<br>y=%{y:.3f}<br>z=%{z:.3f}<extra></extra>"
            ),
            customdata=np.column_stack([path_labels, path_timestamps]),
        )
    )
    covered_ranges = build_anchor_segment_ranges(anchors)
    gap_ranges = build_gap_ranges(len(keyframes), covered_ranges)

    for gap_index, (start_idx, end_idx) in enumerate(gap_ranges):
        segment_positions = after_positions[start_idx : end_idx + 1]
        segment_ids = path_labels[start_idx : end_idx + 1]
        segment_ts = path_timestamps[start_idx : end_idx + 1]
        figure.add_trace(
            go.Scatter3d(
                x=segment_positions[:, 0],
                y=segment_positions[:, 1],
                z=segment_positions[:, 2],
                mode="lines",
                name="After alignment (no GPS coverage)" if gap_index == 0 else None,
                legendgroup="after_gap",
                showlegend=gap_index == 0,
                line=dict(color="rgba(80,80,80,0.7)", width=4, dash="dash"),
                hovertemplate=(
                    "keyframe=%{customdata[0]:.0f}<br>"
                    "timestamp=%{customdata[1]:.3f}<br>"
                    "x=%{x:.3f}<br>y=%{y:.3f}<br>z=%{z:.3f}<extra></extra>"
                ),
                customdata=np.column_stack([segment_ids, segment_ts]),
            )
        )

    for segment_index, segment in enumerate(covered_ranges):
        start_idx = int(segment["start_keyframe_index"])
        end_idx = int(segment["end_keyframe_index"])
        if start_idx < 0 or end_idx < start_idx or end_idx >= len(keyframes):
            continue
        segment_positions = after_positions[start_idx : end_idx + 1]
        segment_ids = keyframe_ids[start_idx : end_idx + 1]
        segment_ts = timestamps[start_idx : end_idx + 1]
        figure.add_trace(
            go.Scatter3d(
                x=segment_positions[:, 0],
                y=segment_positions[:, 1],
                z=segment_positions[:, 2],
                mode="lines",
                name="After alignment (GPS covered)" if segment_index == 0 else None,
                legendgroup="after_covered",
                showlegend=segment_index == 0,
                line=dict(color="rgba(0,180,120,0.95)", width=8),
                hovertemplate=(
                    "keyframe=%{customdata[0]}<br>"
                    "timestamp=%{customdata[1]:.3f}<br>"
                    "segment=%{customdata[2]}<br>"
                    "x=%{x:.3f}<br>y=%{y:.3f}<br>z=%{z:.3f}<extra></extra>"
                ),
                customdata=np.column_stack(
                    [segment_ids, segment_ts, np.full(len(segment_ids), segment["segment_id"])]
                ),
            )
        )

    gps_local = np.asarray([anchor["gps_local"] for anchor in anchors], dtype=float)
    figure.add_trace(
        go.Scatter3d(
            x=gps_local[:, 0],
            y=gps_local[:, 1],
            z=gps_local[:, 2],
            mode="markers",
            name="GPS anchors",
            marker=dict(size=4, color="rgba(39,174,96,0.95)", symbol="circle"),
            hovertemplate=(
                "keyframe=%{customdata[0]}<br>"
                "segment=%{customdata[1]}<br>"
                "path=%{customdata[2]:.2f} m<br>"
                "x=%{x:.3f}<br>y=%{y:.3f}<br>z=%{z:.3f}<extra></extra>"
            ),
            customdata=np.column_stack(
                [
                    [anchor["keyframe_id"] for anchor in anchors],
                    [anchor["segment_id"] for anchor in anchors],
                    [anchor.get("path_m", math.nan) for anchor in anchors],
                ]
            ),
        )
    )

    raw_gps_local = np.asarray([anchor["gps_raw_local"] for anchor in anchors], dtype=float)
    figure.add_trace(
        go.Scatter3d(
            x=raw_gps_local[:, 0],
            y=raw_gps_local[:, 1],
            z=raw_gps_local[:, 2],
            mode="markers",
            name="GPS raw",
            marker=dict(size=2, color="rgba(230,126,34,0.55)", symbol="circle"),
            hoverinfo="skip",
            showlegend=True,
        )
    )

    figure.update_layout(
        height=900,
        margin=dict(l=0, r=0, t=92, b=0),
        legend=dict(
            orientation="h",
            yanchor="bottom",
            y=0.92,
            xanchor="left",
            x=0.0,
            bgcolor="rgba(255,255,255,0.92)",
            bordercolor="rgba(0,0,0,0.08)",
            borderwidth=1,
        ),
        scene=dict(
            xaxis_title="X local (m)",
            yaxis_title="Y local (m)",
            zaxis_title="Z local (m)",
            aspectmode="data",
            camera=dict(eye=dict(x=1.6, y=1.6, z=1.0)),
        ),
    )
    return figure


def write_3d_html(
    output_dir: Path,
    keyframes: List[Dict[str, Any]],
    anchors: List[Dict[str, Any]],
    segment_metrics: Dict[str, Any],
) -> Optional[str]:
    figure = build_3d_comparison_figure(keyframes, anchors, segment_metrics)
    if figure is None:
        return None
    path = output_dir / "stage2_3d_comparison.html"
    covered_ratio = segment_metrics.get("covered_path_ratio", math.nan)
    covered_text = ""
    if np.isfinite(covered_ratio):
        covered_text = f"GPS covered path {covered_ratio * 100.0:.1f}%"
    plot_html = pio.to_html(
        figure,
        include_plotlyjs=True,
        full_html=False,
        config={"responsive": True, "displaylogo": False},
    )
    path.write_text(
        f"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Stage2 3D Alignment Comparison</title>
<style>
body {{ margin: 0; font-family: Arial, sans-serif; background: #f7f8fa; color: #222; }}
.wrap {{ padding: 18px 18px 18px; }}
.header {{ padding-bottom: 14px; margin-bottom: 16px; border-bottom: 1px solid rgba(0, 0, 0, 0.08); }}
h1 {{ margin: 0 0 6px 0; font-size: 24px; font-weight: 700; color: #1f2937; line-height: 1.2; }}
.sub {{ margin: 0; color: #4b5563; }}
.plotwrap {{ margin-top: 20px; min-height: 900px; }}
</style>
</head>
<body>
<div class="wrap">
  <div class="header">
    <h1>Stage2 3D alignment comparison</h1>
    <p class="sub">{html.escape(covered_text)}</p>
  </div>
  <div class="plotwrap">
    {plot_html}
  </div>
</div>
</body>
</html>
"""
    )
    return path.name


def generate_plots(
    output_dir: Path,
    keyframes: List[Dict[str, Any]],
    anchors: List[Dict[str, Any]],
    segment_metrics: Dict[str, Any],
    lever_metrics: Dict[str, Any],
    deltas: List[Dict[str, Any]],
    keyframes_by_id: Dict[int, Dict[str, Any]],
    max_points: int,
) -> List[str]:
    if not HAS_MATPLOTLIB:
        print("Warning: matplotlib not available, skipping plots")
        return []
    images = [
        plot_trajectory(output_dir, keyframes, anchors, max_points),
        plot_residuals(output_dir, anchors),
        plot_distribution(output_dir, anchors),
        plot_gps_quality(output_dir, anchors, segment_metrics),
        plot_height(output_dir, keyframes, anchors),
    ]
    lever_plot = plot_lever(output_dir, lever_metrics)
    if lever_plot:
        images.append(lever_plot)
    delta_plot = plot_pose_delta(output_dir, deltas, keyframes_by_id)
    if delta_plot:
        images.append(delta_plot)
    return images


def json_safe(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, float) and not np.isfinite(value):
        return None
    raise TypeError(f"Unsupported JSON type: {type(value)}")


def fmt_stat_line(name: str, value: Dict[str, Any]) -> str:
    if value.get("count", 0) == 0:
        return f"{name}: no data"
    return (
        f"{name}: count={value['count']} rmse={value['rmse']:.4f} "
        f"mean={value['mean']:.4f} median={value['median']:.4f} "
        f"p95={value['p95']:.4f} max={value['max']:.4f}"
    )


def write_text_report(
    output_dir: Path,
    stage2_dir: Path,
    manifest: Dict[str, Any],
    alignment_metrics: Dict[str, Any],
    segment_metrics: Dict[str, Any],
    lever_metrics: Dict[str, Any],
    pose_delta_metrics: Dict[str, Any],
    alignment_summary: Dict[str, str],
    interactive_artifacts: Optional[Dict[str, str]] = None,
) -> str:
    path = output_dir / "stage2_eval_report.txt"
    with path.open("w") as stream:
        stream.write("Stage2 Alignment Evaluation Report\n")
        stream.write("=" * 72 + "\n")
        stream.write(f"Generated: {datetime.now().isoformat(timespec='seconds')}\n")
        stream.write(f"Stage2 dir: {stage2_dir}\n\n")

        stream.write("Provenance\n")
        stream.write("-" * 72 + "\n")
        dataset = manifest.get("dataset", {}) if isinstance(manifest.get("dataset"), dict) else {}
        provenance = manifest.get("provenance", {}) if isinstance(manifest.get("provenance"), dict) else {}
        stream.write(f"Manifest: {manifest.get('manifest_path', '')}\n")
        stream.write(f"Stage2 generated_at: {manifest.get('generated_at', '')}\n")
        stream.write(f"Vehicle: {manifest.get('vehicle_name', '')}\n")
        stream.write(f"Vehicle config: {manifest.get('vehicle_config_path', '')}\n")
        stream.write(f"Run config: {manifest.get('run_config_path', '')}\n")
        stream.write(f"Dataset source count: {dataset.get('source_count', 0)}\n")
        for source in dataset.get("sources", []) or []:
            stream.write(f"Dataset source: {source}\n")
        stream.write(f"Expanded record count: {dataset.get('expanded_record_count', 0)}\n")
        stream.write(
            f"Source Stage1 manifest: {provenance.get('source_stage1_manifest', manifest.get('source_stage1_manifest', ''))}\n"
        )
        stream.write(
            f"Source Stage1 generated_at: {provenance.get('source_stage1_generated_at', manifest.get('source_stage1_generated_at', ''))}\n\n"
        )

        stream.write("Alignment summary\n")
        stream.write("-" * 72 + "\n")
        if alignment_summary:
            for key, value in alignment_summary.items():
                stream.write(f"{key}: {value}\n")
        stream.write(fmt_stat_line("Before 3D residual (m)", alignment_metrics["before_3d_m"]) + "\n")
        stream.write(fmt_stat_line("After 3D residual (m)", alignment_metrics["after_3d_m"]) + "\n")
        stream.write(fmt_stat_line("Before XY residual (m)", alignment_metrics["before_xy_m"]) + "\n")
        stream.write(fmt_stat_line("After XY residual (m)", alignment_metrics["after_xy_m"]) + "\n")
        stream.write(fmt_stat_line("After abs Z residual (m)", alignment_metrics["after_abs_z_m"]) + "\n")
        stream.write(
            "Note: Stage2 now defaults to a single robust full-SE3 rigid "
            "alignment. 3D residual evaluates the global rigid fit, while XY "
            "residual remains the primary horizontal quality metric.\n"
        )
        mean_reduction = alignment_metrics.get("mean_residual_reduction_pct")
        rmse_reduction = alignment_metrics.get("rmse_residual_reduction_pct")
        stream.write(f"Mean residual reduction: {mean_reduction:.2f}%\n" if mean_reduction is not None else "")
        stream.write(f"RMSE residual reduction: {rmse_reduction:.2f}%\n" if rmse_reduction is not None else "")
        stream.write("\n")

        stream.write("GPS segments and coverage\n")
        stream.write("-" * 72 + "\n")
        stream.write(f"Keyframes: {segment_metrics['keyframe_count']}\n")
        stream.write(f"Anchors: {segment_metrics['anchor_count']}\n")
        stream.write(f"Segments: {segment_metrics['segment_count']}\n")
        stream.write(f"Anchor coverage ratio: {segment_metrics['anchor_coverage_ratio']:.4f}\n")
        stream.write(f"Covered path ratio: {segment_metrics['covered_path_ratio']:.4f}\n")
        stream.write(
            f"Covered path: {segment_metrics['covered_path_m']:.2f} m / "
            f"{segment_metrics['total_keyframe_path_m']:.2f} m\n"
        )
        stream.write(fmt_stat_line("Anchor path gap (m)", segment_metrics["anchor_path_gap_m"]) + "\n")
        stream.write(fmt_stat_line("Segment length (m)", segment_metrics["segment_length_m"]) + "\n\n")

        stream.write("Lever-arm calibration\n")
        stream.write("-" * 72 + "\n")
        lever_summary = lever_metrics.get("summary", {})
        if lever_summary:
            for key in (
                "enabled",
                "success",
                "orientation_model",
                "status_message",
                "candidate_count",
                "selected_count",
                "iterations",
                "initial_x",
                "initial_y",
                "initial_z",
                "optimized_x",
                "optimized_y",
                "optimized_z",
                "correction_norm_m",
                "heading_bias_deg",
            ):
                if key in lever_summary:
                    stream.write(f"{key}: {lever_summary[key]}\n")
        cost_reduction = lever_metrics.get("weighted_cost_reduction_pct")
        mean_residual_reduction = lever_metrics.get("mean_residual_reduction_pct")
        stream.write(f"Weighted cost reduction: {cost_reduction:.2f}%\n" if cost_reduction is not None else "")
        stream.write(
            f"Mean residual reduction: {mean_residual_reduction:.2f}%\n"
            if mean_residual_reduction is not None
            else ""
        )
        stream.write(fmt_stat_line("Selected initial residual XY (m)", lever_metrics["residual_initial_xy_m"]) + "\n")
        stream.write(fmt_stat_line("Selected optimized residual XY (m)", lever_metrics["residual_optimized_xy_m"]) + "\n")
        stream.write(f"Reject reasons: {lever_metrics.get('reject_counts', {})}\n\n")

        stream.write("Pose delta\n")
        stream.write("-" * 72 + "\n")
        stream.write(fmt_stat_line("Translation delta (m)", pose_delta_metrics["translation_delta_m"]) + "\n")
        stream.write(fmt_stat_line("Rotation delta (deg)", pose_delta_metrics["rotation_delta_deg"]) + "\n")
        if interactive_artifacts and interactive_artifacts.get("3d_comparison"):
            stream.write(f"\nInteractive 3D view: {interactive_artifacts['3d_comparison']}\n")
    return path.name


def html_stat_table(title: str, metric: Dict[str, Any]) -> str:
    rows = []
    for key in ("count", "rmse", "mean", "median", "std", "p95", "max"):
        value = metric.get(key)
        if value is None:
            continue
        if isinstance(value, float):
            value_text = f"{value:.6g}"
        else:
            value_text = str(value)
        rows.append(f"<tr><td>{html.escape(key)}</td><td>{value_text}</td></tr>")
    return f"<h3>{html.escape(title)}</h3><table>{''.join(rows)}</table>"


def fmt_vec3(values: Any) -> str:
    arr = np.asarray(values, dtype=float)
    if arr.shape != (3,) or not np.all(np.isfinite(arr)):
        return "n/a"
    return f"[{arr[0]:.6f}, {arr[1]:.6f}, {arr[2]:.6f}]"


def html_lever_arm_table(lever_metrics: Dict[str, Any]) -> str:
    summary = lever_metrics.get("summary", {})
    rows = [
        ("Enabled", summary.get("enabled", "n/a")),
        ("Success", summary.get("success", "n/a")),
        ("Status", summary.get("status_message", "n/a")),
        ("Orientation model", summary.get("orientation_model", "n/a")),
        ("Initial lever arm [x, y, z] m", fmt_vec3(lever_metrics.get("initial_lever_arm"))),
        ("Optimized lever arm [x, y, z] m", fmt_vec3(lever_metrics.get("optimized_lever_arm"))),
        ("Correction [x, y, z] m", fmt_vec3(lever_metrics.get("correction"))),
        ("Correction norm m", lever_metrics.get("correction_norm_m", math.nan)),
        ("Heading bias deg", lever_metrics.get("heading_bias_deg", math.nan)),
        ("Candidate samples", lever_metrics.get("candidate_count", 0)),
        ("Selected samples", lever_metrics.get("selected_count", 0)),
        ("Iterations", summary.get("iterations", "n/a")),
    ]
    html_rows = []
    for label, value in rows:
        if isinstance(value, float):
            value_text = "n/a" if not np.isfinite(value) else f"{value:.6f}"
        else:
            value_text = str(value)
        html_rows.append(
            f"<tr><td>{html.escape(label)}</td><td>{html.escape(value_text)}</td></tr>"
        )
    return f"<h3>Lever-arm values</h3><table>{''.join(html_rows)}</table>"


def write_html_report(
    output_dir: Path,
    stage2_dir: Path,
    manifest: Dict[str, Any],
    images: List[str],
    alignment_metrics: Dict[str, Any],
    segment_metrics: Dict[str, Any],
    lever_metrics: Dict[str, Any],
    pose_delta_metrics: Dict[str, Any],
    text_report: str,
    interactive_artifacts: Optional[Dict[str, str]] = None,
) -> str:
    interactive_artifacts = interactive_artifacts or {}
    after = alignment_metrics["after_3d_m"]
    before = alignment_metrics["before_3d_m"]
    after_xy = alignment_metrics["after_xy_m"]
    reduction = alignment_metrics.get("mean_residual_reduction_pct")
    coverage_ratio = segment_metrics.get("covered_path_ratio", 0.0)
    lever_summary = lever_metrics.get("summary", {})
    lever_initial = fmt_vec3(lever_metrics.get("initial_lever_arm"))
    lever_optimized = fmt_vec3(lever_metrics.get("optimized_lever_arm"))
    lever_correction = fmt_vec3(lever_metrics.get("correction"))
    lever_correction_norm = lever_metrics.get("correction_norm_m", math.nan)
    image_html = "\n".join(
        f'<section><h2>{html.escape(Path(image).stem.replace("_", " ").title())}</h2>'
        f'<img src="{html.escape(image)}" alt="{html.escape(image)}"></section>'
        for image in images
    )
    interactive_html = ""
    if interactive_artifacts:
        blocks = []
        for label, artifact_path in interactive_artifacts.items():
            title = html.escape(label.replace("_", " ").title())
            escaped_path = html.escape(artifact_path)
            block = (
                f"<section><h2>{title}</h2>"
                f'<p><a href="{escaped_path}">{escaped_path}</a></p>'
            )
            if label == "3d_comparison":
                block += (
                    f'<iframe src="{escaped_path}" title="{title}" '
                    'style="width:100%; height:780px; border:1px solid #ddd;"></iframe>'
                )
            block += "</section>"
            blocks.append(block)
        interactive_html = "\n".join(blocks)
    dataset = manifest.get("dataset", {}) if isinstance(manifest.get("dataset"), dict) else {}
    provenance = manifest.get("provenance", {}) if isinstance(manifest.get("provenance"), dict) else {}
    html_text = f"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Stage2 Alignment Evaluation</title>
<style>
body {{ font-family: Arial, sans-serif; margin: 24px; color: #222; background: #f7f8fa; }}
h1, h2, h3 {{ color: #1f2937; }}
.cards {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(230px, 1fr)); gap: 14px; }}
.card {{ background: white; border: 1px solid #ddd; border-radius: 6px; padding: 14px; }}
.value {{ font-size: 24px; font-weight: 700; color: #1f77b4; }}
section {{ background: white; border: 1px solid #ddd; border-radius: 6px; margin-top: 18px; padding: 14px; }}
img {{ max-width: 100%; height: auto; border: 1px solid #eee; }}
table {{ border-collapse: collapse; width: 100%; margin: 8px 0 18px; }}
td, th {{ border: 1px solid #ddd; padding: 6px 8px; }}
td:first-child {{ font-weight: 600; width: 260px; }}
a {{ color: #1f77b4; }}
</style>
</head>
<body>
<h1>Stage2 Alignment Evaluation</h1>
<p>Stage2 dir: <code>{html.escape(str(stage2_dir))}</code></p>
<section>
<h2>Provenance</h2>
<p>Manifest: <code>{html.escape(str(manifest.get('manifest_path', '')))}</code></p>
<p>Generated at: <code>{html.escape(str(manifest.get('generated_at', '')))}</code></p>
<p>Vehicle: <code>{html.escape(str(manifest.get('vehicle_name', '')))}</code></p>
<p>Vehicle config: <code>{html.escape(str(manifest.get('vehicle_config_path', '')))}</code></p>
<p>Run config: <code>{html.escape(str(manifest.get('run_config_path', '')))}</code></p>
<p>Dataset sources: <code>{html.escape(str(dataset.get('source_count', 0)))}</code></p>
<p>Expanded records: <code>{html.escape(str(dataset.get('expanded_record_count', 0)))}</code></p>
<p>Stage1 manifest: <code>{html.escape(str(provenance.get('source_stage1_manifest', manifest.get('source_stage1_manifest', ''))))}</code></p>
<p>Stage1 generated_at: <code>{html.escape(str(provenance.get('source_stage1_generated_at', manifest.get('source_stage1_generated_at', ''))))}</code></p>
</section>
<p>Text report: <a href="{html.escape(text_report)}">{html.escape(text_report)}</a></p>
<div class="cards">
  <div class="card"><h3>Anchors</h3><div class="value">{segment_metrics.get("anchor_count", 0)}</div></div>
  <div class="card"><h3>Segments</h3><div class="value">{segment_metrics.get("segment_count", 0)}</div></div>
  <div class="card"><h3>Coverage Ratio</h3><div class="value">{coverage_ratio:.4f}</div></div>
  <div class="card"><h3>After Mean XY</h3><div class="value">{after_xy.get("mean", 0.0):.4f} m</div></div>
  <div class="card"><h3>After P95 XY</h3><div class="value">{after_xy.get("p95", 0.0):.4f} m</div></div>
  <div class="card"><h3>After Mean 3D</h3><div class="value">{after.get("mean", 0.0):.4f} m</div></div>
  <div class="card"><h3>Mean Reduction</h3><div class="value">{0.0 if reduction is None else reduction:.1f}%</div></div>
  <div class="card"><h3>Lever Status</h3><div class="value">{html.escape(lever_summary.get("status_message", "n/a"))}</div></div>
  <div class="card"><h3>Lever Correction</h3><div class="value">{0.0 if not np.isfinite(lever_correction_norm) else lever_correction_norm:.4f} m</div></div>
</div>
<section>
<h2>Lever-arm Calibration</h2>
<p>Initial lever arm: <code>{html.escape(lever_initial)}</code></p>
<p>Optimized lever arm: <code>{html.escape(lever_optimized)}</code></p>
<p>Correction: <code>{html.escape(lever_correction)}</code></p>
{html_lever_arm_table(lever_metrics)}
{html_stat_table("Selected initial lever-arm residual XY (m)", lever_metrics["residual_initial_xy_m"])}
{html_stat_table("Selected optimized lever-arm residual XY (m)", lever_metrics["residual_optimized_xy_m"])}
</section>
<section>
<h2>Metric Tables</h2>
{html_stat_table("Before 3D residual (m)", before)}
{html_stat_table("After 3D residual (m)", after)}
{html_stat_table("After XY residual (m)", alignment_metrics["after_xy_m"])}
{html_stat_table("Pose translation delta (m)", pose_delta_metrics["translation_delta_m"])}
</section>
{interactive_html}
{image_html}
</body>
</html>
"""
    path = output_dir / "stage2_eval_report.html"
    path.write_text(html_text)
    return path.name


def evaluate(args: argparse.Namespace) -> Path:
    stage2_dir = Path(args.stage2_dir).expanduser().resolve()
    output_dir = create_output_dir(Path(args.output_dir).expanduser().resolve(), args.timestamp)
    manifest = load_manifest(stage2_dir)

    keyframes = load_keyframes(stage2_dir)
    anchors = load_anchors(stage2_dir)
    deltas = load_pose_delta(stage2_dir)
    if not keyframes:
        raise RuntimeError(f"No Stage2 keyframes found under {stage2_dir}")
    if not anchors:
        raise RuntimeError(f"No Stage2 alignment anchors found under {stage2_dir}")

    keyframes_by_id = add_path_distance(keyframes)
    utm_origin = read_utm_origin(stage2_dir, anchors)
    enrich_anchors(anchors, keyframes_by_id, utm_origin)

    alignment_metrics = compute_alignment_metrics(anchors)
    segment_metrics = compute_segment_metrics(stage2_dir, anchors, keyframes)
    lever_metrics = compute_lever_metrics(stage2_dir)
    pose_delta_metrics = compute_pose_delta_metrics(deltas)
    alignment_summary = load_single_csv(stage2_dir, "diagnostics/alignment_summary.csv")

    interactive_artifacts: Dict[str, str] = {}
    images: List[str] = []
    if not args.no_plots:
        images = generate_plots(
            output_dir,
            keyframes,
            anchors,
            segment_metrics,
            lever_metrics,
            deltas,
            keyframes_by_id,
            args.max_plot_points,
        )
        if HAS_PLOTLY:
            html_3d = write_3d_html(output_dir, keyframes, anchors, segment_metrics)
            if html_3d:
                interactive_artifacts["3d_comparison"] = html_3d

    report_name = write_text_report(
        output_dir,
        stage2_dir,
        manifest,
        alignment_metrics,
        segment_metrics,
        lever_metrics,
        pose_delta_metrics,
        alignment_summary,
        interactive_artifacts,
    )

    stats_payload = {
        "stage2_dir": str(stage2_dir),
        "manifest": manifest,
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "utm_origin": utm_origin,
        "alignment": alignment_metrics,
        "segments": segment_metrics,
        "lever_arm": {key: value for key, value in lever_metrics.items() if key != "samples"},
        "pose_delta": pose_delta_metrics,
        "alignment_summary": alignment_summary,
        "plots": images,
        "interactive_plots": interactive_artifacts,
    }
    (output_dir / "stage2_eval_stats.json").write_text(json.dumps(stats_payload, indent=2, default=json_safe))
    write_html_report(
        output_dir,
        stage2_dir,
        manifest,
        images,
        alignment_metrics,
        segment_metrics,
        lever_metrics,
        pose_delta_metrics,
        report_name,
        interactive_artifacts,
    )

    after = alignment_metrics["after_3d_m"]
    before = alignment_metrics["before_3d_m"]
    print(f"Stage2 eval written to: {output_dir}")
    print(f"Anchors: {segment_metrics['anchor_count']}, segments: {segment_metrics['segment_count']}")
    print(
        f"Coverage ratio: {segment_metrics['covered_path_ratio']:.4f} "
        f"({segment_metrics['covered_path_m']:.2f} / {segment_metrics['total_keyframe_path_m']:.2f} m)"
    )
    print(f"Before mean 3D residual: {before.get('mean', math.nan):.4f} m")
    print(f"After mean 3D residual:  {after.get('mean', math.nan):.4f} m")
    if interactive_artifacts.get("3d_comparison"):
        print(f"3D view: {output_dir / interactive_artifacts['3d_comparison']}")
    return output_dir


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    run_config = read_top_level_config(script_dir.parent / "conf" / "current_vehicle.yaml")
    vehicle = run_config["active_vehicle"]
    default_stage2_dir = Path(run_config["data_root"]) / vehicle / "stage2_graph_opt"
    default_output_dir = Path(run_config["debug_root"]) / "stage2_alignment" / vehicle / "stage2"
    parser = argparse.ArgumentParser(description="Evaluate and visualize air_mapping Stage2 alignment artifacts.")
    parser.add_argument("--stage2_dir", default=str(default_stage2_dir), help="Path to Stage2 output directory.")
    parser.add_argument("--output_dir", default=str(default_output_dir), help="Base output directory for reports.")
    parser.add_argument(
        "--timestamp",
        action="store_true",
        help="Write to a timestamped subdirectory instead of overwriting output_dir.",
    )
    parser.add_argument(
        "--no_timestamp",
        action="store_false",
        dest="timestamp",
        help=argparse.SUPPRESS,
    )
    parser.set_defaults(timestamp=False)
    parser.add_argument("--no_plots", action="store_true", help="Only write JSON and text reports.")
    parser.add_argument("--max_plot_points", type=int, default=8000, help="Maximum trajectory points per plot.")
    return parser.parse_args()


def main() -> int:
    try:
        evaluate(parse_args())
    except Exception as exc:
        print(f"Error: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
