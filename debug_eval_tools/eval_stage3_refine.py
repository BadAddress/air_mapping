#!/usr/bin/env python3
"""
Stage 3 graph refinement evaluation and visualization tool.

Reads air_mapping Stage 3 artifacts and produces metrics, plots, a text report,
JSON stats, and a compact HTML report. The tool is read-only and does not
modify Stage 3 outputs.
"""

import argparse
import csv
import html
import json
import math
import shutil
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

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
    plt = None
    HAS_MATPLOTLIB = False

try:
    import plotly.graph_objects as go
    import plotly.io as pio

    HAS_PLOTLY = True
except Exception:
    go = None
    pio = None
    HAS_PLOTLY = False


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
        [
            to_float(row, f"{prefix}_x"),
            to_float(row, f"{prefix}_y"),
            to_float(row, f"{prefix}_z"),
        ],
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


def load_single_csv(stage3_dir: Path, relative: str) -> Dict[str, str]:
    rows = read_csv_rows(stage3_dir / relative)
    return rows[0] if rows else {}


def load_manifest(stage3_dir: Path) -> Dict[str, Any]:
    manifest_path = stage3_dir / "manifest.yaml"
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


def manifest_utm_origin(manifest: Dict[str, Any]) -> np.ndarray:
    value = manifest.get("utm_origin", [0.0, 0.0, 0.0])
    if isinstance(value, str):
        value = value.strip().strip("[]").split(",")
    try:
        arr = np.asarray([float(v) for v in value], dtype=float)
        if arr.shape == (3,) and np.all(np.isfinite(arr)):
            return arr
    except (TypeError, ValueError):
        pass
    return np.zeros(3, dtype=float)


def load_keyframes(stage3_dir: Path) -> List[Dict[str, Any]]:
    rows = read_csv_rows(stage3_dir / "keyframes" / "keyframes_refined.csv")
    keyframes: List[Dict[str, Any]] = []
    for index, row in enumerate(rows):
        stage2 = parse_vec(row, "stage2")
        refined = parse_vec(row, "refined")
        utm = parse_vec(row, "utm")
        delta = refined - stage2
        keyframes.append(
            {
                "id": to_int(row, "id"),
                "index": index,
                "timestamp": to_float(row, "timestamp"),
                "stage2": stage2,
                "refined": refined,
                "utm": utm,
                "delta": delta,
                "delta_norm": float(np.linalg.norm(delta)) if np.all(np.isfinite(delta)) else math.nan,
                "has_gps": to_int(row, "has_gps"),
            }
        )
    return keyframes


def load_pose_delta(stage3_dir: Path) -> List[Dict[str, Any]]:
    rows = read_csv_rows(stage3_dir / "keyframes" / "pose_delta.csv")
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


def load_gps_priors(stage3_dir: Path, utm_origin: np.ndarray) -> List[Dict[str, Any]]:
    rows = read_csv_rows(stage3_dir / "diagnostics" / "gps_priors.csv")
    priors: List[Dict[str, Any]] = []
    for row in rows:
        gps_utm = parse_vec(row, "gps_smooth_utm")
        gps_local = gps_utm - utm_origin
        used = to_int(row, "stage3_used") != 0
        priors.append(
            {
                "keyframe_id": to_int(row, "keyframe_id"),
                "timestamp": to_float(row, "timestamp"),
                "segment_id": to_int(row, "segment_id", -1),
                "stage2_weight": to_float(row, "stage2_weight"),
                "gps_utm": gps_utm,
                "gps_local": gps_local,
                "std": np.array([to_float(row, "std_x"), to_float(row, "std_y"), to_float(row, "std_z")], dtype=float),
                "stage3_used": used,
                "stage3_info": np.array(
                    [
                        to_float(row, "stage3_info_x"),
                        to_float(row, "stage3_info_y"),
                        to_float(row, "stage3_info_z"),
                    ],
                    dtype=float,
                ),
                "stage3_ramp_scale": to_float(row, "stage3_ramp_scale"),
                "stage2_reported_residual_m": to_float(row, "stage2_reported_residual_m"),
                "stage2_recomputed_residual_m": to_float(row, "stage2_recomputed_residual_m"),
                "stage3_residual_m": to_float(row, "stage3_residual_m"),
            }
        )
    return priors


def load_outage_blocks(stage3_dir: Path) -> List[Dict[str, Any]]:
    rows = read_csv_rows(stage3_dir / "diagnostics" / "outage_blocks.csv")
    blocks: List[Dict[str, Any]] = []
    for row in rows:
        blocks.append(
            {
                "block_id": to_int(row, "block_id"),
                "start_keyframe_id": to_int(row, "start_keyframe_id"),
                "end_keyframe_id": to_int(row, "end_keyframe_id"),
                "representative_keyframe_id": to_int(row, "representative_keyframe_id"),
                "left_anchor_index": to_int(row, "left_anchor_index", -1),
                "right_anchor_index": to_int(row, "right_anchor_index", -1),
                "path_length_m": to_float(row, "path_length_m"),
                "icp_valid": to_int(row, "icp_valid") != 0,
                "icp_fitness": to_float(row, "icp_fitness"),
                "source_keyframes": to_int(row, "source_keyframes"),
                "target_keyframes": to_int(row, "target_keyframes"),
                "prior": parse_vec(row, "prior"),
            }
        )
    return blocks


def add_path_distance(keyframes: List[Dict[str, Any]]) -> Dict[int, Dict[str, Any]]:
    distance = 0.0
    previous: Optional[np.ndarray] = None
    by_id: Dict[int, Dict[str, Any]] = {}
    for keyframe in keyframes:
        refined = keyframe["refined"]
        if previous is not None and np.all(np.isfinite(refined)) and np.all(np.isfinite(previous)):
            distance += float(np.linalg.norm(refined - previous))
        keyframe["path_m"] = distance
        previous = refined
        by_id[keyframe["id"]] = keyframe
    return by_id


def enrich_gps_priors(priors: List[Dict[str, Any]], keyframes_by_id: Dict[int, Dict[str, Any]]) -> None:
    for prior in priors:
        keyframe = keyframes_by_id.get(prior["keyframe_id"])
        if keyframe is None:
            prior["path_m"] = math.nan
            prior["stage2"] = np.full(3, math.nan)
            prior["refined"] = np.full(3, math.nan)
            continue
        prior["path_m"] = keyframe.get("path_m", math.nan)
        prior["stage2"] = keyframe["stage2"]
        prior["refined"] = keyframe["refined"]


def compute_metrics(
    keyframes: List[Dict[str, Any]],
    deltas: List[Dict[str, Any]],
    priors: List[Dict[str, Any]],
    blocks: List[Dict[str, Any]],
    refine_summary: Dict[str, str],
) -> Dict[str, Any]:
    used_priors = [prior for prior in priors if prior["stage3_used"]]
    before_mean = stats(prior["stage2_recomputed_residual_m"] for prior in used_priors).get("mean", math.nan)
    after_mean = stats(prior["stage3_residual_m"] for prior in used_priors).get("mean", math.nan)
    return {
        "summary": dict(refine_summary),
        "keyframe_count": len(keyframes),
        "gps_prior_count": len(priors),
        "gps_prior_used_count": len(used_priors),
        "gps_prior_used_ratio": float(len(used_priors) / max(len(priors), 1)),
        "outage_block_count": len(blocks),
        "outage_icp_valid_count": sum(1 for block in blocks if block["icp_valid"]),
        "gps_stage2_recomputed_residual_m": stats(prior["stage2_recomputed_residual_m"] for prior in used_priors),
        "gps_stage3_residual_m": stats(prior["stage3_residual_m"] for prior in used_priors),
        "gps_reported_stage2_residual_m": stats(prior["stage2_reported_residual_m"] for prior in used_priors),
        "gps_residual_reduction_pct": pct_reduction(before_mean, after_mean),
        "pose_translation_delta_m": stats(delta["translation_norm"] for delta in deltas),
        "pose_dx_m": stats(delta["delta"][0] for delta in deltas),
        "pose_dy_m": stats(delta["delta"][1] for delta in deltas),
        "pose_dz_m": stats(delta["delta"][2] for delta in deltas),
        "pose_rotation_delta_deg": stats(delta["rotation_delta_deg"] for delta in deltas),
        "outage_path_length_m": stats(block["path_length_m"] for block in blocks),
        "outage_icp_fitness": stats(block["icp_fitness"] for block in blocks if block["icp_valid"]),
    }


def create_output_dir(base_dir: Path, timestamped: bool) -> Path:
    output_dir = base_dir / datetime.now().strftime("%Y%m%d_%H%M%S") if timestamped else base_dir
    if not timestamped and output_dir.exists():
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


def array_from(items: List[Dict[str, Any]], key: str) -> np.ndarray:
    return np.asarray([item.get(key, math.nan) for item in items], dtype=float)


def plot_trajectory(output_dir: Path, keyframes: List[Dict[str, Any]], priors: List[Dict[str, Any]], max_points: int) -> str:
    fig, ax = plt.subplots(figsize=(13, 9.5), constrained_layout=True)
    idx = downsample_indices(len(keyframes), max_points)
    stage2 = np.asarray([keyframes[i]["stage2"] for i in idx], dtype=float)
    refined = np.asarray([keyframes[i]["refined"] for i in idx], dtype=float)
    if stage2.size:
        ax.plot(stage2[:, 0], stage2[:, 1], color="#d62728", linewidth=1.2, label="Stage2 input")
    if refined.size:
        ax.plot(refined[:, 0], refined[:, 1], color="#1f77b4", linewidth=1.5, label="Stage3 refined")
    used = [prior for prior in priors if prior["stage3_used"]]
    unused = [prior for prior in priors if not prior["stage3_used"]]
    if used:
        gps = np.asarray([prior["gps_local"] for prior in used], dtype=float)
        ax.scatter(gps[:, 0], gps[:, 1], color="#2ca02c", s=16, alpha=0.8, label="Used GPS priors")
    if unused:
        gps = np.asarray([prior["gps_local"] for prior in unused], dtype=float)
        ax.scatter(gps[:, 0], gps[:, 1], color="#999999", s=9, alpha=0.35, label="Unused GPS priors")
    ax.set_title("Stage3 refined trajectory in UTM-local frame")
    ax.set_xlabel("X local (m)")
    ax.set_ylabel("Y local (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    return save_figure(fig, output_dir, "stage3_trajectory_refine.png")


def plot_gps_residuals(output_dir: Path, priors: List[Dict[str, Any]]) -> Optional[str]:
    used = [prior for prior in priors if prior["stage3_used"]]
    if not used:
        return None
    path = array_from(used, "path_m")
    fig, axes = plt.subplots(2, 1, figsize=(13, 8.8), sharex=True, constrained_layout=True)
    axes[0].plot(path, array_from(used, "stage2_recomputed_residual_m"), label="Stage2 recomputed", color="#d62728")
    axes[0].plot(path, array_from(used, "stage3_residual_m"), label="Stage3", color="#1f77b4")
    axes[0].set_ylabel("GPS residual (m)")
    axes[0].set_title("Used GPS prior residuals")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")

    axes[1].plot(path, array_from(used, "stage3_ramp_scale"), label="Ramp scale", color="#9467bd")
    info = np.asarray([prior["stage3_info"][0] for prior in used], dtype=float)
    axes[1].plot(path, info, label="GPS info X", color="#ff7f0e")
    axes[1].set_xlabel("Path distance (m)")
    axes[1].set_ylabel("Scale / information")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best")
    return save_figure(fig, output_dir, "stage3_gps_prior_residuals.png")


def plot_pose_delta(output_dir: Path, deltas: List[Dict[str, Any]], keyframes_by_id: Dict[int, Dict[str, Any]]) -> Optional[str]:
    if not deltas:
        return None
    path = np.asarray([keyframes_by_id.get(delta["id"], {}).get("path_m", math.nan) for delta in deltas], dtype=float)
    delta_vec = np.asarray([delta["delta"] for delta in deltas], dtype=float)
    trans = np.asarray([delta["translation_norm"] for delta in deltas], dtype=float)
    rot = np.asarray([delta["rotation_delta_deg"] for delta in deltas], dtype=float)
    fig, axes = plt.subplots(2, 1, figsize=(13, 8.8), sharex=True, constrained_layout=True)
    axes[0].plot(path, delta_vec[:, 0], label="dx")
    axes[0].plot(path, delta_vec[:, 1], label="dy")
    axes[0].plot(path, delta_vec[:, 2], label="dz")
    axes[0].plot(path, trans, label="translation norm", color="#111111", linewidth=1.2)
    axes[0].set_title("Stage3 pose delta from Stage2")
    axes[0].set_ylabel("Translation delta (m)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")
    axes[1].plot(path, rot, color="#ff7f0e")
    axes[1].set_xlabel("Path distance (m)")
    axes[1].set_ylabel("Rotation delta (deg)")
    axes[1].grid(True, alpha=0.3)
    return save_figure(fig, output_dir, "stage3_pose_delta.png")


def plot_outage_blocks(output_dir: Path, blocks: List[Dict[str, Any]]) -> Optional[str]:
    if not blocks:
        return None
    ids = [block["block_id"] for block in blocks]
    lengths = [block["path_length_m"] for block in blocks]
    fitness = [block["icp_fitness"] if block["icp_valid"] else math.nan for block in blocks]
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.8), constrained_layout=True)
    axes[0].bar(ids, lengths, color="#9467bd")
    axes[0].set_title("Outage block length")
    axes[0].set_xlabel("Block id")
    axes[0].set_ylabel("Path length (m)")
    axes[0].grid(True, axis="y", alpha=0.3)
    axes[1].bar(ids, fitness, color="#2ca02c")
    axes[1].set_title("Outage ICP fitness")
    axes[1].set_xlabel("Block id")
    axes[1].set_ylabel("ICP fitness")
    axes[1].grid(True, axis="y", alpha=0.3)
    return save_figure(fig, output_dir, "stage3_outage_blocks.png")


def build_3d_figure(keyframes: List[Dict[str, Any]], priors: List[Dict[str, Any]]) -> Optional[Any]:
    if not HAS_PLOTLY or not keyframes:
        return None
    stage2 = np.asarray([keyframe["stage2"] for keyframe in keyframes], dtype=float)
    refined = np.asarray([keyframe["refined"] for keyframe in keyframes], dtype=float)
    ids = np.asarray([keyframe["id"] for keyframe in keyframes], dtype=float)
    timestamps = np.asarray([keyframe["timestamp"] for keyframe in keyframes], dtype=float)
    fig = go.Figure()
    fig.add_trace(
        go.Scatter3d(
            x=stage2[:, 0],
            y=stage2[:, 1],
            z=stage2[:, 2],
            mode="lines",
            name="Stage2 input",
            line=dict(color="rgba(214,39,40,0.55)", width=4),
            hoverinfo="skip",
        )
    )
    fig.add_trace(
        go.Scatter3d(
            x=refined[:, 0],
            y=refined[:, 1],
            z=refined[:, 2],
            mode="lines",
            name="Stage3 refined",
            line=dict(color="rgba(31,119,180,0.85)", width=5),
            customdata=np.column_stack([ids, timestamps]),
            hovertemplate="keyframe=%{customdata[0]:.0f}<br>timestamp=%{customdata[1]:.3f}<br>x=%{x:.3f}<br>y=%{y:.3f}<br>z=%{z:.3f}<extra></extra>",
        )
    )
    used = [prior for prior in priors if prior["stage3_used"]]
    if used:
        gps = np.asarray([prior["gps_local"] for prior in used], dtype=float)
        fig.add_trace(
            go.Scatter3d(
                x=gps[:, 0],
                y=gps[:, 1],
                z=gps[:, 2],
                mode="markers",
                name="Used GPS priors",
                marker=dict(size=4, color="rgba(39,174,96,0.95)"),
                hoverinfo="skip",
            )
        )
    fig.update_layout(
        height=900,
        margin=dict(l=0, r=0, t=80, b=0),
        legend=dict(orientation="h", yanchor="bottom", y=0.92, xanchor="left", x=0.0),
        scene=dict(xaxis_title="X local (m)", yaxis_title="Y local (m)", zaxis_title="Z local (m)", aspectmode="data"),
    )
    return fig


def write_3d_html(output_dir: Path, keyframes: List[Dict[str, Any]], priors: List[Dict[str, Any]]) -> Optional[str]:
    fig = build_3d_figure(keyframes, priors)
    if fig is None:
        return None
    path = output_dir / "stage3_3d_refine.html"
    plot_html = pio.to_html(fig, include_plotlyjs=True, full_html=False, config={"responsive": True, "displaylogo": False})
    path.write_text(
        f"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Stage3 3D Refinement Comparison</title>
<style>
body {{ margin: 0; font-family: Arial, sans-serif; background: #f7f8fa; color: #222; }}
.wrap {{ padding: 18px; }}
h1 {{ margin: 0 0 14px 0; font-size: 24px; color: #1f2937; }}
.plotwrap {{ min-height: 900px; }}
</style>
</head>
<body>
<div class="wrap">
  <h1>Stage3 3D refinement comparison</h1>
  <div class="plotwrap">{plot_html}</div>
</div>
</body>
</html>
"""
    )
    return path.name


def generate_plots(
    output_dir: Path,
    keyframes: List[Dict[str, Any]],
    priors: List[Dict[str, Any]],
    deltas: List[Dict[str, Any]],
    blocks: List[Dict[str, Any]],
    keyframes_by_id: Dict[int, Dict[str, Any]],
    max_points: int,
) -> List[str]:
    if not HAS_MATPLOTLIB:
        print("Warning: matplotlib not available, skipping plots")
        return []
    images = [plot_trajectory(output_dir, keyframes, priors, max_points)]
    gps_plot = plot_gps_residuals(output_dir, priors)
    if gps_plot:
        images.append(gps_plot)
    delta_plot = plot_pose_delta(output_dir, deltas, keyframes_by_id)
    if delta_plot:
        images.append(delta_plot)
    outage_plot = plot_outage_blocks(output_dir, blocks)
    if outage_plot:
        images.append(outage_plot)
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


def fmt_stat_line(name: str, metric: Dict[str, Any]) -> str:
    if metric.get("count", 0) == 0:
        return f"{name}: no data"
    return (
        f"{name}: count={metric['count']} rmse={metric['rmse']:.4f} "
        f"mean={metric['mean']:.4f} median={metric['median']:.4f} "
        f"p95={metric['p95']:.4f} max={metric['max']:.4f}"
    )


def html_stat_table(title: str, metric: Dict[str, Any]) -> str:
    rows = []
    for key in ("count", "rmse", "mean", "median", "std", "p95", "max"):
        value = metric.get(key)
        if value is None:
            continue
        value_text = f"{value:.6g}" if isinstance(value, float) else str(value)
        rows.append(f"<tr><td>{html.escape(key)}</td><td>{html.escape(value_text)}</td></tr>")
    return f"<h3>{html.escape(title)}</h3><table>{''.join(rows)}</table>"


def write_text_report(
    output_dir: Path,
    stage3_dir: Path,
    manifest: Dict[str, Any],
    metrics: Dict[str, Any],
    interactive_artifacts: Optional[Dict[str, str]],
) -> str:
    path = output_dir / "stage3_eval_report.txt"
    summary = metrics.get("summary", {})
    with path.open("w") as stream:
        stream.write("Stage3 Graph Refinement Evaluation Report\n")
        stream.write("=" * 72 + "\n")
        stream.write(f"Generated: {datetime.now().isoformat(timespec='seconds')}\n")
        stream.write(f"Stage3 dir: {stage3_dir}\n\n")

        stream.write("Provenance\n")
        stream.write("-" * 72 + "\n")
        provenance = manifest.get("provenance", {}) if isinstance(manifest.get("provenance"), dict) else {}
        stream.write(f"Manifest: {manifest.get('manifest_path', '')}\n")
        stream.write(f"Stage3 generated_at: {manifest.get('generated_at', '')}\n")
        stream.write(f"Vehicle: {manifest.get('vehicle_name', '')}\n")
        stream.write(f"Source Stage2 dir: {provenance.get('source_stage2_dir', '')}\n")
        stream.write(f"Source Stage2 manifest: {provenance.get('source_stage2_manifest', '')}\n\n")

        stream.write("Refine summary\n")
        stream.write("-" * 72 + "\n")
        for key, value in summary.items():
            stream.write(f"{key}: {value}\n")
        stream.write("\n")

        stream.write("GPS priors\n")
        stream.write("-" * 72 + "\n")
        stream.write(f"GPS priors: {metrics['gps_prior_count']}\n")
        stream.write(f"Used GPS priors: {metrics['gps_prior_used_count']}\n")
        stream.write(f"Used ratio: {metrics['gps_prior_used_ratio']:.4f}\n")
        reduction = metrics.get("gps_residual_reduction_pct")
        if reduction is not None:
            stream.write(f"Mean residual reduction: {reduction:.2f}%\n")
        stream.write(fmt_stat_line("Stage2 recomputed GPS residual (m)", metrics["gps_stage2_recomputed_residual_m"]) + "\n")
        stream.write(fmt_stat_line("Stage3 GPS residual (m)", metrics["gps_stage3_residual_m"]) + "\n\n")

        stream.write("Pose delta from Stage2\n")
        stream.write("-" * 72 + "\n")
        stream.write(fmt_stat_line("Translation delta (m)", metrics["pose_translation_delta_m"]) + "\n")
        stream.write(fmt_stat_line("Rotation delta (deg)", metrics["pose_rotation_delta_deg"]) + "\n\n")

        stream.write("Outage blocks\n")
        stream.write("-" * 72 + "\n")
        stream.write(f"Outage blocks: {metrics['outage_block_count']}\n")
        stream.write(f"Valid ICP priors: {metrics['outage_icp_valid_count']}\n")
        stream.write(fmt_stat_line("Outage path length (m)", metrics["outage_path_length_m"]) + "\n")
        stream.write(fmt_stat_line("Outage ICP fitness", metrics["outage_icp_fitness"]) + "\n")
        if interactive_artifacts and interactive_artifacts.get("3d_refine"):
            stream.write(f"\nInteractive 3D view: {interactive_artifacts['3d_refine']}\n")
    return path.name


def write_html_report(
    output_dir: Path,
    stage3_dir: Path,
    manifest: Dict[str, Any],
    metrics: Dict[str, Any],
    images: List[str],
    text_report: str,
    interactive_artifacts: Optional[Dict[str, str]],
) -> str:
    interactive_artifacts = interactive_artifacts or {}
    summary = metrics.get("summary", {})
    gps_after = metrics["gps_stage3_residual_m"]
    pose_delta = metrics["pose_translation_delta_m"]
    rot_delta = metrics["pose_rotation_delta_deg"]
    reduction = metrics.get("gps_residual_reduction_pct")
    image_html = "\n".join(
        f'<section><h2>{html.escape(Path(image).stem.replace("_", " ").title())}</h2>'
        f'<img src="{html.escape(image)}" alt="{html.escape(image)}"></section>'
        for image in images
    )
    interactive_html = ""
    if interactive_artifacts.get("3d_refine"):
        path = html.escape(interactive_artifacts["3d_refine"])
        interactive_html = (
            f"<section><h2>3D Refine Comparison</h2><p><a href=\"{path}\">{path}</a></p>"
            f"<iframe src=\"{path}\" title=\"3D Refine Comparison\" "
            "style=\"width:100%; height:780px; border:1px solid #ddd;\"></iframe></section>"
        )
    provenance = manifest.get("provenance", {}) if isinstance(manifest.get("provenance"), dict) else {}
    html_text = f"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Stage3 Graph Refinement Evaluation</title>
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
<h1>Stage3 Graph Refinement Evaluation</h1>
<p>Stage3 dir: <code>{html.escape(str(stage3_dir))}</code></p>
<section>
<h2>Provenance</h2>
<p>Manifest: <code>{html.escape(str(manifest.get('manifest_path', '')))}</code></p>
<p>Generated at: <code>{html.escape(str(manifest.get('generated_at', '')))}</code></p>
<p>Vehicle: <code>{html.escape(str(manifest.get('vehicle_name', '')))}</code></p>
<p>Source Stage2 dir: <code>{html.escape(str(provenance.get('source_stage2_dir', '')))}</code></p>
<p>Source Stage2 manifest: <code>{html.escape(str(provenance.get('source_stage2_manifest', '')))}</code></p>
</section>
<p>Text report: <a href="{html.escape(text_report)}">{html.escape(text_report)}</a></p>
<div class="cards">
  <div class="card"><h3>Keyframes</h3><div class="value">{metrics.get("keyframe_count", 0)}</div></div>
  <div class="card"><h3>GPS Priors Used</h3><div class="value">{metrics.get("gps_prior_used_count", 0)}</div></div>
  <div class="card"><h3>GPS Used Ratio</h3><div class="value">{metrics.get("gps_prior_used_ratio", 0.0):.4f}</div></div>
  <div class="card"><h3>Stage3 GPS Mean</h3><div class="value">{gps_after.get("mean", 0.0):.4f} m</div></div>
  <div class="card"><h3>GPS Mean Reduction</h3><div class="value">{0.0 if reduction is None else reduction:.1f}%</div></div>
  <div class="card"><h3>Mean Pose Delta</h3><div class="value">{pose_delta.get("mean", 0.0):.4f} m</div></div>
  <div class="card"><h3>Max Pose Delta</h3><div class="value">{pose_delta.get("max", 0.0):.4f} m</div></div>
  <div class="card"><h3>Max Rotation Delta</h3><div class="value">{rot_delta.get("max", 0.0):.4f} deg</div></div>
  <div class="card"><h3>Outage ICP Priors</h3><div class="value">{metrics.get("outage_icp_valid_count", 0)}</div></div>
</div>
<section>
<h2>Refine Summary</h2>
<table>
{''.join(f'<tr><td>{html.escape(str(k))}</td><td>{html.escape(str(v))}</td></tr>' for k, v in summary.items())}
</table>
</section>
<section>
<h2>Metric Tables</h2>
{html_stat_table("Stage2 recomputed GPS residual (m)", metrics["gps_stage2_recomputed_residual_m"])}
{html_stat_table("Stage3 GPS residual (m)", metrics["gps_stage3_residual_m"])}
{html_stat_table("Pose translation delta (m)", metrics["pose_translation_delta_m"])}
{html_stat_table("Pose rotation delta (deg)", metrics["pose_rotation_delta_deg"])}
{html_stat_table("Outage ICP fitness", metrics["outage_icp_fitness"])}
</section>
{interactive_html}
{image_html}
</body>
</html>
"""
    path = output_dir / "stage3_eval_report.html"
    path.write_text(html_text)
    return path.name


def evaluate(args: argparse.Namespace) -> Path:
    stage3_dir = Path(args.stage3_dir).expanduser().resolve()
    output_dir = create_output_dir(Path(args.output_dir).expanduser().resolve(), args.timestamp)
    manifest = load_manifest(stage3_dir)
    utm_origin = manifest_utm_origin(manifest)

    keyframes = load_keyframes(stage3_dir)
    deltas = load_pose_delta(stage3_dir)
    priors = load_gps_priors(stage3_dir, utm_origin)
    blocks = load_outage_blocks(stage3_dir)
    refine_summary = load_single_csv(stage3_dir, "diagnostics/refine_summary.csv")
    if not keyframes:
        raise RuntimeError(f"No Stage3 keyframes found under {stage3_dir}")

    keyframes_by_id = add_path_distance(keyframes)
    enrich_gps_priors(priors, keyframes_by_id)
    metrics = compute_metrics(keyframes, deltas, priors, blocks, refine_summary)

    interactive_artifacts: Dict[str, str] = {}
    images: List[str] = []
    if not args.no_plots:
        images = generate_plots(output_dir, keyframes, priors, deltas, blocks, keyframes_by_id, args.max_plot_points)
        if HAS_PLOTLY:
            html_3d = write_3d_html(output_dir, keyframes, priors)
            if html_3d:
                interactive_artifacts["3d_refine"] = html_3d

    report_name = write_text_report(output_dir, stage3_dir, manifest, metrics, interactive_artifacts)
    stats_payload = {
        "stage3_dir": str(stage3_dir),
        "manifest": manifest,
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "utm_origin": utm_origin,
        "metrics": metrics,
        "plots": images,
        "interactive_plots": interactive_artifacts,
    }
    (output_dir / "stage3_eval_stats.json").write_text(json.dumps(stats_payload, indent=2, default=json_safe))
    write_html_report(output_dir, stage3_dir, manifest, metrics, images, report_name, interactive_artifacts)

    print(f"Stage3 eval written to: {output_dir}")
    print(f"Keyframes: {metrics['keyframe_count']}")
    print(f"GPS priors: {metrics['gps_prior_used_count']} used / {metrics['gps_prior_count']} total")
    print(f"Stage3 mean GPS residual: {metrics['gps_stage3_residual_m'].get('mean', math.nan):.4f} m")
    print(f"Mean pose delta: {metrics['pose_translation_delta_m'].get('mean', math.nan):.4f} m")
    if interactive_artifacts.get("3d_refine"):
        print(f"3D view: {output_dir / interactive_artifacts['3d_refine']}")
    return output_dir


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    run_config = read_top_level_config(script_dir.parent / "conf" / "current_vehicle.yaml")
    vehicle = run_config["active_vehicle"]
    default_stage3_dir = Path(run_config["data_root"]) / vehicle / "stage3_graph_refine"
    default_output_dir = Path(run_config["debug_root"]) / "stage3_refine" / vehicle / "stage3"
    parser = argparse.ArgumentParser(description="Evaluate and visualize air_mapping Stage3 refinement artifacts.")
    parser.add_argument("--stage3_dir", default=str(default_stage3_dir), help="Path to Stage3 output directory.")
    parser.add_argument("--output_dir", default=str(default_output_dir), help="Base output directory for reports.")
    parser.add_argument("--timestamp", action="store_true", help="Write to a timestamped subdirectory.")
    parser.add_argument("--no_timestamp", action="store_false", dest="timestamp", help=argparse.SUPPRESS)
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
