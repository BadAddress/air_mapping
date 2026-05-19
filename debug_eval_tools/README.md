# air_mapping Debug Evaluation Tools

This directory contains read-only evaluation tools for offline mapping artifacts.

## Stage2 Alignment Evaluation

`eval_stage2_alignment.py` reads the Stage 2 output directory and generates:

- `stage2_eval_report.html`
- `stage2_eval_report.txt`
- `stage2_eval_stats.json`
- `stage2_3d_comparison.html`
- trajectory, residual, GPS quality, height, lever-arm, and pose-delta plots

Default input:

```bash
modules/air_mapping/data/stage2_graph_opt
```

Run with defaults:

```bash
cd modules/air_mapping/debug_eval_tools
./run_eval_stage2.sh
```

By default, results overwrite `eval_results/stage2_alignment` so the browser can
keep using the same report paths. Add `--timestamp` only when you want to keep a
separate historical run.

Run against another Stage 2 output:

```bash
./run_eval_stage2.sh --stage2_dir /path/to/stage2_graph_opt
```

Useful direct options:

```bash
python3 eval_stage2_alignment.py \
  --stage2_dir ../data/stage2_graph_opt \
  --output_dir ./eval_results/stage2_alignment
```

Key metrics:

- Alignment residuals before and after Stage 2 global alignment
- GPS anchor count, segment count, anchor path gaps, and segment lengths
- Effective GPS-covered path length and coverage ratio
- Interactive 3D before/after trajectory view with GPS-covered sections highlighted
- GPS standard deviation versus residual consistency
- GPS height smoothing effect
- Lever-arm calibration selection, rejection reasons, residual reduction, and cost reduction
- Stage 2 pose delta from source LIO poses

The 3D comparison uses:

- red line: source LIO trajectory after the same initial translation baseline
- blue line: full Stage2 rigidly aligned trajectory
- green thick line: Stage2 trajectory sections effectively covered by GPS
- gray dashed line: Stage2 trajectory sections outside effective GPS coverage
- green/orange points: smoothed/raw GPS anchors

The tool evaluates the existing artifacts only. It does not rerun Stage 2 and
does not rewrite any mapping output.
