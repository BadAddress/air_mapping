# air_mapping

Independent offline mapping module.

## Stage 1: LIO Record Processing

Stage 1 reads Cyber record files directly and feeds the copied LIO mapping stack
without registering a Cyber component or using publish/subscribe.

Edit `conf/stage1_lio.yaml` before running:

```yaml
stage1:
  records:
    - /path/to/record_directory
  output_dir: /apollo_workspace/modules/air_mapping/data/stage1_lio
```

Each `records` entry may be either a single record file or a directory
containing split record files. Directory entries are expanded and processed in
file-name order.

Build:

```bash
buildtool build -p modules/air_mapping/
```

Run:

```bash
/opt/apollo/neo/bin/stage1_lio \
  --config=/apollo_workspace/modules/air_mapping/conf/stage1_lio.yaml
```

Primary artifacts:

- `keyframes/keyframes.csv`
- `keyframes/poses_lio.tum`
- `keyframes/relative_edges.csv`
- `keyframes/clouds/*.pcd`
- `gps/gps_full.csv`
- `gps/gps_keyframe_assoc.csv`
- `preview/lio_global_preview.pcd`
