# air_mapping

Independent offline mapping module.

## 配置体系

air_mapping 现在使用单一顶层入口：

- `conf/current_vehicle.yaml` 只负责选择当前车型和统一根路径。
- `conf/vehicles/es6.yaml`、`conf/vehicles/minibus.yaml` 负责该车型的数据包路径、通道、外参和 Stage 参数。
- 所有 stage 二进制默认直接读取 `conf/current_vehicle.yaml`，不需要你在命令行后面再加 `--config`。
- 同一车型重跑时直接覆盖 `data/<vehicle>/...` 和 `data/debug/<analysis>/<vehicle>/...`。
- 不同车型只会落在平行子目录下，结构保持一致。
- Docker 内默认模块根路径是 `/apollo_workspace/modules/air_mapping`；配置中的 `/media/x/**` 外接盘路径保持原样。

切换车型只改：

```yaml
air_mapping:
  active_vehicle: "minibus"
  active_vehicle_config: "conf/vehicles/minibus.yaml"
```

统一产物路径：

- `data/<vehicle>/stage1_lio`
- `data/<vehicle>/stage2_graph_opt`
- `data/<vehicle>/stage3_graph_refine`
- `data/debug/<analysis>/<vehicle>/<stage>`

默认入口：

```bash
/opt/apollo/neo/bin/stage1_lio
/opt/apollo/neo/bin/stage2_graph_opt
/opt/apollo/neo/bin/stage3_graph_refine
```

## Stage 1: Pure LIO + LiDAR-Only OPT

Stage 1 reads Cyber record files directly and feeds the copied LIO mapping stack
without registering a Cyber component or using publish/subscribe. After the LIO
front-end finishes, Stage 1 can run LiDAR-only loop detection and pose graph
optimization using LIO relative edges plus NDT-verified loop edges. GPS is only
recorded for later stages; it is not fused in Stage 1.

Edit current vehicle profile before running:

```yaml
dataset:
  records: /path/to/record_directory
```

Each `records` entry may be either a single record file or a directory
containing split record files. Directory entries are expanded and processed in
file-name order.

车型配置说明：

- `profile.vehicle_name` 决定产物子目录名，顶层配置中的 `active_vehicle` 是默认回退值。
- `dataset.records` 是当前车型的数据包入口，支持单文件或目录。
- `channels.*` 是 IMU、GPS、heading、主 LiDAR topic。
- `dual_lidar_fusion.enable=true` 时，Stage 1 会启用离线双雷达同步融合。
- `dual_lidar.channels`、`dual_lidar.sync`、`dual_lidar.derived` 定义双雷达 topic、同步阈值和外参；副雷达会先变换到主雷达坐标系，再送入原 LIO 前端。
- `zleveling.enable=true` 时，Stage 1 会在第一轮 SE3 回环优化结果上做第二轮完整 SE3 图优化，并按 Lightning-LM `with_height` 思路给每个 keyframe 加固定高度先验；`es6` 默认关闭，`minibus` 默认开启。
- 顶层模式下这些派生路径只在运行时注入，车辆 profile 不再手写 `stage1/stage2/stage3` 的产物路径。

Build:

```bash
buildtool build -p modules/air_mapping/
```

Run:

```bash
/opt/apollo/neo/bin/stage1_lio
```

Primary artifacts:

- `keyframes/keyframes.csv`
- `keyframes/poses_lio.tum` (optimized pose, compatibility name)
- `keyframes/poses_lio_raw.tum`
- `keyframes/poses_lio_opt.tum`
- `keyframes/relative_edges.csv` (optimized adjacent pose deltas)
- `keyframes/relative_edges_lio_raw.csv`
- `keyframes/loop_edges.csv`
- `keyframes/clouds/*.pcd`
- `gps/gps_full.csv`
- `gps/gps_keyframe_assoc.csv`
- `gps/gps_keyframe_raw_assoc.csv`
- `utm_alignment.txt`
- `diagnostics/loop_summary.csv`
- `diagnostics/zleveling_summary.csv`
- `diagnostics/zleveling_samples.csv`
- `diagnostics/keyframe_diagnostics.csv`
- `preview/lio_global_preview.pcd`
- `preview/lio_opt_global_preview.pcd`

### Stage 1 记录表说明

Stage 1 的原则是保留 LIO 前端结果和 GPS 原始可复现观测，不在本阶段融合 GPS
位置。后续 Stage 2/Stage 3 可以基于这些表重新构建 lever arm、heading offset、
GPS anchor 筛选、outage/support 一致性等优化模型。

`zleveling` 是 Stage 1 的可选第二轮 SE3 图优化，不使用 GPS 约束：

- 第一轮 Stage 1 回环仍然是完整 SE3 优化，用来闭合轨迹。
- 回环边优化后会按 `loop_outlier_chi2_threshold` 做一次残差外点剔除，剔除后重新优化，避免少量错回环把局部结构拉出重影。
- 第二轮以第一轮结果为初值，继续使用相邻 LIO SE3 边和第一轮检测到的回环边。
- 第二轮额外给每个 keyframe 加 `EdgeHeightPrior`，measurement 固定为 `0m`，默认 `height_noise_m=0.05`，适合基本无坡度的道路场景压制 Z 卷曲漂移。
- `diagnostics/zleveling_summary.csv` 记录是否启用、是否应用、高度先验边数量、最大绝对高度变化等信息。
- `diagnostics/zleveling_samples.csv` 当前保留为空表，用于兼容诊断产物结构。

`gps/gps_full.csv` 记录每一组配对成功的 GNSS BestPose + Heading：

- `antenna_x/y/z` 是蘑菇头天线 UTM 坐标，未做杆臂补偿。
- `imu_x/y/z` 是在线流程用 GNSS heading 和当前配置杆臂补偿出的 IMU UTM 坐标。
- `heading_rad`、`pitch_rad` 是 GNSS 原始姿态观测，`heading_std_deg`、`pitch_std_deg` 是其质量。
- `std_x/y/z`、`sol_status`、`sol_type`、`satellite_tracked` 主要用于后续诊断与质量分析，其中当前筛选逻辑统一不再依赖 `satellite_tracked`。

`gps/gps_keyframe_assoc.csv` 是兼容旧流程的稀疏 keyframe GPS anchor：

- `utm_x/y/z` 是 Stage 1 当时附着到 keyframe 的 IMU UTM 坐标。
- 当前实现中该坐标由 `antenna_utm - Rz(LIO_yaw_raw) * configured_lever_arm` 得到。
- 这个文件适合快速使用，但不适合作为唯一研究依据，因为它已经包含一次杆臂和 yaw 选择。

`gps/gps_keyframe_raw_assoc.csv` 是后续优化建模的主表：

- 每个 keyframe 都尝试从 `gps_full.csv` 插值原始天线观测，`has_raw_gps` 表示是否成功。
- `interp_before_time`、`interp_after_time`、`interp_alpha`、`interp_gap_before_s`、`interp_gap_after_s` 描述插值质量，可用于发现时间同步或 GPS outage 问题。
- `antenna_utm_*` 保留原始天线 UTM，后续 lever arm 标定应优先使用这些字段。
- `gps_heading_imu_utm_*` 使用 GNSS heading 补偿杆臂，`lio_raw_yaw_imu_utm_*` 使用原始 LIO yaw 补偿，`lio_opt_yaw_imu_utm_*` 使用 loop 后 LIO yaw 补偿。
- `lever_arm_config_*` 是本次 Stage 1 配置里的初始杆臂，`lever_arm_*_utm_*` 是不同 yaw 模型旋转到 UTM 后的杆臂。
- `lio_raw_*` 和 `lio_opt_*` 同时记录 keyframe 原始 LIO pose 与 loop 后 pose，便于比较 LIO 一致性和 UTM 一致性。
- `stage1_minus_lio_raw_yaw_*`、`stage1_minus_gps_heading_*` 用于快速检查 Stage 1 anchor 和两种复现模型是否一致。

`diagnostics/keyframe_diagnostics.csv` 记录每个 keyframe 的 LIO 质量摘要：

- `cloud_point_count` 是该 keyframe 点云点数，可用于排查稀疏帧或退化帧。
- `lio_raw_*` 是前端原始 LIO pose，`lio_opt_*` 是 Stage 1 LiDAR-only loop 优化后的 pose。
- `delta_*` 和 `delta_rotation_deg` 描述 loop 优化对该帧的改变量。
- `cov_xx/yy/zz`、`cov_rollroll/pitchpitch/yawyaw` 是保存的 6x6 位姿协方差对角线。
- `relative_*` 是到前一 keyframe 的 LIO 相对运动边，后续图优化可与 `keyframes/relative_edges_lio_raw.csv` 交叉验证。

## Stage 2: GPS-Aided Rigid Alignment

Stage 2 reads Stage 1 artifacts and produces a global UTM-local alignment for
the LIO keyframes. The pipeline is:

1. Load Stage 1 keyframes, relative edges, GPS anchor associations, raw GPS
   history, and covariance files.
2. Run a pre-alignment lever-arm calibration on the raw antenna observations.
3. Rebuild GPS anchors with the optimized lever arm.
4. Solve the final global alignment as one robust full SE3 rigid transform, so
   GPS-supported sections align to their corresponding LIO trajectory while the
   same rigid transform carries that correction through GPS outage sections.

GPS anchors are split into continuous segments; adjacent valid GPS anchors
farther than 30 m in XY start a new segment. GPS height is smoothed inside each
segment using the recorded `std_z`. The final per-anchor GPS prior weight uses
the recorded standard deviation directly, and the current configuration keeps
only very high precision RTK fixed samples for lever-arm calibration.

The lever-arm calibration uses `gps/gps_keyframe_raw_assoc.csv` and estimates a
better antenna-to-IMU lever arm before the final graph solve. It also estimates
an optional global heading bias, so the optimized lever arm does not absorb all
heading mismatch by itself. This makes the Stage 1 / Stage 2 interface clearer
and keeps Stage 3 from inheriting a biased GPS frame.

Stage 2 reads the current vehicle profile automatically. Run:

```bash
/opt/apollo/neo/bin/stage2_graph_opt
```

顶层配置模式下 Stage 2 输入固定为 `data/<vehicle>/stage1_lio`，输出固定为
`data/<vehicle>/stage2_graph_opt`；`source_config_path` 会写入当前车辆 yaml，供后续
Stage 3 读取 LiDAR 外参。

Primary artifacts:

- `diagnostics/lever_arm_calibration.csv`
- `diagnostics/lever_arm_samples.csv`
- `alignment/se3_lio_to_utm_local.txt`
- `alignment/utm_origin.txt`
- `keyframes/poses_opt.tum`
- `keyframes/poses_opt_utm.tum`
- `keyframes/keyframes_opt.csv`
- `keyframes/relative_edges.csv`
- `keyframes/pose_delta.csv`
- `diagnostics/alignment_anchors.csv`
- `diagnostics/alignment_summary.csv`
- `diagnostics/gps_segments.csv`
- `preview/optimized_global_preview.pcd`

### Stage 2 输入

Stage 2 主要消费以下 Stage 1 产物：

- `keyframes/keyframes.csv`：keyframe 的原始/优化后 LIO 位姿、云路径、协方差信息。
- `keyframes/relative_edges.csv`：Stage 1 生成的相邻关键帧约束，供 Stage 2 继承位姿图骨架。
- `gps/gps_keyframe_assoc.csv`：稀疏的旧式 GPS anchor，作为兼容和回退输入。
- `gps/gps_keyframe_raw_assoc.csv`：Stage 2 杆臂优化的主输入，包含 raw antenna、LIO raw/opt yaw、GNSS heading、std_dev、sol_status、sol_type、satellite_tracked。
- `gps/gps_full.csv`：用于按关键帧时间插值原始 GPS 天线/heading 观测。
- `keyframes/covariance/*.txt`：关键帧协方差，供后续维护/诊断参考。
- `keyframes/clouds/*.pcd`：可选预览地图构建输入。

### Stage 2 输出

Stage 2 输出的是一组可追踪、可回放的中间结果，而不是只输出最终地图：

- `diagnostics/lever_arm_calibration.csv`：杆臂优化汇总。
- `diagnostics/lever_arm_samples.csv`：逐样本诊断。
- `diagnostics/alignment_anchors.csv`：最终参与全局对齐的 GPS anchor。
- `diagnostics/gps_segments.csv`：GPS 连续段摘要。
- `alignment/se3_lio_to_utm_local.txt`：最终 LIO 到 UTM-local 的 4x4 变换矩阵。
- `alignment/utm_origin.txt`：UTM-local 原点。
- `keyframes/poses_opt.tum`：Stage 2 优化后的本地轨迹。
- `keyframes/poses_opt_utm.tum`：Stage 2 优化后的绝对 UTM 轨迹。
- `keyframes/keyframes_opt.csv`：每个 keyframe 的原始/优化后位姿与来源路径。
- `keyframes/relative_edges.csv`：Stage 2 重写后的相邻相对边。
- `keyframes/pose_delta.csv`：LIO 到优化结果的位姿变化。
- `preview/optimized_global_preview.pcd`：可选全局预览点云。

### 优化结果说明

`diagnostics/lever_arm_calibration.csv` 记录本阶段杆臂标定的汇总结果：

- `orientation_model` 标记本次标定使用的是 `lio_yaw_only` 还是 `full_lio`。
- `initial_*` 是 Stage 1 配置中的初始杆臂。
- `correction_*` 是标定得到的修正量。
- `optimized_*` 是最终用于重建 GPS anchor 的杆臂。
- `heading_bias_deg` 是联合估计得到的全局 heading 偏差。
- `weighted_cost_initial` / `weighted_cost_optimized` 用于比较鲁棒加权目标是否下降。
- `mean/max_residual_*` 用于比较标定前后 antenna residual 是否下降。

`diagnostics/lever_arm_samples.csv` 记录逐样本标定诊断：

- `selected` 表示该样本是否进入杆臂优化。
- `reject_reason` 说明样本被排除的原因，例如 `std_xy_too_large` 或 `interp_gap_too_large`。
- `sol_status`、`sol_type`、`satellite_tracked`、`heading_std_deg`、`gnss_lio_yaw_diff_deg` 用于解释样本质量，但当前有效样本门控不再使用 `satellite_tracked`。
- `pred_initial_*` 和 `pred_optimized_*` 是标定前后预测的天线位置。
- `residual_initial_*` 和 `residual_optimized_*` 用于定位是 lever arm、heading 还是时间同步在拉坏结果。
- `lio_raw_yaw_rad`、`lio_opt_yaw_rad`、`gnss_heading_rad`、`gnss_pitch_rad`、
  `heading_std_deg`、`pitch_std_deg` 用于对比三种方向来源的一致性。

`diagnostics/alignment_anchors.csv` 记录最终进入全局对齐的 anchor：

- `gps_utm_*` 是原始 GPS 天线位置。
- `gps_smooth_utm_*` 是每个连续段内经过 Z 平滑后的 GPS 位置。
- `weight` 是按 `std_dev` 计算的对齐权重。
- `residual_before_m` / `residual_after_m` 用于查看全局对齐前后误差。

`diagnostics/gps_segments.csv` 记录 GPS 连续段：

- `segment_id` 是段编号。
- `anchor_count` 是该段的 anchor 数量。
- `length_xy_m` 是该段 XY 轨迹长度。
- `mean_abs_z_smoothing_delta_m` / `max_abs_z_smoothing_delta_m` 表示 Z 平滑幅度。

`diagnostics/alignment_summary.csv` 记录最终全局对齐摘要：

- `alignment_model` 是 `yaw_only_translation` 或 `se3`。
- `anchor_count`、`segment_count` 表示本次参与对齐的规模。
- `mean/max_residual_before_m` 与 `mean/max_residual_after_m` 用于比较对齐前后误差。
- `roll_deg/pitch_deg/yaw_deg` 是最终估计出的全局刚体姿态角。

`alignment/se3_lio_to_utm_local.txt` 和 `alignment/utm_origin.txt` 是 Stage 2 的最终对齐结果：

- 默认模型是 `se3`，即用 GPS 有效区域和对应 LIO 轨迹估计一个全局刚体变换。
- 该变换会应用到全部 keyframe，所以 GPS 有效区域估计出的全局姿态会刚性传递到 GPS 信号不好的区域。
- 若需要回退到只估计 yaw 和平移，可显式设置 `constrain_to_yaw_only=true`。
- `utm_origin` 是 UTM-local 坐标系的零点，后续 Stage 3 直接继承它。

## Stage 3: GPS Prior Graph Refinement

Stage 3 reads Stage 2 artifacts and runs an offline pose graph refinement in
the Stage 2 UTM-local frame. It initializes every vertex from Stage 2 poses,
keeps adjacent relative pose edges from `keyframes/relative_edges.csv`, and adds
GPS position priors from `diagnostics/alignment_anchors.csv`. GPS priors are
weighted from their recorded `std_x/std_y/std_z` and ramped down near GPS outage
boundaries. By default Stage 3 ignores GPS Z and only constrains XY, because Z
often over-pulls the graph in outage-heavy sections. Stage 3 also fuses only
very high precision GPS anchors by default (`std_x/std_y <= 0.03 m`).
GPS-supported regions are reconstructed from Stage 2 GPS segments, but
same-segment anchor gaps longer than `gps_support_max_anchor_gap_m` remain
candidate outages. Long no-GPS regions are collapsed into rigid outage blocks;
each block is ICP-matched against nearby GPS-supported support submaps and
optimized through one representative SE3 prior, preventing rubber-band
deformation inside the outage.

Stage 3 reads the current vehicle profile automatically. Run:

```bash
/opt/apollo/neo/bin/stage3_graph_refine
```

顶层配置模式下 Stage 3 输入固定为 `data/<vehicle>/stage2_graph_opt`，输出固定为
`data/<vehicle>/stage3_graph_refine`。

Primary artifacts:

- `keyframes/poses_refined.tum`
- `keyframes/poses_refined_utm.tum`
- `keyframes/keyframes_refined.csv`
- `keyframes/relative_edges.csv`
- `keyframes/pose_delta.csv`
- `diagnostics/refine_summary.csv`
- `diagnostics/gps_priors.csv`
- `diagnostics/outage_blocks.csv`
- `preview/refined_global_preview.pcd`

## Viz: Static Mapping Visualizer

`viz` 是从 `air_viz` 拷贝核心前端渲染和 PCD/HDMap 解析逻辑后裁剪出的独立工具，不依赖
`modules/air_viz`。它只显示静态内容，不订阅 Cyber topic。

构建目标：

```bash
./apollo.sh build //modules/air_mapping/viz:air_mapping_viz
```

启动：

```bash
modules/air_mapping/scripts/viz.sh 12322
```

`viz.sh` 默认读取 `conf/current_vehicle.yaml`，因此文件列表和默认 PCD/alignment 都会随当前车型切换。

打开：

```text
http://localhost:12322
```

前端文件选择只暴露 `modules/air_mapping` 下的相对路径，后端会拒绝越界路径。HDMap
本地默认放置目录是 `viz/hdmap_local/`，仓库自带示例地图仍放在 `viz/hdmap/`。
如果你有长期使用的本地高精地图，建议放到 `viz/hdmap_local/`，这样切换分支不会影响它。

模式说明：

- `LOCAL`：只选择并加载一个 PCD，直接按 PCD 内坐标显示。
- `GLOBAL`：选择 HDMap、PCD 和 `utm_alignment.txt`；PCD 默认已经与 UTM 方向对齐，直接显示。
- `GLOBAL` 下 HDMap 使用 `p_viz = p_hdmap_utm - offset` 转到 PCD 的局部坐标系，只做平移 offset，不额外应用 yaw。
- `utm_alignment.txt` 支持 `offset_x/offset_y/offset_z` YAML-like 格式，也兼容一行 `x y z [yaw]` 格式；`yaw` 字段当前只解析兼容，不参与显示变换。
- 浏览器会记住上次选择的 `LOCAL/GLOBAL` 模式、PCD/HDMap/对齐文件路径和配色模式，下次打开会自动恢复并尝试直接加载。

视角控制：

- 使用 Three.js `PerspectiveCamera`，显示坐标系是 PCD/HDMap 都已经换算后的 local frame。
- 使用自定义 `PclViewerControls` 复刻 VTK/PCLVisualizer 的三键鼠标模型，不再依赖 `OrbitControls`。
- 左键旋转，`Ctrl+左键` 滚转，中键或 `Shift+左键` 平移，右键、滚轮或 `Ctrl+Shift+左键` 缩放。
- 双击点云、HDMap 线或地面，会把当前旋转/缩放中心切到双击位置，便于查看长条道路局部。
- `重置视角` 会重新按当前加载内容整体 fit，不改变数据坐标。
