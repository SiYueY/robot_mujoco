# robot_mujoco 性能基线

相关文档：

- 当前模块边界与线程模型见 [`current_architecture.md`](./current_architecture.md)
- 下游迁移入口见 [`migration_guide.md`](./migration_guide.md)
- 固定 JSON 样例见 [`performance_baseline.sample.json`](./performance_baseline.sample.json)

## 1. 目标

本基线用于阶段 11 的 PR/夜间回归对比，固定覆盖以下四类指标：

- scheduler 1 kHz step 吞吐、实时因子、step jitter
- headless camera sample 延迟与采样频率
- `robot_mujoco_ros2` read/write 主循环耗时
- 长时运行 RSS 曲线

## 2. 固定入口

### 2.1 基线场景定义

- [`mujoco_simulation/test/performance/baseline_scenarios.json`](../mujoco_simulation/test/performance/baseline_scenarios.json)
- [`mujoco_simulation/test/performance/perf_runtime_scenarios.cpp`](../mujoco_simulation/test/performance/perf_runtime_scenarios.cpp)
- [`robot_mujoco_ros2/tests/performance/perf_read_write_loop.cpp`](../robot_mujoco_ros2/tests/performance/perf_read_write_loop.cpp)

### 2.2 统一采集脚本

```bash
python3 mujoco_simulation/test/performance/run_baseline.py \
  --workspace-root /path/to/robot_mujoco \
  --output build/performance_baseline.json \
  --repeat 3 \
  --soak-seconds 60
```

Nightly 版本将 `--soak-seconds` 提升到 `3600`。

如需对当前结果执行阈值判定，可追加：

```bash
python3 scripts/ci/compare_performance_baseline.py \
  --baseline docs/performance_baseline.sample.json \
  --candidate build/performance_baseline.json \
  --output build/performance_comparison.json \
  --markdown-output build/performance_comparison.md
```

`run_soak_and_perf.sh` 在设置 `BASELINE_REFERENCE` 后会默认同时生成这两份产物。

### 2.3 固定样例产物

- repo 内受控样例：[`performance_baseline.sample.json`](./performance_baseline.sample.json)
- 当前样例来源：`2026-06-17` 在本地工作区执行
  `python3 mujoco_simulation/test/performance/run_baseline.py --workspace-root "$(pwd)" --output build/performance_baseline.sample.json --repeat 1 --soak-seconds 5`
- 样例中的 `generated_at_epoch_seconds` 与 `workspace_root` 已做脱敏处理，用于固定格式和字段约定；其余指标值来自实际运行结果

## 3. 指标口径

### 3.1 `scheduler_1khz`

- `steps`
- `wall_seconds`
- `sim_seconds`
- `throughput_hz`
- `realtime_factor`
- `jitter_ms_mean`
- `jitter_ms_p95`

说明：

- 模型 timestep 固定为 `0.001`
- 指标来自 `Simulation::start()` 后的后台调度线程
- jitter 目前按“相邻观测窗口的平均单步 wall time”统计

### 3.2 `headless_camera`

- `samples`
- `sample_frequency_hz`
- `sample_latency_ms_mean`
- `sample_latency_ms_p95`

说明：

- 场景固定为 headless RGB-D 相机
- 相机配置固定为 `160x120`、`50 Hz`
- latency 统计的是“单步推进到新 sample 可读”的 wall 延迟

### 3.3 `hardware_read_write_loop`

- `write_ms_mean`
- `write_ms_p95`
- `read_ms_mean`
- `read_ms_p95`
- `loop_ms_mean`
- `loop_ms_p95`

说明：

- 使用 `robot_mujoco_ros2::MuJoCoHardwareInterface` 真正走 `on_init -> activate -> write/read`
- 包含 `robot_mujoco_ros2` 内部 publisher/service bridge 接线开销

### 3.4 Soak / RSS

- `iterations`
- `wall_seconds`
- `rss_kb_first`
- `rss_kb_min`
- `rss_kb_max`
- `rss_kb_last`
- `rss_kb_samples`
- `rss_kb_series`
- `rss_kb_stable_samples`
- `rss_kb_head_median`
- `rss_kb_tail_median`
- `rss_kb_growth_kb`
- `rss_kb_growth_ratio`

说明：

- 当前 soak 通过单进程运行 `perf_read_write_loop --soak-seconds <N>` 持续采样 RSS
- 启动瞬态样本会保留在 `rss_kb_series` 中，但 `rss_kb_head_median` / `rss_kb_tail_median` / `rss_kb_growth_*` 会先剥离明显低于稳态的平台期前缀，再计算稳态段头尾窗口中位数
- 夜间作业默认跑 `3600 s`

## 4. 结果格式

标准输出产物固定为 JSON，核心结构：

```json
{
  "generated_at_epoch_seconds": 0,
  "repeat": 3,
  "scenarios": [
    {
      "name": "scheduler_1khz",
      "runs": [],
      "aggregated": {}
    }
  ],
  "soak": {
    "command": [],
    "duration_seconds": 3600,
    "result": {}
  }
}
```

比较脚本额外生成两类 nightly artifact：

- `performance_comparison.json`
  - 保留逐项 `checks`
  - 新增 `baseline` / `candidate` 元数据
  - 新增按场景聚合的 `scenarios`
  - 新增 `gating_checks`、`trend_checks` 和 `highlights`
- `performance_comparison.md`
  - 面向 nightly artifact 直接阅读的摘要
  - 固定包含 gating 表、trend 表、场景汇总和最大回退/提升项
- `nightly_summary.json` / `nightly_summary.md`
  - 由 nightly workflow 汇总 `performance_comparison.*` 与 `tsan_summary.*`
  - 用于快速判断整次夜跑的总体健康状态，而不必分别打开两个 job artifact
- `nightly_metrics.json`
  - 从 `nightly_summary` 中进一步抽取扁平化关键指标
  - 用于后续按 run 聚合成趋势历史，而不必重新解析完整 comparison / summary JSON
- `nightly_trend.json` / `nightly_trend.md`
  - 对比当前 `nightly_metrics.json` 与上一轮成功 nightly 的 snapshot
  - 若上一轮 artifact 不可用，则以 `missing_previous` 状态退化输出，不影响当前 nightly 继续产出 summary
  - 上一轮 artifact 获取由 repo 内脚本直接调用 GitHub Actions API，不依赖 runner 预装 `gh`
  - `nightly.yml` 已显式声明 `actions: read` / `contents: read`，避免依赖仓库默认 token 权限
- `nightly_history.json` / `nightly_history.md`
  - 汇总当前 run 与最近若干次成功 nightly 的 `nightly_metrics.json`
  - 关注持续 warning、窗口内最大回退/改善项，作为滚动趋势摘要

对应脚本测试：

- `scripts/ci/tests/test_nightly_ci_scripts.py`
  - 覆盖 nightly artifact 拉取、summary 状态判定、trend/history 核心逻辑
  - 已接入 `ci.yml` 和 `nightly.yml`，作为脚本级回归保护

## 5. 回归阈值

- scheduler 吞吐或 realtime factor 相对基线回退不得超过 5%
- camera sample 频率或 latency 相对基线回退不得超过 10%
- soak 期间不允许出现崩溃、死锁或持续性 RSS 上升趋势

当前脚本状态：

- `compare_performance_baseline.py` 已将 `scheduler_1khz.throughput_hz_mean`、`scheduler_1khz.realtime_factor_mean`、`headless_camera.sample_frequency_hz_mean` 作为 gating 指标
- `headless_camera` latency、`hardware_read_write_loop` 各项耗时和 soak RSS 目前先进入 `trend_checks`，其中 soak RSS 依据稳态段 `rss_kb_growth_ratio` 和 `rss_kb_growth_kb` 追踪；等 CI/nightly 基线积累稳定后再升级为硬阈值 gate
- nightly artifact 现在同时保留机器可读 JSON 和便于直接查看的 Markdown summary，后续累计真实 nightly 结果时无需再改比较协议

## 6. 本地执行顺序

```bash
source ../install/setup.bash
source install/setup.bash
colcon build --packages-select mujoco_simulation robot_mujoco_ros2
python3 mujoco_simulation/test/performance/run_baseline.py \
  --workspace-root "$(pwd)" \
  --output build/performance_baseline.json
```

如果只验证样例格式是否仍可生成，可执行：

```bash
source ../install/setup.bash
source install/setup.bash
python3 mujoco_simulation/test/performance/run_baseline.py \
  --workspace-root "$(pwd)" \
  --output build/performance_baseline.sample.json \
  --repeat 1 \
  --soak-seconds 5
```
