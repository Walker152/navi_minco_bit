# Livox + Point-LIO 频率诊断记录

## 测试环境

| 项目 | 结果 |
|---|---|
| 日期 | 2026.6.8-00：22 |
| 机器 | Intel NUC 13，i7，32GB |
| ROS2 版本 | Humble |
| RMW | 未记录；RMW 是 ROS 2 的中间件实现层，实机可用 `echo $RMW_IMPLEMENTATION` 查看；若为空，Humble 通常使用默认 Fast DDS，建议再用 `ros2 doctor --report | grep -i rmw` 确认 |
| CPU | Intel i7（Intel NUC 13，32GB）；具体型号建议实机补充：`lscpu | grep 'Model name'`；测试时整机/线程占用仍建议用 `htop` 或 `top -H` 记录 |
| 雷达 | MID360 × 2 |
| xfer_format | custom |
| 点云 topic | /livox/lidar |
| IMU topic | /livox/imu_192_168_1_135 |
| 是否 internal merge | 是 |
| Point-LIO 参数文件 | mid360.yaml |
| Driver 参数文件 | mixed_MID360_component.yaml |

## 测试 A：driver + min_sub

| 项目 | 结果 |
|---|---|
| driver publish Hz | 10 |
| min_sub Hz | 1–9，中间频率居多 |
| ros2 topic hz | 稳定后达 8.5Hz，中间有一次长时间丢包后下降到 7.5Hz |
| CPU | merge CPU time 约 1.0–1.1ms；整机 CPU 占用未记录 |
| 结论 | 只启动 driver 与外部最小订阅器时，driver 侧稳定 10Hz，但跨进程订阅侧仍无法稳定达到 10Hz，说明普通跨进程 DDS / RMW 接收链路对双 MID360 合并后的大点云存在明显抖动或丢帧。`ros2 topic hz` 能到 7.5–8.5Hz，但仍低于 driver 10Hz，因此它不是唯一测量误差，外部 C++ min_sub 也体现了跨进程链路不稳。|

## 测试 B：driver + 原跨进程 Point-LIO

| 项目 | 结果 |
|---|---|
| Point-LIO cloud_cb Hz | 1–10 |
| Point-LIO sync_in Hz | 1–10，和 cloud_cb 同步 |
| Point-LIO avg/max process_ms | 6–10ms / 7–9ms |
| 外部 min_sub Hz | 1–10 |
| CPU | 未记录整机 CPU 占用；从 process_ms 看，Point-LIO 单帧主处理时间明显小于 100ms |
| 结论 | 原跨进程模式下，Point-LIO 的 `cloud_cb` 和 `sync_in` 同步跳变，说明点云进入 callback 的频率本身已经不稳，而不是 `sync_packages` 额外大量丢帧。Point-LIO 单帧处理约 6–10ms，理论上足够支撑 10Hz，因此当前主要瓶颈不在 LIO 主计算耗时，而更像是跨进程 DDS 传输 / 订阅调度 / QoS 丢帧造成的输入频率抖动。|

## 测试 C：component intra-process

| 项目 | 结果 |
|---|---|
| driver publish Hz | 10 |
| Point-LIO cloud_cb Hz | 9.8–10 |
| Point-LIO sync_in Hz | 9.8–10 |
| Point-LIO avg/max process_ms | 6–7ms / 6–11ms |
| 是否仍 2~7Hz 跳变 | 无 |
| CPU | 未记录整机 CPU 占用；从 process_ms 看，LIO 处理余量充足 |
| 结论 | component + intra-process 后，Point-LIO 的 callback 输入频率和 `sync_in` 均恢复到接近 driver 发布频率，且不再出现 2–7Hz 跳变。这是最关键对照结果，说明同进程 intra-process 改造有效，原问题的主因基本可以锁定在跨进程大点云传输 / DDS 订阅链路，而不是 driver 合并、Point-LIO 同步逻辑或 LIO 主计算。|

## 测试 D：component intra-process + 外部 subscriber 对照

| 项目 | 结果 |
|---|---|
| Point-LIO cloud_cb Hz | 10 |
| Point-LIO sync_in Hz | 10 |
| 外部 min_sub Hz | 0 |
| 内外差异 | Point-LIO 与 driver 在同一 component container 内通过 intra-process 通讯稳定 10Hz；外部 min_sub 属于另一个进程，仍依赖 inter-process DDS 链路。本次外部 min_sub 为 0，说明外部订阅链路没有收到数据或没有匹配到正确 topic / QoS / 类型，但这不影响“Point-LIO 内部同进程已稳定 10Hz”的判断。需要单独检查外部订阅器的 topic、msg_type、QoS、namespace、remap 与是否仍存在 inter-process publisher。 |
| 结论 | component 内部链路已经稳定，外部订阅器结果不能代表 Point-LIO 内部接收频率。外部 min_sub 为 0 更像是外部测试配置或 intra-process/inter-process 发布可见性问题，需要另行排查，但不推翻 component intra-process 对 Point-LIO 有效的结论。|

## 最终判断

- DDS / 跨进程是否是主因：是。测试 A 中 driver 10Hz 但跨进程 min_sub / topic hz 均低于 10Hz；测试 B 中 Point-LIO 的 cloud_cb 与 sync_in 同步跳变；测试 C 中 component intra-process 后 cloud_cb 与 sync_in 恢复 9.8–10Hz。三组对照共同说明主因是跨进程大点云传输 / DDS 接收链路不稳定。
- Point-LIO 调度是否是主因：不是主要矛盾。原跨进程模式下 cloud_cb 本身已经跳变，而 component 后 cloud_cb 与 sync_in 同时恢复稳定，说明改造后的 executor / worker 结构有效；但从结果看，决定性改善来自绕过跨进程链路。若后续长时间运行仍偶发掉频，再继续检查 callback group、worker thread、锁竞争和 CPU 亲和性。
- Point-LIO 计算是否是主因：基本不是。测试 B 中主处理约 6–10ms，测试 C 中约 6–7ms / 最大 11ms，均明显小于 100ms 的 10Hz 周期；component 后稳定 10Hz 也证明 LIO 计算量具备实时余量。
- QoS 是否是主因：不是唯一主因，更像是跨进程链路不稳时的表现放大因素。SensorDataQoS / best effort 可能导致大点云在订阅端处理不及时或 DDS 抖动时直接丢帧，但 component intra-process 后同样能稳定 10Hz，说明根本问题不应只靠改 reliable QoS 解决。Reliable 可能把丢帧变成延迟堆积，不建议作为实时导航的主要修复方案。
- 后续需要继续优化的点：
  1. 保留 driver + Point-LIO 同一 component container + intra-process 作为实机默认启动方式。
  2. 确认 Point-LIO 点云订阅优先使用 `UniquePtr` 或至少 `ConstSharedPtr`，避免为了适配又复制整帧点云。
  3. 检查外部 min_sub 为 0 的原因：topic 名、namespace、remap、`msg_type=custom` 是否匹配、QoS 是否匹配、component launch 是否仍允许 inter-process 外部订阅。
  4. 补充记录 RMW、Intel i7 的具体型号、测试时整机 CPU 占用和线程占用，便于后续复现。
  5. 长时间运行测试 5–10 分钟，确认 component 模式下 cloud_cb / sync_in 持续稳定 9.8–10Hz。
  6. 若仍需要 RViz / rosbag / 外部调试订阅，建议只订阅降频或降采样后的可视化点云，避免重新制造大点云跨进程压力。
  7. 保留原跨进程 launch 作为回滚方案，但 RM 实机场景优先使用 component intra-process。