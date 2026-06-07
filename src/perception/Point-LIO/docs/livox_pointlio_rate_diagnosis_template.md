# Livox + Point-LIO 频率诊断记录

## 测试环境

| 项目 | 结果 |
|---|---|
| 日期 | 2026.6.8-00：22|
| 机器 | intel nuc 13 -i7 32GB|
| ROS2 版本 |humble |
| RMW | |
| CPU | |
| 雷达 | mid360-2个|
| xfer_format | custom|
| 点云 topic | /livox/lidar|
| IMU topic | /livox/imu_192_168_1_135|
| 是否 internal merge | 是|
| Point-LIO 参数文件 | mid360.yaml|
| Driver 参数文件 | mixed_MID360_component.yaml|

## 测试 A：driver + min_sub

| 项目 | 结果 |
|---|---|
| driver publish Hz | 10 |
| min_sub Hz | 1-9,中间频率居多 |
| ros2 topic hz | 稳定后达8.5hz，中间有一次长时间丢包后下降到7.5hz|
| CPU | 1.0-1.1ms|
| 结论 | |

## 测试 B：driver + 原跨进程 Point-LIO

| 项目 | 结果 |
|---|---|
| Point-LIO cloud_cb Hz | 1-10|
| Point-LIO sync_in Hz | 1-10 和cloud同步|
| Point-LIO avg/max process_ms | 6-10ms/7-9ms|
| 外部 min_sub Hz | 1-10|
| CPU | |
| 结论 | |

## 测试 C：component intra-process

| 项目 | 结果 |
|---|---|
| driver publish Hz | 10|
| Point-LIO cloud_cb Hz | 9.8-10|
| Point-LIO sync_in Hz | 9.8-10|
| Point-LIO avg/max process_ms | 6-7ms/6-11ms|
| 是否仍 2~7Hz 跳变 | 无|
| CPU | |
| 结论 | |

## 测试 D：component intra-process + 外部 subscriber 对照

| 项目 | 结果 |
|---|---|
| Point-LIO cloud_cb Hz | 10|
| Point-LIO sync_in Hz | 10|
| 外部 min_sub Hz | 0|
| 内外差异 | |
| 结论 | |

## 最终判断

- DDS / 跨进程是否是主因：
- Point-LIO 调度是否是主因：
- Point-LIO 计算是否是主因：
- QoS 是否是主因：
- 后续需要继续优化的点：
