# Livox + Point-LIO 频率诊断记录

## 测试环境

| 项目 | 结果 |
|---|---|
| 日期 | |
| 机器 | |
| ROS2 版本 | |
| RMW | |
| CPU | |
| 雷达 | |
| xfer_format | |
| 点云 topic | |
| IMU topic | |
| 是否 internal merge | |
| Point-LIO 参数文件 | |
| Driver 参数文件 | |

## 测试 A：driver + min_sub

| 项目 | 结果 |
|---|---|
| driver publish Hz | |
| min_sub Hz | |
| ros2 topic hz | |
| CPU | |
| 结论 | |

## 测试 B：driver + 原跨进程 Point-LIO

| 项目 | 结果 |
|---|---|
| Point-LIO cloud_cb Hz | |
| Point-LIO sync_in Hz | |
| Point-LIO avg/max process_ms | |
| 外部 min_sub Hz | |
| CPU | |
| 结论 | |

## 测试 C：component intra-process

| 项目 | 结果 |
|---|---|
| driver publish Hz | |
| Point-LIO cloud_cb Hz | |
| Point-LIO sync_in Hz | |
| Point-LIO avg/max process_ms | |
| 是否仍 2~7Hz 跳变 | |
| CPU | |
| 结论 | |

## 测试 D：component intra-process + 外部 subscriber 对照

| 项目 | 结果 |
|---|---|
| Point-LIO cloud_cb Hz | |
| Point-LIO sync_in Hz | |
| 外部 min_sub Hz | |
| 内外差异 | |
| 结论 | |

## 最终判断

- DDS / 跨进程是否是主因：
- Point-LIO 调度是否是主因：
- Point-LIO 计算是否是主因：
- QoS 是否是主因：
- 后续需要继续优化的点：
