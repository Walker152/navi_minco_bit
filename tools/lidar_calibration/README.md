# 雷达 x/y 与 Roll 标定工具

本工具针对当前工程的 `/aft_mapped_to_init`，计算：

- 旋转中心到 `body`/雷达 LIO 原点的平面 `lidar_offset_x/y`；
- 水平地面上的 `lidar_roll_offset` 候选值；
- 圆拟合、角度覆盖、分段漂移、姿态和时间质量指标。

Pitch 只用于诊断，不会导出为待标定参数。Yaw 和 z 不做标定。可选云台编码器只用于质量对比，不参与 x/y 主计算。

面向首次使用者的完整上车操作、结果解释、参数填写和低速验证流程见：

```text
docs/雷达_xy_roll_标定工具使用方法.md
```

## 坐标与符号

每个时刻先计算世界系半径向量：

```text
r_world = lidar_position_world - fitted_circle_center
```

再使用 odom yaw 做二维反旋转：

```text
dx =  cos(yaw) * r_world.x + sin(yaw) * r_world.y
dy = -sin(yaw) * r_world.x + cos(yaw) * r_world.y
```

所以输出方向固定为：

```text
旋转中心 → body/lidar 原点
```

该方向与当前 planner/controller 的杆臂补偿公式一致。工具不会自动修改正式参数文件。

## 采集

单雷达、最小链路：

```bash
bash scripts/collect_lidar_calibration.bash
```

双雷达：

```bash
bash scripts/collect_lidar_calibration.bash --mixed
```

同时启动 communication 并录制可选编码器：

```bash
bash scripts/collect_lidar_calibration.bash --with-communication
```

如果 Livox 和 Point-LIO 已经启动：

```bash
bash scripts/collect_lidar_calibration.bash --existing
```

脚本不会自动控制云台或底盘。确认安全后再手动低速旋转云台，至少一圈，推荐 1.5～2 圈。

## 分析

先加载 ROS 和工作空间：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

列出 bag 中的话题：

```bash
python3 tools/lidar_calibration/cli.py /path/to/bag --list-topics
```

标定：

```bash
python3 tools/lidar_calibration/cli.py \
  /path/to/bag \
  --odom-topic /aft_mapped_to_init
```

带编码器质量检查：

```bash
python3 tools/lidar_calibration/cli.py \
  /path/to/bag \
  --odom-topic /aft_mapped_to_init \
  --gimbal-topic /sentry/offline_info
```

默认丢弃最初 5 秒。可按实际初始化时间调整：

```bash
python3 tools/lidar_calibration/cli.py /path/to/bag --discard-seconds 8
```

## 输出

默认写到 `<bag>/lidar_calibration_report/`：

```text
report.md
report.json
candidate_parameters.yaml
calibration.svg
```

SVG 由工具直接生成，不依赖 Matplotlib。候选 YAML 只用于人工检查和复制，不会覆盖 `sentry1.yaml`。

退出码：

- `0`：`PASS` 或 `WARNING`，结果与告警已写入报告；
- `2`：`FAIL` 或输入不可标定；
- `130`：用户中断。

## 数据要求

- Point-LIO 初始化时保持静止；
- 启动时雷达 x 轴朝前；
- 底盘和旋转轴位置固定；
- 云台低速稳定旋转；
- Yaw 覆盖至少 270°，推荐 330°以上；
- 不使用高速小陀螺数据做首次标定。

## Roll 说明

Roll 来自 `/aft_mapped_to_init.pose.pose.orientation`。工具同时计算：

- 普通 RPY Euler Roll；
- 将世界竖直方向变换到 body 后得到的重力方向 Roll。

输出采用重力方向法，两种方法的差值作为质量指标。Pitch 只用于检查旋转轴、地面和初始化稳定性。
