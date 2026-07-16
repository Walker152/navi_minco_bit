# MINCO 杆臂补偿偏置参数化改造记录

## User Intent

将 `minco_utils` 中杆臂补偿硬编码的 `0.2` 改为从 YAML 加载的参数，并参考 controller 的 `lidar_offset_x/y` 命名和公式。

## Scope

- `src/navigation/minco_planner/include/minco_core/minco_utils.hpp`
- `src/navigation/minco_planner/src/minco_core/minco_utils.cpp`
- `src/navigation/minco_planner/include/minco_core/minco_planner.hpp`
- `src/navigation/minco_planner/src/minco_core/minco_planner.cpp`
- `src/navigation/navi2_bringup/params/sentry1.yaml`
- 本改造记录

## Out of Scope

- Controller 杆臂补偿逻辑
- Odom topic、frame、QoS 与发布频率
- 其他 planner/controller 参数

## Explorer Findings

### Files inspected

- `src/navigation/minco_planner/include/minco_core/minco_utils.hpp`
- `src/navigation/minco_planner/src/minco_core/minco_utils.cpp`
- `src/navigation/minco_planner/include/minco_core/minco_planner.hpp`
- `src/navigation/minco_planner/src/minco_core/minco_planner.cpp`
- `src/navigation/minco_controller/include/minco_controller/minco_mpc_controller.hpp`
- `src/navigation/minco_controller/src/minco_mpc_controller.cpp`
- `src/navigation/navi2_bringup/params/sentry1.yaml`

### Active logic path

`MincoPlanner::getCurrentSpeed()` 从最新 odom 读取激光雷达速度并调用 `utils::compensateLeverArm()`，原函数将 Y 方向杆臂偏置硬编码为 `-0.2 m`。

### Data flow

`sentry1.yaml` 的 `planner_server.ros__parameters.MincoPlanner.lidar_offset_x/y` 由 `MincoPlanner::configure()` 声明并读取到成员变量，再传入 `utils::compensateLeverArm()`。

### Risk notes

本次修改涉及比赛验证逻辑，采用最小改动策略。默认值与原硬编码保持等价：`lidar_offset_x = 0.0`、`lidar_offset_y = -0.2`。

### Recommended modification boundary

只参数化杆臂 X/Y 偏置，不改变速度坐标旋转、角速度输出或 odom 通信链路。

## Modifier Changes

### Files changed

- `src/navigation/minco_planner/include/minco_core/minco_utils.hpp`
- `src/navigation/minco_planner/src/minco_core/minco_utils.cpp`
- `src/navigation/minco_planner/include/minco_core/minco_planner.hpp`
- `src/navigation/minco_planner/src/minco_core/minco_planner.cpp`
- `src/navigation/navi2_bringup/params/sentry1.yaml`

### Key changes

- `compensateLeverArm()` 新增 `lidar_offset_x/y` 入参。
- 公式改为与 controller 一致：`v_body_x = v_lidar_x + omega_z * lidar_offset_y`、`v_body_y = v_lidar_y - omega_z * lidar_offset_x`。
- MincoPlanner 新增对应成员、参数声明与读取。
- YAML 的 MincoPlanner 段新增 `lidar_offset_x: 0.0`、`lidar_offset_y: -0.20`。

### Behavior preserved

- 默认配置下补偿结果与原硬编码完全一致。
- yaw 旋转、角速度输出、odom topic 和通信方式不变。

### Behavior intentionally adjusted

杆臂 X/Y 偏置现在可通过 MincoPlanner YAML 配置。

### Notes

未修改 controller 参数或逻辑。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查
- [x] grep 检查
- [x] XML / launch / yaml 检查
- [x] 用户允许范围内的测试或静态检查
- [ ] 如需构建，已取得用户明确许可

YAML 使用 `yaml.safe_load()` 解析通过；函数声明、定义和唯一调用点参数一致；`git diff --check` 通过。

未执行构建：AGENTS.md 禁止在未获用户明确许可前运行构建命令。

### Issues found

未发现静态检查问题。尚未执行编译检查。

### Final result

PASS（静态检查）
