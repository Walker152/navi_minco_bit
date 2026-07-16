# Point-LIO 可选 Blind 中心改造记录

## User Intent

将 MID360/Livox 输入点云的 `blind` 球心可选迁移到车辆 `base_link` 中心。代码默认关闭，当前车辆 `mid360.yaml` 启用 `[0.0, 0.20, 0.0]`。`det_range` 必须继续以 LiDAR 原点为中心。

## Scope

Point-LIO 参数读取、预处理类、Livox 普通与切帧过滤路径、节点初始化和当前 MID360 配置。

## Out of Scope

TF 发布、LiDAR/IMU 外参、非 Livox handler、特征提取、去畸变、EKF、地图和点云发布链路。

## Explorer Findings

### Files inspected

- `src/perception/Point-LIO/src/parameters.cpp`
- `src/perception/Point-LIO/src/parameters.h`
- `src/perception/Point-LIO/src/preprocess.cpp`
- `src/perception/Point-LIO/src/preprocess.h`
- `src/perception/Point-LIO/src/laserMapping.cpp`
- `src/perception/Point-LIO/src/li_initialization.cpp`
- `src/perception/Point-LIO/config/mid360.yaml`

### Active logic path

MID360 使用 Livox CustomMsg。普通路径通过 `Preprocess::avia_handler()`，切帧路径通过 `Preprocess::process_cut_frame_livox()`。

### Data flow

`readParameters()` 创建并配置 `p_pre`；`LaserMappingNode::initialize()` 在创建 Livox 订阅前设置实际 blind 球心；两条 Livox 路径在点进入 `pl_surf` 前过滤。

### Risk notes

本次修改涉及比赛验证逻辑，采用最小改动策略。用户已确认 `[0.0, 0.20, 0.0]` 是车辆中心相对雷达安装位置在输入点云坐标系中的关系。

### Recommended modification boundary

只拆分两条 Livox 路径中的 blind 距离和 LiDAR 原点距离，不修改其他 handler 或特征提取语义。

## Modifier Changes

### Files changed

- `src/perception/Point-LIO/src/parameters.cpp`
- `src/perception/Point-LIO/src/parameters.h`
- `src/perception/Point-LIO/src/preprocess.cpp`
- `src/perception/Point-LIO/src/preprocess.h`
- `src/perception/Point-LIO/src/laserMapping.cpp`
- `src/perception/Point-LIO/config/mid360.yaml`

### Key changes

新增 `preprocess.blind_center_enable` 和 `preprocess.blind_center`；新增预处理球心 setter 和零值成员；两条 Livox 路径分别计算 blind 中心距离与 LiDAR 原点距离。

### Behavior preserved

代码默认关闭时 blind 球心为 LiDAR 原点；`det_range`、非 Livox 路径以及其他点云处理链路保持不变。

### Behavior intentionally adjusted

当前 `mid360.yaml` 启用车辆中心 `[0.0, 0.20, 0.0]` 作为 blind 球心。

### Notes

数组长度非法时仅警告一次并回退到 LiDAR 原点，不阻止节点启动。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查
- [x] grep 检查
- [x] XML / launch / yaml 检查
- [x] 用户允许范围内的测试或静态检查
- [ ] 如需构建，已取得用户明确许可

### Issues found

未发现越界修改。未执行构建，因为 AGENTS.md 禁止未授权构建。

### Final result

PASS
