# delta_yaw 异常清零与通信 100 Hz 修复改造记录

## User Intent

修复 odom 时间戳未匹配到底盘 IMU 历史时 `delta_yaw_` 被错误覆盖为 0 的问题；首次获得有效偏角前禁止世界系平移；将通信定时器从 200 Hz 调整为 100 Hz。保持现有最近邻时间匹配、20 ms 默认窗口、协议和发送顺序不变。

## Scope

- `src/navigation/communication/include/com_interface_ros.hpp`
- 本事后审计记录

本次修改属于 launch / communication 模块的 bug 修复和定时器调整，允许按用户要求改变上述运行行为。本次修改涉及比赛验证逻辑，采用最小改动策略。

## Out of Scope

- 串口包长度、payload、CRC、重同步、接收缓冲区和未对齐读取
- 串口协议、波特率、MCU 时间戳和 ROS/MCU 时间同步
- IMU 插值、外推、动态窗口、TF 补偿和复杂滤波
- path timer、2 ms 包间隔、执行器和线程模型

## Explorer Findings

### Files inspected

- `codex_delta_yaw_100hz_fix.md`
- `src/navigation/communication/include/com_interface_ros.hpp`
- `src/navigation/communication/src/main.cpp`
- communication 包文件清单、目标文件近期历史与当前工作区 diff

### Active logic path

`publishSentryInfoOffline()` 以 ROS `now()` 记录底盘 IMU yaw；`odomCB()` 保存 odom 后调用 `updateDeltaYaw()`；后者在 `communication.imu_yaw_window_ms` 内选择最近样本。原实现无论是否匹配成功，最终都会把局部变量 `delta` 写入 `delta_yaw_`。

### Data flow

底盘 IMU 历史和 odom orientation 生成 `delta_yaw_`；`communicationLoop()` 在 `state_mutex_` 下获取快照，并通过 `ChassisTarget` 发送给底盘。通信与订阅回调位于不同 callback group，共享状态依靠 mutex 保护。

### Risk notes

- 匹配失败时局部 `delta` 默认是 0，导致已有有效偏角被清零。
- 启动默认值 0 不能区分“未初始化”和合法的 0° 偏角。
- 首次匹配前发送世界系平移会使底盘使用未校准偏角转换速度。

### Recommended modification boundary

仅在目标头文件中增加独立初始化状态、失败早返回、首次初始化保护和 10 ms 通信周期，不改变现有最近邻搜索和通信协议。

## Modifier Changes

### Files changed

- `src/navigation/communication/include/com_interface_ros.hpp`
- `docs/ai_refactor_records/20260715_delta_yaw_100hz_fix.md`

### Key changes

- 未匹配或 yaw 非有限时提前返回，保持最后有效 `delta_yaw_`。
- 成功计算时在同一 `state_mutex_` 临界区更新偏角和初始化状态。
- 首次有效匹配前将 `vx_mps`、`vy_mps`、`fx_global`、`fy_global` 置零并节流警告。
- 将 `com_timer_` 周期从 5 ms 改为 10 ms。

### Behavior preserved

- IMU 样本使用 ROS `now()`，odom 使用 header stamp。
- 20 ms 默认窗口和窗口内最近邻匹配算法。
- yaw 归一化范围 `[-180°, 180°]`。
- 角速度、`fw_global`、path timer、发送顺序和 2 ms 包间隔。

### Behavior intentionally adjusted

- 匹配失败不再把偏角清零。
- 首次校准前不发送世界系平移和横向力指令。
- NAV/Behavior 通信循环由 200 Hz 调整为 100 Hz。

### Notes

未修改串口解析、协议结构、`main.cpp` 或其他通信源文件。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查
- [x] grep 检查
- [x] XML / launch / yaml 检查（本次未修改，确认参数默认值和 path timer 保持不变）
- [x] 用户允许范围内的测试或静态检查
- [ ] 如需构建，已取得用户明确许可

### Issues found

未发现越界修改或其他 `delta_yaw_ = 0` 写入路径。未执行构建，因为 AGENTS.md 禁止未授权构建。

### Final result

PASS
