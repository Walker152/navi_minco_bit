# 冲家姿态与大能量锁存修复改造记录

## User Intent

修正冲家隧道、堡垒外、堡垒内三个连续姿态阶段；将大能量触发限制在比赛最后两分钟并正确锁存和复位；删除强化姿态本地剩余时间倒计时。

## Scope

- 行为树 / 自主决策 bug 修复。
- 仅修改姿态树、战术树、大能量条件节点和姿态累计节点。
- 允许按用户明确要求调整冲家姿态和自动冲家触发行为。
- 本次修改涉及比赛验证逻辑，采用最小改动策略。

## Out of Scope

导航、隧道检测与 PID、区域定义、资源树、云台树、恢复树、手动接管、通信协议、参数默认值及其他比赛策略均不处理。

## Explorer Findings

### Files inspected

- `codex_prompt_attack_fort_stance_fix.md`
- `src/decision/bt_manager/tree/stance_tree.xml`
- `src/decision/bt_manager/tree/tactical_tree.xml`
- `src/decision/bt_manager/src/condition/auto_conditions.cpp`
- `src/decision/bt_manager/include/bt_manager/condition/auto_conditions.hpp`
- `src/decision/bt_manager/src/action/change_stance_action.cpp`
- `src/decision/bt_manager/src/condition/change_stance_condition.cpp`
- `src/decision/bt_manager/src/ros_interface.cpp`
- `src/decision/bt_manager/include/bt_manager/blackboard.hpp`

### Active logic path

`stance_tree.xml` 中冲家隧道分支优先于普通隧道分支，二者又优先于强化姿态和冲家堡垒分支。原冲家堡垒分支只在堡垒内外选择角速度，随后统一设置 `DEFEND`。`tactical_tree.xml` 原先先检查时间窗口，导致窗口外不 tick 大能量锁存节点。

### Data flow

- `game_status`、`game_time_remaining`、`big_energy_status` 由 ROS 接口写入 blackboard。
- `CheckBigEnergyActive` 读取上述键并决定自动冲家条件。
- `enhanced_*_remaining_time` 由 ROS 接口写入，`CheckManualStanceOverride` 读取。
- `UpdateStanceDuration` 使用 `game_time_remaining` 差值更新三个姿态累计时间。

### Risk notes

`ReactiveFallback` 和 `ReactiveSequence` 的顺序决定抢占行为；修改必须保持顶部隧道分支和手动强化分支优先级。大能量成员锁存必须在窗口外也被 tick 才能复位。

### Recommended modification boundary

仅在 `AttackFortDefendGyroBranch` 内按区域同时设置姿态和角速度；交换自动冲家两个条件节点顺序；在大能量条件节点内部实现时间窗口锁存；删除本地强化倒计时 switch。

## Modifier Changes

### Files changed

- `src/decision/bt_manager/tree/stance_tree.xml`
- `src/decision/bt_manager/tree/tactical_tree.xml`
- `src/decision/bt_manager/src/condition/auto_conditions.cpp`
- `src/decision/bt_manager/src/action/change_stance_action.cpp`
- `docs/ai_refactor_records/20260714_attack_fort_stance_fix.md`

### Key changes

- 堡垒外设置 `MOVE + gyro_vel=0`，堡垒内设置 `DEFEND + gyro_vel=80`。
- 大能量状态仅在 `game_status == 4` 且剩余时间为 `0~120` 秒时锁存；窗口外清除。
- `active_status` 默认值改为 `1`，并实际参与比较。
- 删除 `enhanced_*_remaining_sec` 的本地递减逻辑。
- 将隧道后的 `ManualEnhancedStanceBranch` 保留为唯一强化覆盖检查入口，删除后续自动姿态分支中的六处重复 gate。

### Behavior preserved

隧道分支顺序、PID 参数、手动攻击优先级、手动强化优先级、普通姿态累计桶及所有 topic、frame、blackboard key 均保持不变。强化覆盖成功时仍由外层 `ReactiveFallback` 阻止所有后续自动姿态分支执行。

### Behavior intentionally adjusted

出隧道但未进入敌方堡垒时由 `DEFEND` 改为 `MOVE`；大能量自动冲家锁存增加局次和最后两分钟边界；强化剩余时间仅采用裁判系统反馈。

### Notes

未修改头文件，现有 `energy_activated` 成员足以实现锁存。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查
- [x] grep 检查
- [x] XML / launch / yaml 检查
- [x] 用户允许范围内的测试或静态检查
- [ ] 如需构建，已取得用户明确许可

### Issues found

无本次修改引入的问题。工作区原有 `src/decision/bt_manager/tree/nav_tree.xml` 修改未触碰。

### Final result

PASS
