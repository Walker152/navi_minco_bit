# Codex 修改提示：收敛修复冲家姿态流程、大能量触发语义与强化姿态计时

请直接基于当前代码进行最小修改，不要重构整体行为树，不要增加无关状态机、枚举、黑板字段或封装。完成代码修改、格式检查和现有构建测试即可，不要创建额外计划文档，不要执行任何 Git 提交、推送、变基或历史修改操作。

## 一、修改目标

本次只处理以下三个问题：

1. 修正冲家姿态的三个连续阶段：
   - 冲家下坡并经过隧道时：`DEFEND + TunnelGyroAlignAction PID`
   - 完全离开隧道区、尚未进入敌方堡垒时：`MOVE + gyro_vel=0`
   - 进入敌方堡垒后：`DEFEND + gyro_vel=80`，持续小陀螺驻守

2. 修正大能量机关触发语义：
   - 仅在比赛剩余 `0~120 s` 的最后两分钟内检测并锁存；
   - 最后两分钟内只要曾收到一次“大能量机关已激活”，立即进入 `attack`；
   - 后续大能量状态恢复为未激活时，仍保持 `attack`；
   - 比赛结束、离开比赛中状态，或下一局比赛时间恢复到 120 秒以上时，必须清除锁存。

3. 删除强化姿态的本地剩余时间倒计时：
   - 强化姿态剩余时间只使用裁判系统写入的 `enhanced_*_remaining_time`；
   - `UpdateStanceDuration` 继续使用裁判系统 `game_time_remaining` 的差值统计普通姿态累计时间；
   - 不再读取或写入 `enhanced_*_remaining_sec`。

---

## 二、严格限制修改范围

主要只修改：

```text
stance_tree.xml
tactical_tree.xml
auto_conditions.cpp
change_stance_action.cpp
```

如果编译确实要求同步修改对应头文件，只做必要的声明适配。

明确不要修改：

```text
nav_tree.xml
nav_action.hpp
Wait 节点及巡逻等待逻辑
recovery_tree.xml
recovery_actions.cpp
resource_tree.xml
resource_conditions.cpp
resource_actions.cpp
gimbal_tree.xml
gimbal_condition.cpp
gimbal_action.cpp
ros_interface.cpp
area.hpp
nav_zone.hpp
手动接管逻辑
隧道检测逻辑
隧道编号判断
PID 参数
敌方堡垒区域定义
导航目标和导航停止逻辑
姿态分支的整体优先级关系
```

不要讨论或修复其他审计问题。当前不可达条件、资源树、云台分区巡检、前哨状态来源、巡逻等待时间等均不在本次范围内。

---

# 三、冲家姿态流程修改

## 3.1 必须保留当前隧道优先级

当前 `stance_tree.xml` 顶部两个隧道分支的顺序和职责保持不变：

```text
1. ManualAttackTunnelDefendAlignBranch
   attack + CheckCrossZoneTransition
   → DEFEND + TunnelGyroAlignAction

2. MoveStanceNoGyroBranch
   普通 CheckCrossZoneTransition
   → MOVE + TunnelGyroAlignAction
```

不要删除、合并或交换这两个分支。

原因：

- 冲家规划路径必定经过既定隧道；
- `attack` 隧道分支必须先于普通隧道分支；
- 冲家下坡过隧道时必须使用防御姿态；
- 隧道 PID 对齐逻辑和参数保持原样；
- 当 `CheckCrossZoneTransition` 退出后，行为树自然落到后续冲家阶段分支。

预期阶段一：

```text
tactical_mode == attack
&& CheckCrossZoneTransition == SUCCESS
→ desired_stance = DEFEND
→ TunnelGyroAlignAction 持续输出 PID gyro_vel
```

## 3.2 仅重写 `AttackFortDefendGyroBranch` 内部阶段选择

当前问题是：

```text
堡垒外：gyro_vel=0
堡垒内：gyro_vel=80
但两种情况最后都统一切换为 DEFEND
```

因此当前“出洞后上堡垒”错误地使用了防御姿态。

保持该分支在 `ManualEnhancedStanceBranch` 之后、`AbsoluteAttackBranch` 之前的位置不变，只修改其内部结构，使其按堡垒区域同时选择姿态和角速度。

建议改成等价于以下结构：

```xml
<ReactiveSequence name="AttackFortDefendGyroBranch">
  <CheckTacticalModeCondition
      mode="attack"
      branch="AttackFortDefendGyroBranch" />

  <ReactiveFallback name="EnemyFortStanceSelector">

    <!-- 阶段三：已经进入敌方堡垒，防御姿态持续小陀螺 -->
    <Sequence name="ArrivedEnemyFortDefendGyro">
      <CheckInEnemyFortZone
          branch="ArrivedEnemyFortDefendGyro" />

      <SetGyroState
          use_gyro="true"
          gyro_vel="80.0"
          random_speed="false" />

      <Sequence name="ArrivedEnemyFortDefendStanceGate">
        <Inverter>
          <CheckManualStanceOverride
              branch="ArrivedEnemyFortDefendGyro" />
        </Inverter>

        <ChangeStance
            name="ChangeStanceArrivedEnemyFortDefend"
            stance="DEFEND" />
      </Sequence>
    </Sequence>

    <!-- 阶段二：已经出隧道但尚未进入堡垒，移动姿态零角速度冲堡垒 -->
    <Sequence name="ApproachEnemyFortMoveZeroGyro">
      <Inverter>
        <CheckInEnemyFortZone
            branch="ApproachEnemyFortMoveZeroGyro" />
      </Inverter>

      <SetGyroState
          use_gyro="true"
          gyro_vel="0.0"
          random_speed="false" />

      <Sequence name="ApproachEnemyFortMoveStanceGate">
        <Inverter>
          <CheckManualStanceOverride
              branch="ApproachEnemyFortMoveZeroGyro" />
        </Inverter>

        <ChangeStance
            name="ChangeStanceApproachEnemyFortMove"
            stance="MOVE" />
      </Sequence>
    </Sequence>

  </ReactiveFallback>
</ReactiveSequence>
```

允许根据项目 XML 风格微调节点名称，但不得改变业务含义。

## 3.3 修改后的完整抢占流程

必须确保实际 tick 结果为：

```text
A. attack + 隧道条件成立
   → 顶部 ManualAttackTunnelDefendAlignBranch
   → DEFEND + PID

B. attack + 隧道条件失败 + 不在 enemy_fort_zone
   → AttackFortDefendGyroBranch 的 Approach 分支
   → MOVE + use_gyro=true + gyro_vel=0

C. attack + 隧道条件失败 + 在 enemy_fort_zone
   → AttackFortDefendGyroBranch 的 Arrived 分支
   → DEFEND + use_gyro=true + gyro_vel=80
```

不要增加 `attack_phase`、`attack_fort_arrived` 或新的黑板锁存。

不要修改导航逻辑。进入堡垒后停止平移由现有导航目标完成；姿态树只负责持续输出 `DEFEND + 80 rpm`。

---

# 四、大能量机关最后两分钟锁存

## 4.1 修改 `CheckBigEnergyActive`

当前问题：

```cpp
if (big_energy_status == 1) {
  energy_activated = true;
}
```

存在以下缺陷：

- XML 的 `active_status` 没有实际参与比较；
- 锁存不会在下一局复位；
- 节点当前位于时间窗口节点之后，窗口之外不会被 tick，无法自行清除旧锁存。

修改 `CheckBigEnergyActive::tick()`，使用以下语义：

```cpp
const int active_status =
  getInput<int>("active_status").value_or(1);

const int big_energy_status =
  blackboard->get<int>("big_energy_status");

const int game_status =
  blackboard->get<int>("game_status");

const int game_time_remaining =
  blackboard->get<int>("game_time_remaining");

const bool in_last_two_minutes =
  game_status == 4 &&
  game_time_remaining >= 0 &&
  game_time_remaining <= 120;

if (!in_last_two_minutes) {
  energy_activated = false;
  active = false;
} else {
  if (big_energy_status == active_status) {
    energy_activated = true;
  }
  active = energy_activated;
}
```

返回值：

```cpp
return active
  ? BT::NodeStatus::SUCCESS
  : BT::NodeStatus::FAILURE;
```

更新日志，使其至少包含：

```text
game_status
game_time_remaining
in_last_two_minutes
big_energy_status
active_status
energy_activated
```

`providedPorts()` 中 `active_status` 的默认值改为 `1`，与当前 XML 和裁判系统“已激活”状态一致。

不要把状态 `2`“正在激活”作为冲家触发条件。

## 4.2 调整 `tactical_tree.xml` 的节点顺序

当前顺序是：

```text
CheckGameTimeWindow
→ CheckBigEnergyActive
```

这会导致窗口之外 `CheckBigEnergyActive` 不执行，无法复位。

调整为：

```xml
<Sequence name="AutoAttackFortCondition">
  <CheckBigEnergyActive
      name="CheckAutoAttackFortBigEnergy"
      active_status="1"
      branch="AutoAttackFortCondition" />

  <CheckGameTimeWindow
      name="CheckAutoAttackFortWindow"
      min_remaining="0"
      max_remaining="120"
      branch="AutoAttackFortCondition" />
</Sequence>
```

虽然 `CheckBigEnergyActive` 内部已经检查最后两分钟，仍保留 `CheckGameTimeWindow` 作为战术树的显式时间门控，不要删除它。

必须得到以下结果：

```text
剩余 121 秒，大能量状态为 1
→ 清除锁存，normal

剩余 120 秒，大能量尚未激活
→ normal

剩余 100 秒，大能量状态首次变为 1
→ 锁存成功，attack

剩余 90 秒，大能量状态恢复为 0
→ 仍保持 attack

比赛结束或下一局恢复到 420 秒
→ 清除锁存，normal
```

手动 `CheckAttackCondition` 分支保持原样，仍然高于自动冲家条件。

---

# 五、删除强化姿态本地倒计时

修改 `UpdateStanceDuration::tick()`。

## 5.1 删除内容

删除：

```cpp
const int int_delta = raw_delta;
```

以及整个强化姿态本地倒计时 switch：

```cpp
switch (current_stance) {
case SentryStance::ENHANCED_ATTACK:
  get/set enhanced_attack_remaining_sec
  break;
case SentryStance::ENHANCED_DEFEND:
  get/set enhanced_defend_remaining_sec
  break;
case SentryStance::ENHANCED_MOVE:
  get/set enhanced_move_remaining_sec
  break;
default:
  break;
}
```

项目中不应再由该节点读取或写入：

```text
enhanced_attack_remaining_sec
enhanced_defend_remaining_sec
enhanced_move_remaining_sec
```

## 5.2 必须保留内容

保留当前基于裁判系统比赛剩余时间的累计逻辑：

```cpp
const int raw_delta = last_game_time_ - current_time;
const double delta = static_cast<double>(raw_delta);
```

保留：

```text
ATTACK / ENHANCED_ATTACK → attack_accumulated_time
DEFEND / ENHANCED_DEFEND → defend_accumulated_time
MOVE / ENHANCED_MOVE → move_accumulated_time
```

强化姿态剩余可用时间只使用现有裁判系统键：

```text
enhanced_attack_remaining_time
enhanced_defend_remaining_time
enhanced_move_remaining_time
```

这些键继续由 ROS 在线信息写入，并由 `CheckManualStanceOverride` 读取。不要改名，不要在 `UpdateStanceDuration` 中修改它们。

同步更新注释，明确：

```text
UpdateStanceDuration 只根据裁判系统 game_time_remaining 统计姿态累计时间；
强化姿态剩余时间完全由裁判系统反馈管理。
```

---

# 六、禁止附带修改

本次不要顺手修改：

- 巡逻点和 Wait；
- 导航超时；
- 前哨响应；
- 补给和资源树；
- 手动模式；
- 云台树；
- 隧道恢复；
- `CheckCrossZoneTransition`；
- `TunnelGyroAlignAction`；
- `SetGyroState` 的正反转逻辑；
- 姿态冷却；
- 任何区域坐标；
- attack 战术的整体优先级；
- 其他日志或统计。

不要为了“统一结构”重命名无关节点或大面积格式化文件。

---

# 七、验收条件

修改后必须逐项确认。

## 7.1 姿态树

### 场景 1：冲家下坡过隧道

输入：

```text
tactical_mode = attack
CheckCrossZoneTransition = SUCCESS
```

预期：

```text
ManualAttackTunnelDefendAlignBranch 被选中
desired_stance = DEFEND
TunnelGyroAlignAction = RUNNING
gyro_vel 为 PID 输出
AttackFortDefendGyroBranch 不执行
```

### 场景 2：完全出隧道，尚未进入堡垒

输入：

```text
tactical_mode = attack
CheckCrossZoneTransition = FAILURE
CheckInEnemyFortZone = FAILURE
```

预期：

```text
desired_stance = MOVE
use_gyro_mode = true
gyro_vel = 0
```

### 场景 3：进入敌方堡垒

输入：

```text
tactical_mode = attack
CheckCrossZoneTransition = FAILURE
CheckInEnemyFortZone = SUCCESS
```

预期：

```text
desired_stance = DEFEND
use_gyro_mode = true
gyro_vel = 80
```

### 场景 4：普通模式经过隧道

输入：

```text
tactical_mode = normal
CheckCrossZoneTransition = SUCCESS
```

预期：

```text
MoveStanceNoGyroBranch 被选中
desired_stance = MOVE
TunnelGyroAlignAction = RUNNING
```

## 7.2 大能量触发

确认：

```text
窗口外不会保留旧锁存
窗口内首次收到 active_status=1 后进入 attack
窗口内状态恢复为 0 后仍保持 attack
比赛结束或下一局开始时锁存清除
```

## 7.3 强化姿态计时

全项目搜索确认：

```text
UpdateStanceDuration 不再引用 enhanced_*_remaining_sec
CheckManualStanceOverride 仍读取 enhanced_*_remaining_time
普通姿态累计仍基于 game_time_remaining 差值更新
```

---

# 八、完成要求

完成后请输出：

1. 实际修改的文件列表；
2. 每个文件的核心改动；
3. 三阶段冲家姿态的最终 tick 路径；
4. 大能量锁存的复位与保持逻辑；
5. 搜索结果，证明本地强化倒计时已移除；
6. 执行过的格式化、编译或测试命令及结果；
7. 如无法编译，说明具体缺失依赖或错误，不要声称已经通过。

不要执行 Git 提交。
