# bt_manager 🧠⚙️

> RoboMaster 哨兵机器人决策核心（ROS 2 + BehaviorTree.CPP v3）
>
> 现代化控制流：**Reactive BT + Pure Blackboard + Debounce/Throttle**

---

## 1. 架构简介 (Introduction)

`bt_manager` 是哨兵机器人的“**大脑节点**”，负责两条并行主链路：

- 🧭 **导航决策**：在巡逻、追击、前哨响应、撤退等策略之间进行优先级抢占与目标分发。
- 🎯 **姿态决策**：在 MOVE / ATTACK / DEFEND 间快速切换，驱动云台与底盘协同。

本包采用了 **Pure Blackboard** 设计：

- ✅ 摒弃繁琐的 XML 端口映射与复杂数据穿透。
- ✅ 关键状态由 C++ 底层直接读写全局黑板。
- ✅ 在响应式行为树中实现**低延迟、强可控、易调试**的数据驱动闭环。

> 📌 设计目标：在极端对抗场景下，保持“可抢占、可回退、可恢复”的稳定决策行为。

---

## 2. 核心导航策略与打断机制 (Navigation Strategies)

主树导航部分采用 `ReactiveFallback`，具备严格的**高优先级抢占低优先级**能力。

### 优先级总览（高 → 低）

| 优先级 | 分支名 | 节点类型 | 触发条件 | 目标/行为特性 |
|:--:|:--|:--|:--|:--|
| 1 | `StairsEvacuation` | `Sequence` | `health < 50` 且位于台阶区域（`CheckInStairsZone`） | 不可逆物理动作链，避免“抽搐式”反复打断 |
| 2 | `EmergencyRetreat` | `ReactiveSequence` | `health < 50` | 强制前往 `HOME`，高危场景优先保命 |
| 3 | `TargetPursuit` | `ReactiveSequence` | `target_valid == true` | 动态追击目标点，支持视觉/空间防抖 |
| 4 | `OutpostResponse` | `ReactiveSequence` | `enemy_outpost_destroyed == false` | 快速前往 `OUTPOST` 执行响应 |
| 5 | `RegularPatrol` | `Sequence` | 兜底逻辑（默认） | 顺序巡逻并带状态记忆，避免高频切点 |

### 工程亮点 ✨

- 👁️ **视觉防抖 Debounce（1.0s）**：短时丢帧不立刻丢失追击态，避免“闪断-回退-再追击”振荡。
- 📏 **空间限频 Throttle（0.5m）**：对追击目标刷新做位移门限，降低 Nav2 频繁重规划压力，提升稳定性。
- 🔁 **Reactive 抢占机制**：高优条件一旦成立，可在当前 Tick 周期内打断低优任务。

> 💡 `Sequence` 用于“必须完整执行”的动作链；`ReactiveSequence` 用于“可被更高优条件实时抢占”的行为链。

---

## 3. 姿态切换逻辑 (Stance Switching)

姿态链路位于主树顶层 `ForceSuccess` 中，与导航链路形成**双轨并行**：

- 导航在抢占，姿态也在独立抢占。
- 姿态判定采用绝对物理指标，不与导航模式做强耦合。

### 姿态优先级（高 → 低）

| 优先级 | 姿态 | 条件 | 说明 |
|:--:|:--|:--|:--|
| 1 | `DEFEND` | `health <= 30.0` | 强制防御（小陀螺/保命） |
| 2 | `ATTACK` | `target_valid == true` 或 `outpost_msg == true` | 进入进攻姿态 |
| 3 | `MOVE` | 其他条件均不满足 | 兜底移动姿态 |

### `ChangeStance` 动作节点机制

- ⏱️ 内置 **5 秒 CD**（物理冷却时间）。
- ✅ CD 内直接返回 `SUCCESS`，避免阻塞主树 Tick。
- 🔄 保证姿态切换有节律，降低频繁切换导致的执行器冲击。

---

## 4. 核心全局黑板变量一览 (Blackboard Dictionary)

| 变量名 | C++ 类型 | 用途 |
|:--|:--|:--|
| `health` | `float` | 当前血量（决策核心输入） |
| `enemy_outpost_destroyed` | `bool` | 敌方前哨站是否被摧毁 |
| `target_valid` | `bool` | 视觉锁敌标志（受 1s 防抖保护） |
| `target_pose` | `geometry_msgs::msg::Pose` | 敌人实时坐标（用于追击目标） |
| `nav_goal` | `Point2D` | 最终下发给 Nav2 的导航目标 |
| `current_mode` | `int` | 当前导航模式枚举（PATROL/TRACING/RETREAT/RESPONSE） |
| `current_stance` / `desired_stance` | `SentryStance` | 当前姿态 / 期望姿态 |
| `patrol_index` | `int` | 巡逻点索引（配合标准 Sequence 稳态更新） |

> ✅ 推荐实践：所有跨节点共享状态统一进入黑板，保持“单一事实源（Single Source of Truth）”。

---

## 5. 坐标域与状态配置 (`nav_zone` Configuration)

`bt_manager/utils/nav_zone.hpp` 与 `bt_manager/utils/nav_zone.cpp` 负责地图点位、巡逻序列及状态名称配置。

### 5.1 如何修改点位

直接修改 `Point2D` 的 `x`/`y` 即可：

```cpp
std::vector<Point2D> nav_points = {
  {3.0, 3.0},   // HOME
  {8.5, 8.5},   // BONUS
  {15.7, 11.0}  // OUTPOST
};
```

- `nav_points`：固定全局目标点（`HOME/BONUS/OUTPOST`）。
- `patrol_points_normal`：常规巡逻路径。
- `patrol_points_attack`：进攻态巡逻路径（可用于前压/压制策略）。

### 5.2 停留时间映射

`patrol_points_milliseconds` 与巡逻点数组按索引一一对应：

- `patrol_points_normal[i]` ↔ `patrol_points_milliseconds[i]`
- 行为树在每个点到达后，按该毫秒值执行 `Wait` 节点停留。

> 📌 调参建议：先固定点位，再调停留时间；避免同时改两组参数导致定位问题难复现。

---

## 6. 时间轴压测节点 (`event_status_pub_test`)

该测试节点用于执行“阶段化极限剧本”，验证 Reactive BT 在**导航优先级抢占**与**姿态优先级抢占**上的鲁棒性。

### 压测流程时间轴（1Hz 发布）

- **[0-15s]**：兜底常规巡逻测试（RegularPatrol + MOVE）。
- **[15-30s]**：视觉锁敌触发抢占（TargetPursuit + ATTACK）。
- **[30-45s]**：视觉丢失防抖测试（1.0s 记忆后平滑回落）。
- **[45-60s]**：前哨站响应打断测试（OutpostResponse 抢占巡逻）。
- **[60-75s]**：大掉血触发紧急撤退（EmergencyRetreat 强制抢占）。
- **[75-90s]**：濒死防御姿态测试（DEFEND 最高优先级姿态介入）。
- **[90-105s]**：回血重置测试（撤退解除，恢复响应/常规策略）。

### 运行示例

```bash
# 构建
colcon build --packages-select bt_manager

# 终端 1：启动决策节点
ros2 launch bt_manager bt_manager.launch.py

# 终端 2：启动时间轴压测发布器
ros2 run bt_manager event_test
```

---

## 附：工程准则 🏆

- 以响应式抢占保证战术实时性。
- 以黑板直连保证链路低延迟。
- 以防抖/限频保证复杂链路稳定运行。
- 以阶段化压测保证可回归、可复现、可演进。

---
