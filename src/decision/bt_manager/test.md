# 哨兵机器人行为树全面测试计划与流程规范 (Sentry BT Test Plan)

## 概述
本测试计划覆盖 MainTree(导航), StanceTree(云台底盘姿态), TacticalTree(战术模式) 以及 TunnelRecoverySubTree(隧道脱困) 的所有执行逻辑。

**核心测试机制**
- ReactiveFallback (响应式回退/优先抢占): 从上到下按优先级 Tick。一旦高优先级子节点返回 SUCCESS 或 RUNNING，正在执行的低优先级节点将被立即 HALT 并被抢占。
- ReactiveSequence (响应式顺序): 只要任意条件节点变为 FAILURE，后续所有的动作节点将立即被 HALT 打断。

## 模块一：导航主树 (NavTree) 测试项
导航树的优先级严格按照 P0 (最高) 到 P4 (最低) 划分。

### 1. 公共前置节点测试 (Common Check)
| 测试节点 | 测试条件 | 操作步骤 | 预期结果 (正向/反向) |
| --- | --- | --- | --- |
| CheckWillThroughTunnel | lifter_current_pos 与 through_tunnel | 1. 机器人进入 transform_zone。<br>2. 设定 through_tunnel = true。 | 正向: 黑板写入 desired_lifter_pos = BOTTOM，返回 SUCCESS。<br>反向: 离开区域或 through_tunnel = false 时，写入 TOP，返回 SUCCESS。 |

### 2. P0 - P4 优先级正反序抢占与节点测试

#### 2.1 P0：手动接管 (Manual Override)
| 测试节点 | 测试条件 (正反序 & 重复触发) | 操作步骤 | 预期结果 |
| --- | --- | --- | --- |
| CheckManualOverride, SetManualOverrideGoal | 正序抢占: 在 P4 巡逻时，切换 control_mode = MANUAL_CONTROL | 触发遥控器手动模式，推杆设定目标点 A | 立即打断 P4 巡逻，执行目标 A 的手动导航。 |
| CheckManualOverride | 反序压制: 在手动模式下，设定血量 < 30 (满足 P1撤退条件) | 手动模式下强行扣血至濒死 | 手动模式(P0)不受影响，继续执行手动导航，不执行撤退 (P0 优先级最高)。 |
| CheckManualOverride | 重复触发/超时测试: 手动目标点持续 30s 不改变 | 手动模式下不推摇杆，目标点变化量 < 0.05，保持 30s | 超时触发，节点返回 FAILURE。黑板 control_mode 强切为 AUTO，降级回 P4 自动巡逻。 |

#### 2.2 P1：生存撤退与买血 (Survival & Ammo)
| 测试节点 | 测试条件 (抢占 & 恢复) | 操作步骤 | 预期结果 |
| --- | --- | --- | --- |
| CheckRetreatCondition, SetHomeGoal | 正序抢占: 在 P2(追击目标) 途中，health < 30 或 ammo < 0 | 模拟血量骤降到 20 | 立即打断 P2，模式切换为 RETREAT，向 Goal 0 (Home) 导航。 |
| CheckRetreatCondition | 反序压制: 处于撤退状态时，目标进入视野 (满足 P2) | 在回家路上生成可见的敌方目标 | 无视目标。P1 优先级高于 P2，必须等 health >= 80 且 ammo >= 0 才能解除撤退。 |
| CheckAmmoLow, AccumulateAmmoPurchase | 后置重复触发: 导航回 Home 成功且 ammo < 100 | 机器人抵达 Home 点，剩余子弹 50 发 | 触发买弹逻辑(步长30)，单次买弹后返回 SUCCESS。由于外层是 ReactiveFallback，下一帧会重复触发，直到弹药 >= 100。 |

#### 2.3 P1.5：隧道脱困 (Tunnel Recovery)
| 测试节点 | 测试条件 (异常复现) | 操作步骤 | 预期结果 |
| --- | --- | --- | --- |
| CheckTimeInZone | 机器人在 transform_zone 停留时间在 [20.0, 50.0] 秒之间 | 让机器人在隧道区域原地卡住 20 秒以上 | 触发 P1.5 脱困子树(进入 RecoveryExecution)。 |

#### 2.4 P2：目标追击 (Target Attack)
| 测试节点 | 测试条件 (防抖与抢占) | 操作步骤 | 预期结果 |
| --- | --- | --- | --- |
| CheckTargetLocked | 防抖测试/重复触发: 目标短时间内闪烁 (丢失 < 1.0s) | 目标 Valid 状态变为 false，0.5s 后恢复 | CheckTargetLocked 内置容忍 <1s 丢失，持续返回 SUCCESS，不打断追击动作。 |
| CheckTargetLocked | 超时抢占: 目标丢失 > 1.0s | 遮挡目标 1.5s | 节点返回 FAILURE，追击终止，向下跳跃并降级执行 P3 或 P4。 |

#### 2.5 P3：特殊响应 (Special Response)
此部分为 ReactiveFallback，内部为平级抢占，仅由 tactical_mode 的状态决定。

| 分支名称 | 测试条件与节点 | 预期结果 |
| --- | --- | --- |
| NormalOutpostResponse | CheckNormalMode (normal模式)<br>CheckOutpostRemained (前哨站存活)<br>CheckOutpostSafeResponse (无受击稳定 >5s) | 设置 Goal 2 (EnemyOutpost) 导航。若途中突然掉血，CheckOutpostSafeResponse 会检测到血量下降，进入 5 秒 CD 并返回 FAILURE，动作被打断降级至 P4。 |
| DefendFortResponse | CheckDefendMode (defend模式)<br>CheckOwnFortIdle (fort_status == 0) | 设置 Goal 3 (OwnFort) 导航。若己方堡垒被占领 (fort_status != 0)，则返回 FAILURE，降级 P4。 |
| AttackFortResponse | CheckAttackMode (attack模式) | 设置 Goal 4 (EnemyFort) 导航。 |

#### 2.6 P4：常规巡逻 (Track Patrol)
| 测试节点 | 测试条件 | 操作步骤 | 预期结果 (重复触发) |
| --- | --- | --- | --- |
| SelectPatrolPoint, WaitAtPatrolPoint | 前置所有高优先级节点均为 FAILURE | 机器人处于完全安全、无目标的开阔地 | 选点 -> 导航 -> 等待 (WaitAtPatrolPoint) -> 序列完成返回 SUCCESS。下一帧重新评估，重复触发下一个巡逻点。 |

## 模块二：姿态主树 (StanceTree) 测试项
姿态树通过 ReactiveFallback 实现严格的条件优先级机制(最高优先级在上)。

| 优先级 / 分支名 | 测试条件与操作步骤 | 预期结果与抢占验证 |
| --- | --- | --- |
| P0: RuleRefresh | CheckStanceRefreshRequired: 在同一姿态下停留超过 180s | 强制重复触发: 瞬时返回 SUCCESS，输出反转的 target_stance。强切瞬态姿态(如 ATTACK 切 DEFEND)，刷新比赛规则限制，下一帧恢复原姿态。 |
| P1: MoveStanceNoGyro | CheckCrossZoneTransition: 跨区域且经过隧道，或位于隧道内 | 正序抢占: 即使正处于交战开火状态，只要满足过隧道条件，强制切 MOVE 姿态并关闭底盘小陀螺(由 PID 接管并输出 tunnel_speed_y)。 |
| P2: AbsoluteAttack | CheckHeat (>200) 或 CheckOutpostTarget (距前哨<0.5m) | 反序压制: 即使血量低于 40 (应触发P6防御)，若热量超标，P2 直接抢占，强切 ATTACK 姿态散热，陀螺仪设为 50.0。 |
| P3: PursuitStance | CheckTargetVisible 且 CheckTargetDistance (> 4.0m) | 距离过远，优先追赶。切换为 MOVE 姿态，开启高速小陀螺 90.0。 |
| P4: CombatAttack | CheckEngagedStatus (交战) 且 CheckHealth (> 60) | 正序剥离: 目标拉近至 <4.0m 时 P3 失效。平滑降级至 P4，切换 ATTACK 姿态，陀螺仪降速至 50.0。 |
| P5: MoveStanceGyro | CheckCapacitorCapacity (<30) 或 未交战省电/赶路模式 | 切 MOVE 姿态，关闭陀螺仪 (gyro_vel=0.0)。 |
| P6: DefendStance | CheckEngagedStatus (交战) 且 CheckHealth (< 40) | 濒死挨打状态。切 DEFEND 姿态，小陀螺 50.0。 |
| P7: DefaultDefend | 以上均不满足 | 默认兜底状态。切 DEFEND 姿态，小陀螺 50.0。 |

## 模块三：隧道脱困子树 (RecoveryTree) 测试项
注: 依据逻辑分析，XML 中 <Inverter><CheckTimeInZone min_seconds="0.0" .../></Inverter> 存在必定返回 FAILURE 的逻辑死锁(驻留时间 > 20s 才能进入该树，因此 >0 恒成立，Inverter 翻转后恒为 FAILURE)。以下为假定剔除该 Bug 后的预期流程。

| Phase | 节点测试条件 | 操作步骤 | 预期结果 |
| --- | --- | --- | --- |
| Phase 1 | CheckTimeInZone: 停留 20s - 25s | 将机器人在隧道卡住 20s | 条件成立。执行 SetTunnelRecoveryAttemptPoint，尝试向前脱困。 |
| Phase 2 | CheckTimeInZone: 停留 25s - 30s | 假定 Phase 1 失败，继续卡住达 25s | Phase 1 失效被抢占。Phase 2 接管，执行 SetTunnelRecoveryRetreatPoint，尝试向后倒车。 |
| Phase 3 | CheckTimeInZone: 停留 > 30s | 彻底卡死达 30s | Phase 1/2 均失效。接管全局速度，设 global_vx=0.0，彻底停止前进，仅依靠 tunnel_speed_y 等待摩擦。 |

## 模块四：战术模式树 (TacticalTree) 测试项
该树负责向黑板实时写入 tactical_mode 变量，直接影响 NavTree 中 P3 节点的选择。

| 测试节点 | 测试条件 | 操作步骤 | 预期结果 (状态机切换与联动) |
| --- | --- | --- | --- |
| AttackPriority | CheckAttackCondition:<br>1. enemy_outpost_destroyed = true<br>2. 大/小能量机关激活<br>3. home_health > 1000 | 模拟前哨站击毁，激活能量机关，保持基地满血 | 节点返回 SUCCESS，触发 SetAttackMode 写入 attack。联动: 导致 NavTree 放弃巡逻，立即切至 P3 的 AttackFortResponse 冲向敌方堡垒。 |
| DefendPriority (XML中已被注释屏蔽) | -- | -- | -- |
| NormalMode | 默认状态 (上述攻击条件不满足时) | 基地突然掉血至 <1000 (模拟被偷家) | AttackPriority 瞬间变为 FAILURE。回退至底部，写入 normal 模式。联动: NavTree 瞬间停止进攻敌方堡垒，逻辑降级。 |