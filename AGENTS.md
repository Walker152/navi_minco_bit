# AGENTS.md

# 项目 Agent 行为准则

本文件是本仓库中 AI Agent、Codex、Claude、superpower skill、多 agent 工作流的统一行为规范。  
所有代码修改、审计、重构、调参、文档化任务都必须优先阅读并遵守本文件。

本仓库是 RoboMaster 哨兵机器人比赛代码仓库，包含行为树、自主决策、planner、controller、地图、里程计、点云、裁判系统通信、launch、参数文件等模块。仓库中存在已经经过比赛验证的逻辑，任何修改都必须以“保持可复现、最小改动、可审计”为第一原则。

---

## 0. 最高优先级限制：编译构建限制

**禁止在未经用户明确允许的情况下执行任何编译或构建操作**，包括但不限于：

- `colcon build`
- `catkin build` / `catkin_make`
- `cmake` 构建
- `make` / `ninja`
- `cargo build` / `cargo check`
- `npm run build` / `yarn build`
- 任何其他会触发代码编译、链接的命令

在执行任何编译构建命令之前，必须先向用户确认并获得明确许可。

允许在未确认前执行的低风险检查包括：

- `grep` / `rg`
- `find`
- `ls`
- `sed` / `cat`
- `python3` 静态解析脚本
- XML 语法解析
- YAML 语法解析
- git diff / git status
- 不触发编译、链接、生成代码的静态检查

如果不确定某条命令是否会触发构建，必须先询问用户。

---

## 1. 项目背景

本仓库包含但不限于以下模块：

- 行为树与自主决策：`bt_manager`、行为树 XML、blackboard、裁判系统通信。
- 规划器：MINCO planner、路径搜索、轨迹优化、重规划逻辑。
- 控制器：MPC controller、controller_server、速度输出、参数切换。
- 地图与感知：ROGMap、ProjectionLayer、ESDF、地形投影、动态障碍处理。
- 里程计与点云：Point-LIO / Batch-LIWO、Livox 驱动、完整点云输出、去畸变、odom。
- ROS 2 通信：component container、intra-process、QoS、topic、launch、参数文件。
- 比赛策略：前哨站、冲家、补给、复活、姿态、小陀螺、隧道、台阶、资源兑换。
- 文档与实验材料：论文、申请材料、性能记录、CSV、图片、演讲稿等。

这是比赛代码，不是单纯实验仓库。  
不要为了“理论更优”而随意改变比赛验证过的逻辑。

---

## 2. 总原则

所有 agent 必须遵守：

1. 不破坏比赛验证过的主逻辑。
2. 不进行无关重构。
3. 不顺手删除历史逻辑、注释分支或旧方案，除非用户明确要求。
4. 不引入大量新变量、新函数、新模块。
5. 不改变已有 topic、frame、参数名、blackboard key，除非用户明确要求。
6. 不改变模块边界，除非这是本次任务目标。
7. 不添加高频 debug 日志。
8. 不添加无用统计。
9. 不擅自修改 launch 组合方式、QoS、timer、callback group。
10. 不擅自修改比赛参数默认值。
11. 不把未验证推断写成确定事实。
12. 每次非平凡修改必须留下改造记录。

如果用户没有明确要求重构，应优先选择：

```text
最小修改 > 局部修补 > 结构整理 > 大规模重构
```

---

## 3. 用户确认时使用多 Agent 工作流

每次非平凡代码修改，如果用户明确说明使用三阶段流程：

```text
Explorer Agent → Modifier Agent → Auditor Agent
```

多 agent 的目标不是让多个 agent 同时乱改，而是通过角色隔离降低误判风险。

### 3.1 Explorer Agent：探索与事实确认

Explorer Agent 只读仓库，不修改代码。

职责：

1. 找到与用户需求相关的源码、头文件、XML、launch、yaml、参数文件。
2. 梳理调用链、数据流、topic、blackboard key、参数来源。
3. 区分“当前生效逻辑”和“注释/废弃/历史逻辑”。
4. 标记比赛验证逻辑、隐式依赖、潜在冲突。
5. 给 Modifier Agent 明确修改边界。
6. 把探索结果写入改造记录。

必须检查：

- XML 或 launch 中实际启用的路径。
- cpp/hpp 中对应节点是否注册、是否被调用。
- blackboard key 的读写者。
- ROS topic 的发布者、订阅者、QoS、频率限制。
- 参数是否来自 yaml、declare_parameter、硬编码或 launch override。
- 是否有 static、全局变量、类成员状态影响多实例行为。
- 是否存在同名节点、重复分支、历史注释误导。
- 是否有用户明确要求不能动的模块。

Explorer Agent 禁止修改文件。

### 3.2 Modifier Agent：最小修改

Modifier Agent 只能根据 Explorer Agent 的记录和用户目标修改代码。

职责：

1. 做最小范围修改。
2. 保留原有行为优先级。
3. 不做用户未要求的清理、重命名、重构。
4. 修改后更新必要注释、参数说明、XML name 或记录文件。
5. 如果发现 Explorer 结论不完整，先补充记录，不要直接扩大修改范围。

禁止：

- 顺手改其他模块。
- 顺手加调试统计。
- 顺手改参数默认值。
- 顺手删注释分支。
- 顺手重构类结构。
- 顺手改 topic 名、frame_id、QoS、blackboard key。
- 顺手把逻辑改成“理论上更优”的版本。

### 3.3 Auditor Agent：审计与验收

Auditor Agent 不相信 Modifier 的自述，必须重新检查。

职责：

1. 查看 diff。
2. 重新 grep 关键变量、节点、topic、参数。
3. 检查是否越界修改。
4. 检查是否破坏原有优先级和数据流。
5. 运行用户允许范围内的静态检查。
6. 若需要编译构建，必须先向用户确认。
7. 给出 `PASS` 或 `NEEDS_FIX`。

Auditor 发现问题后，不要大范围自行改造。  
应记录问题，并让 Modifier 进行针对性修复。

---

## 4. 改造记录要求

每次非平凡任务必须新建或更新记录文件：

```text
docs/ai_refactor_records/<YYYYMMDD>_<task_name>.md
```

如果用户指定记录文件，则使用用户指定路径。

记录文件必须包含：

```md
# <任务名称> 改造记录

## User Intent

用户原始目标和关键约束。

## Scope

本次允许修改的范围。

## Out of Scope

本次明确不处理的内容。

## Explorer Findings

### Files inspected

### Active logic path

### Data flow

### Risk notes

### Recommended modification boundary

## Modifier Changes

### Files changed

### Key changes

### Behavior preserved

### Behavior intentionally adjusted

### Notes

## Auditor Review

### Checks performed

- [ ] 关键路径检查
- [ ] diff 检查
- [ ] grep 检查
- [ ] XML / launch / yaml 检查
- [ ] 用户允许范围内的测试或静态检查
- [ ] 如需构建，已取得用户明确许可

### Issues found

### Final result

PASS / NEEDS_FIX
```

最终回答用户时，必须提供：

1. 修改摘要。
2. 修改文件列表。
3. 检查结果。
4. 改造记录路径。
5. `PASS` 或 `NEEDS_FIX`。
6. 如果未执行构建，必须说明“未执行构建，因为 AGENTS.md 禁止未授权构建”。

---

## 5. 修改前必须建立任务边界

修改前必须明确并写入记录文件：

### 5.1 本次修改属于哪个模块

可选：

- 行为树 / 自主决策
- planner
- controller
- map / ROGMap
- odom / pointcloud
- launch / communication
- referee / blackboard
- parameter / yaml
- 文档 / 实验记录

### 5.2 本次修改目标是什么

可选：

- bug 修复
- 策略调整
- 性能优化
- 通信链路改造
- 参数暴露
- 日志/统计
- 清理历史残余
- 文档化

### 5.3 是否允许改变运行行为

必须明确：

- 不允许：只修 bug 或清理边界。
- 小幅允许：保持主逻辑，仅修边界条件。
- 允许：用户明确要求策略变化。

### 5.4 是否涉及比赛验证逻辑

如果涉及，必须特别保守，并在记录中写明：

```text
本次修改涉及比赛验证逻辑，采用最小改动策略。
```

---

## 6. 行为树模块修改规范

涉及 `bt_manager`、行为树 XML、blackboard、裁判系统通信时必须遵守：

1. XML active path 是当前逻辑的源头。
2. 注释掉的 XML 不是当前生效逻辑。
3. `ReactiveFallback` 的顺序就是优先级。
4. `ReactiveSequence` 中任何 condition 失败都会阻断后续 action。
5. `ForceSuccess` 包裹的节点一般只用于刷新状态或日志，不应误判为控制分支。
6. Condition 节点可能有 blackboard 写入副作用，必须检查源码。
7. 多棵树共享 blackboard 时，必须检查同一个 key 的多个写入者。
8. 修改关键 blackboard key 时，必须列出读写者。
9. 修改姿态、资源、复活、买弹、买血、冲家策略时，必须确认裁判系统通信位段。
10. XML 中同名节点允许存在但会影响日志和可视化，只有用户要求时才重命名。
11. 不因为命名不完美而改变比赛验证逻辑。
12. 不擅自启用被注释的恢复、追击、远程资源分支。

关键 blackboard key 包括：

```text
tactical_mode
current_mode
nav_goal
desired_stance
current_stance
desired_lifter_pos
cmd_vel
use_gyro_mode
gyro_vel
target_valid
target_pose
outpost_safe_cooldown_active
enemy_outpost_destroyed
```

行为树修改后至少执行用户允许范围内的静态检查，例如：

```bash
grep -RIn "SetTacticalMode\|SetNavMode\|SetCoordinate\|ChangeStance\|CheckTacticalModeCondition" .
grep -RIn "current_mode\|tactical_mode\|nav_goal\|desired_stance\|cmd_vel" .
```

如需构建，必须先请求用户确认。

---

## 7. Planner 修改规范

涉及 MINCO、搜索、轨迹优化、重规划、障碍物响应时必须遵守：

1. 不轻易改变全局框架。
2. 优先小改动修补问题。
3. 不重新启用已废弃 backup trajectory，除非用户明确要求。
4. 不把软约束误改成会频繁停车的硬拒绝。
5. 保持“新最优轨迹可覆盖旧轨迹”的策略。
6. 不随意增加全段 validation 以外的重复检查。
7. 不增加大量新 cost 项，除非用户明确要求。
8. 动态障碍响应优先从权重、吸引项、ESDF 输入质量、重规划时机排查。
9. 性能相关修改必须保留规划耗时和频率可观测性。
10. 不引入会破坏实时性的高频日志。

Planner 修改后至少做静态检查：

```bash
grep -RIn "optimize\|replan\|validation\|poscost\|esdf\|backup\|traj" <planner_package>
```

构建前必须先获得用户许可。

---

## 8. Controller 修改规范

涉及 MPC、controller_server、速度输出、参数切换时必须遵守：

1. 不改变控制接口 topic，除非用户明确要求。
2. 不随意改变控制频率、QoS、timer、callback group。
3. 不让行为树直接长期覆盖 controller 输出，除非是明确的恢复/急停逻辑。
4. 修改参数切换时必须保存原始参数并恢复。
5. 修改小陀螺、yaw opt、隧道控制时，必须检查进入和退出条件。
6. 修改 MPC 求解时，不引入大规模统计或高频日志。
7. 若 controller 参数由行为树动态切换，必须确认退出分支会恢复原参数。

Controller 修改后至少做静态检查：

```bash
grep -RIn "cmd_vel\|FollowPath\|use_small_gyro_mode\|enable_yaw_opt\|timer\|callback" <controller_package>
```

构建前必须先获得用户许可。

---

## 9. ROGMap / 地图修改规范

涉及 ROGMap、ProjectionLayer、ESDF、占据栅格、decay、地形分类时必须遵守：

1. 区分 occupancy、ESDF、projection、active window。
2. 修改 decay 时必须明确 hit/miss/decay 的关系。
3. 动态障碍清除优先检查 decay 与 active 维护，不盲目调 hit/miss。
4. 投影层耗时优化必须先确认瓶颈是遍历、邻域、内存访问还是通信。
5. 地形分类要保持 free/occupied/passable/unknown 语义稳定。
6. 不随意改变 map resolution、frame_id、map_size。
7. 不增加无用调试日志。
8. 参数加载逻辑必须清楚，不保留无意义 fallback，除非用户要求。
9. 修改 decay 时，若用户要求“min 内保持、max 衰减到 free”，不得额外引入多余 decay_time 概念。

ROGMap 修改后至少做静态检查：

```bash
grep -RIn "decay\|hit\|miss\|active\|projection\|esdf\|occupancy" <map_package>
```

构建前必须先获得用户许可。

---

## 10. Odom / PointCloud 修改规范

涉及 Livox、Point-LIO、Batch-LIWO、完整点云、去畸变、intra-process 时必须遵守：

1. 点云链路必须区分原始点云、降采样点云、完整点云。
2. 完整点云输出必须保证时间升序、去畸变补偿、世界系变换。
3. 高速小陀螺场景必须特别注意点云时间戳和去畸变。
4. intra-process 需要 publisher 使用 `std::unique_ptr` 发布才可能发挥零拷贝优势。
5. odom 高频问题不应简单归因于内存拷贝，需检查调度、QoS、队列、发布频率。
6. 不随意提高点云发布频率。
7. 不让 callback 中打印高频详细日志。
8. 修改完整点云输出时，必须确认下游 ROGMap 使用的 topic、frame、stamp 不变。
9. 修改 odom 限频时，必须确认 planner、MPC、TF 的订阅需求。

PointCloud/Odom 修改后至少做静态检查：

```bash
grep -RIn "cloud_registered_full\|aft_mapped_to_init\|unique_ptr\|publish\|deskew\|undistort\|odom" .
```

构建前必须先获得用户许可。

---

## 11. Launch / ROS 2 通信修改规范

涉及 launch、component container、QoS、intra-process 时必须遵守：

1. 确认节点是否在同一个 component container。
2. 开启 intra-process 不等于一定零拷贝，必须检查 unique_ptr 发布和订阅形式。
3. 大点云优先进程内传输，小消息如 odom 更关注调度和频率。
4. 不破坏 Nav2 官方功能。
5. 修改 launch 时必须确认 include 路径和参数路径。
6. 修改 QoS 时必须考虑 sensor_data、volatile、keep_last、reliable/best_effort 的影响。
7. 修改 container 划分时，必须说明哪些节点在同一进程，哪些仍跨进程。
8. 不擅自替换官方 launch，除非用户明确要求。

Launch 修改后至少做静态检查：

```bash
grep -RIn "ComposableNode\|component_container\|use_intra_process_comms\|qos\|parameters\|remappings" launch/ src/
```

构建前必须先获得用户许可。

---

## 12. 参数修改规范

涉及 yaml、declare_parameter、默认值时必须遵守：

1. 不保留无意义 fallback，如果用户明确要求参数必须配置。
2. 参数名要和 yaml 一致。
3. 删除参数时必须删除读取、默认值、文档和 yaml。
4. 修改默认值必须说明原因。
5. 比赛参数改动必须写入记录文件。
6. 不擅自把硬编码改为参数，除非用户明确要求。
7. 不擅自删除现有参数兼容性，除非用户明确要求。

---

## 13. 通信协议 / 裁判系统修改规范

涉及裁判系统、blackboard、姿态、资源请求、sentry_info、sentry_cmd 时必须遵守：

1. 先确认协议位段，再改枚举或打包。
2. 保持旧值兼容，尤其是已有姿态、复活、买弹、买血请求。
3. 修改 enum 时必须同步：
   - blackboard 默认值
   - 回调解析
   - 指令打包
   - 行为树 XML
   - 日志转换
   - 下位机约定
4. 不要只改发送，不改接收。
5. 不要只改接收，不改 blackboard。
6. 不要只改 enum，不改 XML 和打包。
7. 对 bit field 的修改必须写入记录文件。
8. 新规则接入建议分阶段：
   - 先接收并记录
   - 再扩展枚举和打包
   - 最后修改策略使用

---

## 14. 静态检查与命令限制

允许优先运行：

```bash
git status
git diff
grep -RIn "<keyword>" .
rg "<keyword>"
find . -name "*.xml"
python3 - <<'PY'
# 静态解析脚本
PY
```

XML 检查可使用：

```bash
python3 - <<'PY'
import xml.etree.ElementTree as ET
from pathlib import Path
for f in Path(".").rglob("*.xml"):
    try:
        ET.parse(f)
        print("OK", f)
    except Exception as e:
        print("FAIL", f, e)
PY
```

YAML 检查可使用 Python 解析，但不得触发构建或代码生成。

禁止在未获用户允许前运行：

```bash
colcon build
catkin build
catkin_make
cmake
make
ninja
cargo build
cargo check
npm run build
yarn build
```

如果用户明确允许构建，构建后必须记录：

1. 构建命令。
2. 构建是否通过。
3. 失败关键日志。
4. 是否是本次修改导致。

---

## 15. 最终输出格式

每次完成后，向用户输出：

```md
## 修改摘要

## 修改文件

## 行为保持说明

## 有意改变的行为

## 检查结果

## 改造记录

## 未解决问题

## Final Result

PASS / NEEDS_FIX
```

如果没有执行构建，必须明确写：

```text
未执行构建：AGENTS.md 禁止在未获用户明确许可前运行构建命令。
```

---

## 16. 禁止事项汇总

除非用户明确要求，否则禁止：

1. 大规模重构。
2. 删除历史注释分支。
3. 改动多个无关模块。
4. 修改比赛验证策略。
5. 添加复杂新框架。
6. 添加大量 debug 输出。
7. 修改 topic / frame / blackboard key。
8. 修改参数默认值。
9. 修改 launch 组合方式。
10. 把未验证推断写成确定事实。
11. 未获许可执行构建。
12. 以“清理”为名删除仍可能复用的比赛逻辑。
13. 将单模块任务扩展为全仓库重构。
14. 在未记录的情况下修改行为优先级。

---

## 17. 推荐阶段化工作方式

对于复杂任务，按阶段拆分：

```text
Step 1：职责边界和残余清理
Step 2：通信链路或 blackboard 字段
Step 3：枚举 / 协议 / 参数接入
Step 4：行为树策略接入
Step 5：恢复 / 异常处理
Step 6：审计和清理
```

每一步单独记录、单独验收，不要一次完成所有阶段。

---

## 18. 当前项目已知偏好

用户偏好：

1. 最小改动。
2. 少新增变量、函数、类。
3. 不整体重构。
4. 不增加无用调试。
5. 先分析问题，再列约束，再给方案，再给风险和验收。
6. 复杂任务使用 Explorer → Modifier → Auditor 流程。
7. 构建必须先询问用户。
8. 输出应明确哪些行为保持不变、哪些行为被有意改变。
9. 行为树以当前比赛验证 XML 为准，不按理论重新设计。
10. 对 planner、controller、ROGMap、Point-LIO 的改造必须保持实时性和通信链路稳定。