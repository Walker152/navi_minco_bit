# Codex 任务：修复 MincoPlanner 双模式架构剩余逻辑漏洞，不修改速度/yaw 处理

## 0. 背景

当前 `MincoPlanner` 已经完成 `PRIORMAP / EXPLORATION` 双模式架构改造，整体方向正确：

```text
PRIORMAP:
  planning_frame = map
  output_frame   = map
  global search  = Nav2 costmap / prior map
  ROGMap         = dynamic gradient + safety grid only

EXPLORATION:
  planning_frame = camera_init
  output_frame   = camera_init
  global search  = ROGMap reachable boundary search
  ROGMap         = search map + optimization query + safety grid
```

本轮任务不是重写架构，不要修改算法内部原理，只修复当前实现中仍然存在的接口、坐标、模式语义漏洞。

特别说明：**不要处理速度 / yaw 到 planning_frame 的转换问题。**
上一轮检查中的第 5 点本轮不做，保持当前速度和 yaw 逻辑不动。

---

## 1. 禁止事项

本轮禁止做以下事情：

1. 不要修改 Minco 优化器核心数学。
2. 不要修改 A*/Smac 搜索算法原理。
3. 不要修改 FSM 主流程。
4. 不要改变 recovery / safety timer 状态机逻辑。
5. 不要重构成复杂框架。
6. 不要修改当前速度和 yaw 获取逻辑。
7. 不要改变 `PRIORMAP` 下 `/opt_path` 输出为 `map` 的语义。
8. 不要让 `PRIORMAP` 下 ROGMap 参与全局搜索。
9. 不要让 `PRIORMAP` 下 ROGMap 参与路径稀疏 / 控制点选择。
10. 不要让 `EXPLORATION` 依赖 map / PGM / static ESDF。

---

## 2. 修复目标总览

需要修复以下 7 类问题：

```text
1. PRIORMAP 下路径稀疏仍然使用 ROGMap。
2. clipLocalPathByRogBoundary() 返回值被忽略，首点越界时没有正确裁剪。
3. createPlan() 中 TF 失败时不能伪装 frame。
4. PRIORMAP 下 odom fallback 对空 frame 的处理不安全。
5. EXPLORATION 下 goal 在窗口内但不可达时，应 fallback 到 reachable boundary search。
6. setMap() 不能绕过 mode wrapper。
7. mode / frame / 双模式核心参数不允许 hot reload。
```

注意：上一轮检查中的“速度/yaw 转 planning_frame”问题本轮不处理。

---

## 3. 修复 1：新增 sparsify_query_，PRIORMAP 稀疏不能用 ROGMap

### 当前问题

`ReplanLocal()` 中 `getSparseWaypoints()` 的 line-free 判断仍然使用：

```cpp
isLineFree(this->dynamic_query_, a, b)
```

而 `PRIORMAP` 下 `dynamic_query_` 是 ROGMap frame-aware wrapper。
这会导致 ROGMap 参与路径稀疏 / 控制点选择，违反设计要求。

### 要求

新增成员变量：

```cpp
std::shared_ptr<rog_map::MapQueryInterface> sparsify_query_;
```

在 mode init 中设置：

```cpp
PRIORMAP:
  sparsify_query_ = global_search_query_;   // Nav2 costmap / prior map query

EXPLORATION:
  sparsify_query_ = dynamic_query_;         // ROGMap query
```

然后将 `ReplanLocal()` 中路径稀疏使用的查询对象改为：

```cpp
isLineFree(this->sparsify_query_, a, b)
```

而不是 `dynamic_query_`。

### 语义要求

`PRIORMAP` 下：

```text
Nav2/map:
  global search
  path sparsify
  control point selection

ROGMap:
  optimization gradient
  safety grid check
  optional seed boundary clipping only
```

`EXPLORATION` 下：

```text
ROGMap:
  global search
  path sparsify
  optimization query
  safety grid check
```

---

## 4. 修复 2：clipLocalPathByRogBoundary() 必须正确返回并被调用方检查

### 当前问题

`ReplanLocal()` 中调用：

```cpp
clipLocalPathByRogBoundary(dense_local_path);
```

但没有检查返回值。

同时，如果 path 第一个点就不在 ROGMap 边界内，函数可能直接 break，但没有清空 path，导致后续仍拿越界 path 继续优化。

### 要求

修改 `clipLocalPathByRogBoundary()`：

1. 返回 `bool` 表示裁剪后路径是否仍有效。
2. 如果输入 path 为空，返回 false。
3. 如果首点不在 ROGMap 有效边界内：

   * `path.clear();`
   * 打 WARN_THROTTLE；
   * 返回 false。
4. 如果裁剪后 path size < 2：

   * 返回 false。
5. 如果裁剪成功且至少保留 2 个点：

   * 返回 true。

示意逻辑：

```cpp
bool MincoPlanner::clipLocalPathByRogBoundary(std::vector<Eigen::Vector3d> & path)
{
  if (path.empty()) {
    return false;
  }

  std::vector<Eigen::Vector3d> clipped;
  clipped.reserve(path.size());

  for (size_t i = 0; i < path.size(); ++i) {
    if (!isInsideRogBoundaryWithMargin(path[i])) {
      if (i == 0) {
        path.clear();
        return false;
      }
      break;
    }
    clipped.push_back(path[i]);
  }

  path.swap(clipped);
  return path.size() >= 2;
}
```

### ReplanLocal 调用要求

`ReplanLocal()` 必须检查：

```cpp
const bool clip_ok = clipLocalPathByRogBoundary(dense_local_path);
if (clip_required && (!clip_ok || dense_local_path.size() < 2)) {
  RCLCPP_WARN_THROTTLE(...);
  return false;
}
```

其中：

```text
PRIORMAP:
  clip_required = priormap_clip_seed_by_rog_boundary_

EXPLORATION:
  clip_required = true
```

---

## 5. 修复 3：createPlan() 中 transform 失败不能伪装 frame

### 当前问题

当前逻辑可能出现：

```cpp
normalized_goal = goal;
normalized_goal.header.frame_id = output_frame_;
```

这会导致坐标值没有经过 TF 变换，但 header 被强行改成 output frame，属于严重坐标语义错误。

### 要求

在 `createPlan()` 中：

1. goal 需要归一到 `output_frame_` 或 `planning_frame_`。
2. 如果 transform 失败：

   * 不允许修改 header 后继续使用；
   * 不允许设置 `pending_goal_`；
   * 返回空 path 或 minimal failure path；
   * 打 ERROR 日志，明确说明 transform 失败。
3. 如果 goal header 为空：

   * `PRIORMAP` 下默认按 `map_frame_` 解释；
   * `EXPLORATION` 下默认按 `rog_frame_ / camera_init` 解释；
   * 打 WARN。

建议逻辑：

```cpp
geometry_msgs::msg::PoseStamped normalized_goal;
if (!normalizePoseToPlanningFrame(goal, normalized_goal)) {
  RCLCPP_ERROR(logger_, "[MincoPlanner] Failed to normalize goal pose to planning frame; reject pending goal.");
  std::lock_guard<std::mutex> lk(goal_mutex_);
  has_pending_goal_ = false;
  return nav_msgs::msg::Path{};
}
```

不要出现“TF 失败后强行改 header”的代码。

---

## 6. 修复 4：PRIORMAP odom fallback 空 frame 不能当成 map

### 当前问题

`getRobotPose()` 在 `PRIORMAP` fallback 中，如果 odom frame 为空，可能把 odom 直接当成 `planning_frame_ = map`。

但当前 odom 实际语义是 `camera_init`。
所以 PRIORMAP 下空 odom frame 应默认按 `rog_frame_ / camera_init` 处理，然后 TF 到 map。

### 要求

修改 `getRobotPose()` 的 PRIORMAP fallback：

```text
PRIORMAP:
  1. 优先 costmap_ros_->getRobotPose(pose)，保持原 Nav2 语义。
  2. 若失败，读取 latest_odom_。
  3. 若 latest_odom_.header.frame_id 为空，则按 rog_frame_ 处理，并 WARN。
  4. 如果 odom frame 已经是 map_frame_，可直接返回。
  5. 如果 odom frame 是 rog_frame_ / camera_init，则必须 TF 到 map_frame_。
  6. 如果 TF 不可用或失败，返回 false。
```

禁止逻辑：

```cpp
if (odom_pose.header.frame_id.empty()) {
  odom_pose.header.frame_id = planning_frame_;  // 禁止在 PRIORMAP 这样做
}
```

EXPLORATION 下可以保留：

```text
odom frame 为空时按 camera_init 处理。
```

---

## 7. 修复 5：EXPLORATION goal 在窗口内但不可达时 fallback 到边界搜索

### 当前问题

`planGlobalPathExploration()` 中，如果 goal 在 ROGMap 内且 cell 可通行，会直接尝试搜索到 goal。
如果搜索失败，当前可能直接返回 false。

但 goal cell 可通行不代表从当前位置可达，可能被障碍物隔开。

### 要求

修改逻辑：

```cpp
if (goal_traversable) {
  if (makePlanOnQuery(... goal ...)) {
    return true;
  }

  RCLCPP_WARN(
    logger_,
    "[MincoPlanner] Exploration goal is inside ROGMap and traversable but unreachable; fallback to reachable boundary search.");
}

// 继续执行 reachable boundary search
return planExplorationToReachableBoundary(start, goal);
```

边界搜索仍必须满足：

```text
从 start 出发搜索；
边界候选必须来自 start 可达连通域；
不能直接投影 goal 到边界后假设可达；
goal 方向只能作为候选评分或启发项。
```

---

## 8. 修复 6：setMap() 不能绕过 mode wrapper

### 当前问题

如果 `setMap(raw_rog_map)` 直接执行：

```cpp
dynamic_query_ = map;
minco_optimizer_->setMap(dynamic_query_);
corridor_gen_->setMap(dynamic_query_);
```

那么 PRIORMAP 下会绕过 `FrameAwareRogQuery`，导致 map 系轨迹点再次直接查 camera_init ROGMap。

### 要求

修改 `setMap()`，使其不会破坏 mode wrapper。

推荐方式：

```cpp
void MincoPlanner::setMap(const std::shared_ptr<rog_map::MapQueryInterface> & raw_rog_map)
{
  rog_query_raw_ = raw_rog_map;
  rebuildModeDependentQueries();
}
```

其中 `rebuildModeDependentQueries()` 根据当前 mode 重新设置：

```text
PRIORMAP:
  dynamic_query_ = FrameAwareRogQuery(raw_rog_map, map_frame_, rog_frame_, tf_)
  minco_optimizer_->setMap(dynamic_query_)
  corridor_gen_->setMap(dynamic_query_)
  safety check uses dynamic_query_

EXPLORATION:
  dynamic_query_ = raw_rog_map
  global_search_query_ = raw_rog_map
  sparsify_query_ = raw_rog_map
  minco_optimizer_->setMap(dynamic_query_)
  corridor_gen_->setMap(dynamic_query_)
```

如果当前初始化顺序不方便，也至少保证：

```text
PRIORMAP 下 setMap() 不会把 dynamic_query_ 覆盖成 raw ROGMap。
```

并加日志：

```cpp
RCLCPP_INFO(logger_, "[MincoPlanner] Rebuilt mode-dependent map queries after raw ROGMap update.");
```

---

## 9. 修复 7：禁止 mode / frame / 双模式核心参数 hot reload

### 当前问题

当前可能只禁止了 `planner_mode` 热更新，但没有禁止：

```text
frames.map_frame
frames.rog_frame
priormap.*
exploration.*
```

这些参数都是 configure-time 决定的。如果运行时变化，内部变量不会自动一致，容易造成调试混乱。

### 要求

在 `onSetParameters()` 中拒绝以下参数运行时修改：

```text
MincoPlanner.planner_mode
MincoPlanner.frames.map_frame
MincoPlanner.frames.rog_frame

MincoPlanner.priormap.use_nav2_global_search
MincoPlanner.priormap.clip_seed_by_rog_boundary
MincoPlanner.priormap.rog_boundary_margin
MincoPlanner.priormap.rog_boundary_sample_step

MincoPlanner.exploration.boundary_margin
MincoPlanner.exploration.boundary_sample_step
MincoPlanner.exploration.unknown_as_occupied
MincoPlanner.exploration.prefer_goal_direction
```

也可以用后缀匹配，只要不要误伤普通优化器参数。

返回：

```cpp
rcl_interfaces::msg::SetParametersResult result;
result.successful = false;
result.reason = "Planner mode/frame parameters are configure-time only; restart planner_server to apply.";
return result;
```

注意不要禁止已有可动态调整的优化器权重、速度、加速度等参数，除非当前代码本来就是这样设计的。

---

## 10. 需要检查的旧错误是否消失

修复后检查以下问题：

### PRIORMAP

必须满足：

```text
1. start/goal 全局搜索不进入 raw ROGMap。
2. 路径稀疏不使用 ROGMap。
3. 控制点选择不因 ROGMap 改变。
4. ROGMap 只用于：
   - dynamic_query_ 优化梯度；
   - safety check 栅格/ESDF；
   - 可选 seed boundary clipping。
5. /opt_path.header.frame_id = map。
6. TF 失败时不会把 camera_init 坐标伪装成 map。
```

### EXPLORATION

必须满足：

```text
1. getRobotPose 直接使用 camera_init odom。
2. global path 在 ROGMap / camera_init 中生成。
3. goal 在窗口内但不可达时，会 fallback 到 reachable boundary search。
4. reachable boundary 是从当前位置搜索得到，不是直接投影。
5. 不加载 static ESDF。
6. /opt_path.header.frame_id = camera_init。
```

---

## 11. 编译与静态验证

完成后执行：

```bash
colcon build --symlink-install
```

并输出一个 `planner_mode_fix_validation.md`，包含：

```text
1. 修改文件列表。
2. 每个修复点对应的代码位置。
3. PRIORMAP 下：
   - global_search_query_ 是 Nav2 costmap；
   - sparsify_query_ 是 Nav2 costmap；
   - dynamic_query_ 是 FrameAwareRogQuery；
   - /opt_path frame 是 map。
4. EXPLORATION 下：
   - global_search_query_ 是 ROGMap；
   - sparsify_query_ 是 ROGMap；
   - dynamic_query_ 是 ROGMap；
   - /opt_path frame 是 camera_init。
5. 编译结果。
6. 仍需实机验证的项目。
```

---

## 12. 最终验收标准

本轮修复后，必须能明确保证：

```text
PRIORMAP:
  旧 Nav2 + map + BT + controller 语义保持不变。
  ROGMap 不再影响全局搜索、路径稀疏、控制点选择。
  ROGMap 仅提供优化梯度、安全检测栅格，以及可选边界裁剪。

EXPLORATION:
  全流程在 camera_init 下运行。
  ROGMap 搜索到可达边界，而不是直接投影不可达边界点。
```

再次强调：**不要修改速度/yaw 当前处理逻辑。**
