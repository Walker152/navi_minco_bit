# MINCO collision_dist 参数拆分改造记录

## User Intent

将 MINCO planner 中 validation 碰撞检查使用的距离阈值与优化器的距离阈值分开，新参数命名为 `collision_dist`。

## Scope

- MINCO planner 参数加载与 `TrajectorySafetyChecker` 配置。
- `sentry1.yaml` 中的 MINCO planner 参数。
- MINCO planner 参数文档。

## Out of Scope

- 不修改 ESDF 查询、碰撞检查采样周期或判定方式。
- 不修改 MINCO 优化代价、权重或优化框架。
- 不调整当前比赛参数数值。

## Explorer Findings

### Files inspected

- `src/navigation/minco_planner/src/minco_core/minco_planner.cpp`
- `src/navigation/minco_planner/src/minco_core/components/trajectory_safety_checker.cpp`
- `src/navigation/minco_planner/include/minco_core/components/trajectory_safety_checker.hpp`
- `src/navigation/minco_planner/include/traj_opt/minco_optimizer.hpp`
- `src/navigation/minco_planner/src/traj_opt/minco_optimizer.cpp`
- `src/navigation/navi2_bringup/params/sentry1.yaml`
- `src/navigation/minco_planner/README.md`

### Active logic path

`minco_optimizer.safe_dist` 原本同时写入优化器 `magnitudeBounds(0)` 并传入 `TrajectorySafetyChecker::configure()`。优化器使用前者计算 ESDF 位置惩罚，validation 和运行期 safety timer 通过后者判断轨迹是否安全。

### Data flow

- `safe_dist` → `MincoOptimizer::Config::safe_dist` → `magnitudeBounds(0)` → 优化位置惩罚。
- `collision_dist` → `TrajectorySafetyChecker::configure()` → `safe_dist_` → validation/运行期碰撞检查。

### Risk notes

本次修改涉及比赛验证的 planner 逻辑，采用最小改动策略。若两个阈值调整不当，可能出现优化器允许的轨迹被 validation 拒绝，或 validation 安全余量不足。

### Recommended modification boundary

仅拆分参数加载和 safety checker 入参，不改动两条现有检查/优化路径。

## Modifier Changes

### Files changed

- `src/navigation/minco_planner/src/minco_core/minco_planner.cpp`
- `src/navigation/navi2_bringup/params/sentry1.yaml`
- `src/navigation/minco_planner/README.md`
- `docs/ai_refactor_records/20260713_minco_collision_dist.md`

### Key changes

- 新增 `minco_optimizer.collision_dist`。
- `TrajectorySafetyChecker` 改用 `collision_dist`。
- `safe_dist` 保留为 MINCO 优化距离惩罚阈值。

### Behavior preserved

- `collision_dist` 未显式配置时默认继承已加载的 `safe_dist`，保持旧配置行为。
- `sentry1.yaml` 中两个阈值均为 `0.25`，不改变当前运行判定。
- topic、frame、QoS、timer、采样周期和 ESDF 查询方式保持不变。

### Behavior intentionally adjusted

validation/运行期碰撞检查阈值现在可通过 `collision_dist` 独立于优化阈值调节。

### Notes

未引入新类、新函数或无关重构。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查
- [x] grep 检查
- [x] XML / launch / yaml 检查（本次无 XML/launch 修改，YAML 已解析通过）
- [x] 用户允许范围内的测试或静态检查
- [ ] 如需构建，已取得用户明确许可（未执行构建）

### Issues found

未发现越界修改或参数交叉使用。`safe_dist` 仅流入优化器的 `magnitudeBounds(0)`，`collision_dist` 仅流入 `TrajectorySafetyChecker`。`git diff --check` 和 YAML 静态解析通过。

### Final result

PASS
