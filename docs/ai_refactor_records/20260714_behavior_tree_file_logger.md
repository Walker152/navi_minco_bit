# 行为树终端文件 Logger 改造记录

## User Intent

为当前生效的行为树增加 logger，将通用节点状态变化以及现有 `std::cout` 中的
`ACTIVE/INACTIVE` 等终端输出保存为工作空间 `docs` 目录下的 `.log` 文件。

## Scope

- `bt_manager` 行为树加载与日志生命周期。
- 当前实际加载的 resource、tactical、nav、stance、gimbal 五棵树。
- `std::cout` 终端输出的 tee 与日志 ANSI 控制码清理。

## Out of Scope

- 不捕获 `std::cerr`、C `printf` 或 ROS 2 `RCLCPP_*` 输出。
- 不修改行为树 XML、节点优先级、topic、QoS、blackboard key 或比赛参数。
- 不接入当前未加载的 `recovery_tree.xml`。

## Explorer Findings

### Files inspected

- `src/decision/bt_manager/src/bt_manager.cpp`
- `src/decision/bt_manager/include/bt_manager/bt_manager.hpp`
- `src/decision/bt_manager/src/main.cpp`
- `src/decision/bt_manager/include/bt_manager/utils/log.hpp`
- `src/decision/bt_manager/launch/bt_manager.launch.py`
- BehaviorTree.CPP v3 `abstract_logger.h`

### Active logic path

`SentryBTManager::loadTrees()` 加载 resource、nav、gimbal、stance、tactical 五棵树；
`run()` 按既有顺序逐棵 tick。`recovery_tree.xml` 当前未被管理器加载。

### Data flow

BehaviorTree.CPP 状态信号经 `BehaviorTreeLogger` 输出到 `std::cout`；
`TeeStreamBuffer` 保留原终端输出，同时将去除 ANSI 控制序列的文本写入日志文件。

### Risk notes

- 本次修改涉及比赛验证逻辑外围的可观测性，采用最小改动策略。
- 文件路径相对进程启动目录解析；从工作空间启动时写入
  `docs/bt_logs/bt_<时间戳>.log`。
- 状态变化日志会增加磁盘写入量，但不改变 tick 顺序或节点返回值。

### Recommended modification boundary

仅新增 logger/tee 类，在树加载完成后订阅状态，并在 `main` 生命周期内安装输出 tee。

## Modifier Changes

### Files changed

- `src/decision/bt_manager/include/bt_manager/behavior_tree_logger.hpp`
- `src/decision/bt_manager/src/behavior_tree_logger.cpp`
- `src/decision/bt_manager/include/bt_manager/bt_manager.hpp`
- `src/decision/bt_manager/src/bt_manager.cpp`
- `src/decision/bt_manager/src/main.cpp`
- `src/decision/bt_manager/src/test/test_behavior_tree_logger_contract.py`

### Key changes

- 新增 BehaviorTree.CPP 状态变化 logger，并为五棵生效树增加树标签。
- 新增线程安全的 `std::cout` tee，日志文件去除 ANSI 颜色控制码。
- 日志默认写入 `docs/bt_logs/bt_<时间戳>.log`。
- 将 `bt_debug_logs` 直接运行默认值设为 `true`，与现有 launch 默认值保持一致，
  使既有 `ACTIVE/INACTIVE` 输出进入 tee 日志。

### Behavior preserved

- 行为树 XML、加载文件、tick 顺序、节点逻辑和条件优先级不变。
- `std::cout` 仍输出到原终端，颜色显示不变。
- 原有可选 `bt_debug_log_to_file` 机制未删除。

### Behavior intentionally adjusted

- 启动 `bt_manager_node` 后自动创建时间戳日志并记录节点状态变化和 `std::cout`。
- 不通过 launch 直接运行时，`bt_debug_logs` 默认由 false 调整为 true。

### Notes

持久日志不包含 ANSI 颜色码，便于检索和审计。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查
- [x] grep 检查
- [x] XML / launch / yaml 检查
- [x] 用户允许范围内的测试或静态检查
- [ ] 如需构建，已取得用户明确许可

执行结果：

- 源码契约测试：2 passed。
- Python 测试文件语法检查：通过。
- 六个行为树 XML 静态解析：通过。
- `git diff --check`：通过。
- 未执行构建：AGENTS.md 禁止在未获用户明确许可前运行构建命令。

### Issues found

未发现越界修改。由于未获构建许可，尚未完成 C++ 编译与运行时日志落盘验证。

### Final result

PASS
