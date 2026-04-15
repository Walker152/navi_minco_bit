1. 任务背景 (Task Context)
本项目为 RoboMaster 2026 赛季哨兵机器人（Sentry）的智能决策系统。针对赛场高度动态的复杂环境，系统采用 正交化多树并行架构 (Orthogonal Multi-Tree Architecture)。
作为 AI 编程助手，你的任务是基于现有的架构基础，完成完整的行为树 XML 搭建、C++ 节点开发、静态检查及自动化测试节点编写。
2. 架构强制约束 (Strict Architectural Constraints)
在编写任何代码之前，必须严格遵守以下工程规范，绝不允许违背：
1. 通信隔离原则：所有的 ROS 2 数据收发仅限在 ros_interface.cpp 中进行。行为树内部的 Action/Condition 节点绝对禁止包含任何 ROS pub/sub 逻辑。BT 节点只能通过单例黑板/数据总线 (blackboard.hpp) 获取数据或下发指令。
2. 语义化导航原则：禁止在行为树中硬编码 (Hardcode) 绝对坐标。所有涉及点位的操作，必须调用 nav_zone.hpp 和 area.hpp 中的语义地图数据结构（如 Zone::BASE, Zone::ENEMY_OUTPOST）。
3. 数据驱动战术：tactical_tree 作为大脑，只负责修改黑板中的 tactical_mode 等宏观战略字段。nav_tree、gimbal_tree 和 stance_tree 必须优先读取该字段，通过多态/查表来改变自身的参数（如巡逻点、巡检范围），而不是在 XML 中写复杂的 if-else。
3. 参考文件目录 (Workspace Directory)
请基于以下目录结构进行代码生成与修改：
- BT 功能包: src/decision/bt_manager/
  - tree/: 存放所有 .xml 文件 (tactical_tree.xml, nav_tree.xml, gimbal_tree.xml, stance_tree.xml)。
  - include/bt_manager/condition/ & src/condition/: 条件节点。
  - include/bt_manager/action/ & src/action/: 动作节点。
  - include/bt_manager/utils/: 包含 nav_zone.hpp, area.hpp (语义地图/区域定义)。
  - include/bt_manager/blackboard.hpp: 数据初始化与接口总线。
  - src/ros_interface.cpp: 唯一通信出入口。
  - src/bt_manager.cpp: 行为树工厂预加载与总控。
- 通信接口包: src/ros_interfaces/new_msgs/ (存放交互 msg)。
- 下位机通信: behaviorsend (发送给下位机的协议结构)。
4. 核心业务逻辑需求 (Behavior Tree Structures)
4.1 战术树 (Tactical Tree) - 宏观决策
- 高优逻辑 (ATTACK)：IF (敌方前哨站被摧毁 AND 己方任意能量机关激活 AND 己方基地 HP > 1000) THEN tactical_mode = ATTACK.
- 次优逻辑 (DEFEND)：IF (己方基地 HP < 1000) THEN tactical_mode = DEFEND.
- 兜底逻辑 (NORMAL)：ELSE tactical_mode = NORMAL.
4.2 导航树 (Nav Tree) - 底盘控制
- Priority 1: 绝对生存 (回血补弹)
  - IF (HP < 30% OR Ammo < 100) -> 发布回家点位，切换至“回家地图”。
  - 下台阶处理：前往回家途中的“下台阶区域”。IF (到达该区域 AND 区域下方无队友) -> 发布固定速度下台阶指令(CmdVel)，直到脱离区域。
  - 补给策略：IF (Ammo < 100) -> 触发购买弹药逻辑，购买量 Y 为单调递增累积计算。
  - 退出补给：IF (HP > 80% AND Ammo > 100) -> 退出生存模式，出门。
- Priority 2: 特殊事件响应
  - NORMAL 战术：IF (敌方前哨站未摧毁) -> 优先前往敌方前哨站。
  - DEFEND 战术：IF (己方堡垒无人占领) -> 占领己方堡垒。
  - ATTACK 战术：IF (敌方基地 HP < 1000) -> 占领敌方堡垒。
- Priority 3: 追踪与巡逻 (常规战术)
  - 追踪判定：读取追击优先级，读取视觉坐标并 TF 转换为 Map 系，判定区域可行性。IF (目标在可行区内) -> 发布导航点进行索敌（必须加入目标点防抖滤波逻辑避免底盘抽搐）。
  - 常规巡逻：IF (无追击目标) -> 读取当前 tactical_mode 对应的语义巡逻点进行巡逻。
- 全局约束：非 Priority 1 状态必须使用正常地图；非回血状态移动时，必须并发检查 过洞(Tunnel) 条件，若过洞需触发升降云台动作。
4.3 云台树 (Gimbal Tree) - 视觉索敌
- Priority 1: 强锁敌（由视觉提供目标）。
- Priority 2: 战术/区域巡检（动态扫描角度）。
  - 特殊事件响应（同 Nav Tree）：如在前哨站附近 -> 抬高 Pitch，设定特定的 Yaw 扫描范围。
  - 普通模式：基于当前底盘所在的语义区域决定姿态。
    - IF (高地区域) -> 设定高地视角范围。
    - IF (己方防守区) -> 设定防守预警角度。
    - IF (敌方防守区) -> 设定压制预警角度。
    - ELSE -> 360度常规巡检。
4.4 姿态树 (Stance Tree) - 硬件构型
- 硬性冷却/惩罚规则：
  1. 两次姿态切换之间必须具有 5秒 CD。
  2. 同一姿态持续时间超过 3分钟 会受规则惩罚。IF (姿态维持达 3 分钟) -> 必须触发一次异构姿态切换，然后再迅速切回原姿态刷新时间。
- 姿态调度逻辑：
  - 默认：移动姿态 (MOVE)。
  - 特定事件：IF (响应前哨站 AND 到达前哨站区域) OR (追踪到敌人) -> 切换攻击姿态 (ATTACK)。
  - 特定事件：IF (在堡垒区域盘踞) -> 切换防御姿态 (DEFEND，通常带小陀螺 Spin)。
5. 任务执行流程 (Task Execution Workflow)
请严格按照以下步骤执行，每完成一步请输出确认信息：
- [ ] Step 1: XML 构建: 根据上述逻辑，编写 4 棵树的 XML 结构。要求使用 ReactiveFallback 等高级控制流管理优先级，保持树的扁平化。
- [ ] Step 2: C++ 节点实现: 编写对应的 .cpp 和 .hpp 文件。要求从单例/黑板提取数据，完成逻辑判定。
- [ ] Step 3: 静态检查: 模拟静态检查。确认黑板变量名 (key) 在 XML 和 C++ 中完全匹配；确认 C++ 节点中无 while(true) 或 sleep 导致的 BT Tick 线程阻塞。
- [ ] Step 4: 代码规范: 应用 C++17 规范，假设使用 clang_format 进行风格格式化（输出的代码需符合 Google/LLVM 标准排版）。
- [ ] Step 5: 综合虚拟测试构建:
  - 依赖启动说明：测试环境必须先行启动导航堆栈，可通过执行 navi2_bringup 中的 navigation2.launch.py 或直接运行 start.bash。
  - 虚拟测试声明 (重要)：此次测试为电脑纯虚拟测试。提供的 TF 仅用于“欺骗”并维持 Nav2 的正常运行生命周期。实际上车子并不会发生任何物理移动。在测试过程中，绝对不要因为 Nav2 未输出真实运动而认为发生错误。测试核心且唯一关注点是行为树的状态流转是否正确。
  - 日志输出规范 (重要)：为避免高频 Tick 导致终端刷屏，行为树内部任何节点的变化日志，**必须遵循“状态发生改变时才打印 (Print on State Change Only)”**的原则。
  - 测试用例编写：在 src/test/ 下编写 sentry_bt_test_node.cpp。模拟 ROS 边界数据注入，验证全优先级拦截与退出的预期行为。
6. 预期产出 (Expected Deliverables)
完成任务后，你需要交付：
1. 完整的 XML 树文件代码 (tactical_tree.xml, nav_tree.xml, gimbal_tree.xml, stance_tree.xml)。
2. 核心新建/修改的 C++ 节点代码（重点展示数据解耦、状态机切换、防抖逻辑、3分钟超时刷新逻辑、状态变更触发式日志打印逻辑）。
3. 综合测试节点代码 (sentry_bt_test_node.cpp)。
4. 测试演练报告 (Test Report)：包含具体的测试用例场景验证，明确说明“哪些输入数据发生了变化”、“哪些状态变量从什么变成了什么”。例如：“血量降至25%” -> 预期结果：NavTree 切换回家目标，StanceTree 切换为 Move 姿态 -> 终端输出的状态变更记录。