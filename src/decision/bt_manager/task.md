RoboMaster 2026 Sentry BT - 区域系统与动态追踪重构任务 (Area & Pursuit Refactor)

1. 任务背景 (Task Context)

当前哨兵系统在空间判断上主要依赖正方形边界框 (Area_Square)，无法适应 RoboMaster 赛场真实的不规则梯形/多边形区域（如高地增益区、前哨站增益区等）。
同时，哨兵的“索敌追踪区域 (target_feasible_zone)”目前是硬编码的静态区域。实际上，追踪的可行区域必须由当前的宏观战术 (tactical_mode) 动态决定组合（例如：进攻态允许跨越半场追踪，防御态仅限己方半场）。

2. 架构强制约束 (Strict Architectural Constraints)

零内存碎片原则 (Stack Allocation Only)：不规则多边形类 AreaPolygon 必须被设计为模板类，通过模板参数在编译期预先决定多边形的顶点个数，使用 std::array 而非 std::vector，避免在运行时的高频 Tick 中产生堆内存分配。

阵营对称性声明：对于赛场上敌我双方对称的区域（隧道、高地增益区、基地增益区），必须在 area.hpp 中成对定义（如 own_tunnel_zone, enemy_tunnel_zone）。

动态追踪隔离：绝对禁止在 XML 中为不同的战术写不同的追踪分支。必须在底层的 CheckTargetLocked 或 SetTargetCoordinate 的 C++ 节点中，读取黑板上的 tactical_mode，然后在 C++ 代码里用逻辑运算组合出当前允许的追踪区域。

3. 核心功能实现要求 (Core Implementations)

3.1 核心数据结构: 模板化多边形区域类 (AreaPolygon)

在 bt_manager/utils/nav_zone.hpp 中新增 AreaPolygon 模板结构体：

类定义：template <std::size_t N> struct AreaPolygon。

成员变量：std::array<Point2D, N> vertices; （接受按顺时针或逆时针顺序排列的顶点）。

判断函数：实现 bool contains(const Point2D& pt) const 方法。

算法参考 (射线法)：从目标点向右发射一条水平射线。必须先进行“上下”高度包围框边界的快速判定，再进行“左右”X轴截距的严格计算。奇数在内，偶数在外。

3.2 赛场语义地图补全 (area.hpp)

利用 AreaPolygon<N> 或原有的 Area_Square，在 area.hpp 中补充以下赛场关键区域（坐标可暂用占位符，后续根据官方场地 CAD 填入）：

中央高地增益区 (Central Highland Buff Zones)：敌我各一个（通常是四边形梯形，使用 AreaPolygon<4>）。

隧道区 (Tunnel Zones)：补齐上/下或敌/我两侧的隧道区（own_tunnel_zone, enemy_tunnel_zone）。

基地/前哨增益区 (Base/Outpost Buff Zones)：敌我各一个。

半场区域 (Half Courts)：own_half_court, enemy_half_court，用于战术组合。

3.3 动态战术追踪区域判定 (SetTargetCoordinate.cpp)

废弃原来单一静态的 target_feasible_zone。在追踪动作/条件节点中，引入战术区域组合逻辑：

读取 tactical_mode：

IF (mode == DEFEND)：可追踪区域 = own_half_court + own_highland_zone。若敌人在该区域外，判定为不可追踪（停止追击）。

IF (mode == NORMAL)：可追踪区域 = own_half_court + 全图高地区域 + 敌方前哨站区。

IF (mode == ATTACK)：可追踪区域 = 全图，但排除 enemy_base_penalty_zone (敌方基地绝对禁区，防止犯规)。

如果视觉识别的敌方坐标转换到 map 系后，不在上述动态生成的组合区域内，则清空追踪目标，返回 FAILURE。

4. 任务执行流程 (Task Execution Workflow)

请按以下步骤逐一完成代码的重构与输出：

[x] Step 1: 开发 AreaPolygon<N>：在 nav_zone.hpp 中实现基于定长模板的多边形射线法判断逻辑，确保能够处理边缘共线等特殊数学情况。

[x] Step 2: 填充 area.hpp：使用新的模板类实例化任务中提及的增益区、隧道、半场等敌我对称区域（如 inline AreaPolygon<4> own_highland_buff_zone{...};）。

[x] Step 3: 重写追踪判定逻辑：定位到判断目标可行性的 C++ 节点（如 SetTargetCoordinate.cpp），提取黑板上的 tactical_mode，用 C++ 代码编写动态追踪区域的集合运算逻辑。

[x] Step 4: 静态审查验证：检查是否所有用到静态 target_feasible_zone 的旧代码都已被新逻辑平替，确保不会引发未定义的编译错误。

💡 Copilot 实施提示 (Copilot Instruction Guidelines)

请在生成 AreaPolygon 时严格参考以下 C++17 模板与射线法（Ray Casting）的鲁棒实现：

template <std::size_t N>
struct AreaPolygon {
    std::array<Point2D, N> vertices;

    // 默认与列表初始化构造函数
    AreaPolygon() = default;
    template <typename... Args>
    AreaPolygon(Args... args) : vertices{args...} {
        static_assert(sizeof...(args) == N, "Number of arguments must match template parameter N");
    }

    bool contains(const Point2D& pt) const {
        bool inside = false;
        for (std::size_t i = 0, j = N - 1; i < N; j = i++) {
            // 核心算法：
            // 条件1 (上下判定): 目标点 Y 必须夹在两顶点的 Y 之间
            // (注意使用 != 以巧妙处理射线刚好穿过顶点导致重复计算的边界情况)
            if (((vertices[i].y > pt.y) != (vertices[j].y > pt.y)) &&
            // 条件2 (左右判定): 相似三角形求交点的 X 坐标，判断交点是否在目标点右侧
                (pt.x < (vertices[j].x - vertices[i].x) * (pt.y - vertices[i].y) / (vertices[j].y - vertices[i].y) + vertices[i].x)) {
                inside = !inside; // 穿过一次边界，内外状态反转
            }
        }
        return inside;
    }
};
