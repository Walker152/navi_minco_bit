你现在接手一个 ROS2 工作空间，目标不是在当前电脑上完成实机频率验证，而是把所有代码改造、测试工具、component 组建注册、launch 文件和实机操作文档全部落实好。当前电脑端无法连接雷达/无法进行真实性能测试，实机端没有 Codex，所以你必须一次性完成可移植到实机的代码和文档，最后输出清晰的《操作流程.md》，让我复制到实机后按步骤验证。

已知问题背景：

1. Livox driver 端发布点云频率稳定 10Hz。
2. driver 内部双雷达点云合并耗时约 1ms，不是主要瓶颈。
3. Point-LIO 关闭自身大点云发布后仍然接收慢。
4. `ros2 topic hz` 和 Point-LIO 自己打印的点云接收频率都是 2~7Hz 跳变。
5. 怀疑链路瓶颈在跨进程 DDS 传输大点云、QoS 丢帧、订阅端调度、Point-LIO 主循环 `spin_some()` 与 LIO 主计算耦合。
6. 当前 Livox driver ROS2 端已经是 component node，可以被 component container 加载。
7. 当前 Point-LIO 的 `laserMapping.cpp` 仍是普通 `main()`，内部创建 `rclcpp::Node("laserMapping")`，创建 `MultiThreadedExecutor`，然后在 while 循环中反复 `executor.spin_some()` 和执行 `sync_packages(Measures)` 后的 LIO 主计算。
8. 当前 Point-LIO 点云订阅使用 `rclcpp::SensorDataQoS()`，callback 是 `SharedPtr` 形态：

   * `livox_ros_driver2::msg::CustomMsg::SharedPtr`
   * `sensor_msgs::msg::PointCloud2::SharedPtr`

你的任务：
只负责完成代码、测试节点、launch、文档和可编译性检查。不要声称已经在实机验证 10Hz。所有真实性能结论必须留给《操作流程.md》中的实机测试步骤。

============================================================
一、总体交付物
=======

请最终交付以下内容：

1. 最小 C++ 点云订阅测试节点：

   * 可执行名建议：`min_lidar_subscriber`
   * 用于实机验证跨进程订阅是否能收到 10Hz
   * 支持 `PointCloud2` 和 Livox `CustomMsg`

2. Point-LIO component 化改造：

   * 新增 `point_lio::LaserMappingNode : public rclcpp::Node`
   * 支持 component container 加载
   * 保留原普通可执行启动方式，除非确实无法保留，若无法保留必须在文档中说明

3. Point-LIO executor / 主循环解耦：

   * 不再依赖 while 循环中的 `executor.spin_some()` 来处理订阅 callback
   * component 由外部 `component_container_mt` spin
   * LIO 主处理放入独立 worker thread
   * subscriber callback 只做轻量入队和统计

4. 同进程 component launch：

   * driver 和 Point-LIO 加载到同一个 `component_container_mt`
   * 开启 `use_intra_process_comms`
   * 支持传入 driver yaml 和 Point-LIO yaml

5. 可选但推荐：driver 普通非 merge 发布路径 unique_ptr 优化：

   * internal merge 路径若已经是 unique_ptr publish，则只检查不破坏
   * 普通 `PointCloud2` / `CustomMsg` 发布路径如果仍是 const ref publish，增加 ROS2 unique_ptr 发布路径

6. 文档：

   * 必须新增 `docs/livox_pointlio_intra_process_操作流程.md`
   * 必须新增或更新 `docs/livox_pointlio_rate_diagnosis_template.md`
   * 文档必须是给实机端人工验证用的，不要依赖 Codex 在实机上运行

7. 构建检查：

   * 尝试执行 `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`
   * 如果当前电脑缺 ROS / 缺依赖导致无法编译，不要停止任务；必须尽量做静态检查，并在最终说明中列出未能编译的原因和需要在实机上执行的命令
   * 不允许把“无法实机测试”当作不改代码的理由

============================================================
二、先新增最小 C++ 点云订阅器
=================

新增一个最小 C++ subscriber，用来验证“driver 发布 10Hz，但跨进程订阅者到底能不能收到 10Hz”。

建议文件：

* 如果 Point-LIO 包更合适：`point_lio/src/min_lidar_subscriber.cpp`
* 如果工具包更合适：按当前工程结构放置
* 不要创建过度复杂的新包，除非当前工程确实不适合加入 target

可执行名：

```bash
min_lidar_subscriber
```

参数要求：

1. `lid_topic`

   * 默认：`/livox/lidar` 或当前工程实际默认 topic
2. `msg_type`

   * 可选：`pointcloud2` / `custom`
   * 默认：`pointcloud2`
3. `qos_mode`

   * 可选：`sensor` / `reliable`
   * 默认：`sensor`
4. `qos_depth`

   * 默认：5 或 10
5. `print_period`

   * 默认：1.0 秒

功能要求：

1. 如果 `msg_type=pointcloud2`：

   * 订阅 `sensor_msgs::msg::PointCloud2`
   * callback 第一行立即计数
   * 不做 PCL 转换
   * 不发布任何消息
   * 不保存文件
   * 打印以下信息：

     * callback 接收 Hz
     * 当前统计窗口内 count
     * `width`
     * `height`
     * `point_step`
     * `data.size()` MB
     * header stamp 间隔 ms
     * steady_clock callback 平均间隔 ms
     * 如果能安全计算，打印 `now - header.stamp` 延迟 ms
2. 如果 `msg_type=custom`：

   * 订阅 `livox_ros_driver2::msg::CustomMsg`
   * 打印：

     * callback 接收 Hz
     * `point_num`
     * `points.size()`
     * 估算消息大小 MB
     * `timebase` 间隔 ms
     * steady_clock callback 平均间隔 ms
3. 日志格式必须清晰，例如：

```text
[MIN_SUB][PointCloud2] hz=9.98 count=10 bytes=4.32MB width=123456 height=1 point_step=26 stamp_dt=100.02ms wall_avg=100.15ms delay=12.4ms
```

QoS 要求：

* `sensor` 使用 `rclcpp::SensorDataQoS()`，并根据参数设置 depth，如 API 支持
* `reliable` 使用 keep_last + reliable + volatile
* 文档中说明：reliable 只用于定位问题，不一定适合最终实时导航

CMake / package 要求：

* 补齐 `rclcpp`
* `sensor_msgs`
* `livox_ros_driver2`
* 其他必要依赖
* 确保 `ros2 run <pkg> min_lidar_subscriber` 可运行

============================================================
三、Point-LIO component 化
=======================

当前 `laserMapping.cpp` 是普通 main。请低风险重构为 component，同时尽量保留原 main。

目标结构：

```cpp
namespace point_lio
{
class LaserMappingNode : public rclcpp::Node
{
public:
  explicit LaserMappingNode(const rclcpp::NodeOptions & options);
  ~LaserMappingNode() override;

private:
  void initialize();
  void startWorker();
  void stopWorker();
  void processingLoop();

  std::atomic_bool running_{false};
  std::thread worker_thread_;
};
}
```

注册 component：

```cpp
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(point_lio::LaserMappingNode)
```

CMake 要求：

1. 添加 component library target，例如：

```cmake
add_library(point_lio_component SHARED
  src/laserMapping.cpp
  ...
)
```

或者根据实际工程拆分文件，避免重复定义 main。
2. 添加：

```cmake
rclcpp_components_register_nodes(point_lio_component "point_lio::LaserMappingNode")
```

3. 保留普通 executable，例如：

```cpp
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false);
  auto node = std::make_shared<point_lio::LaserMappingNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
```

4. 如果当前 `laserMapping.cpp` 同时放 component 和 main 会冲突，请拆分：

   * `laser_mapping_node.hpp`
   * `laser_mapping_node.cpp`
   * `laser_mapping_main.cpp`
   * `laser_mapping_component.cpp`
5. `package.xml` 补充：

   * `rclcpp_components`
   * 其他缺失依赖

低风险重构原则：

1. 当前文件里大量全局变量和自由函数可以暂时保留，不要求一次性完全类成员化。
2. 把原 main 里的初始化流程迁移到 `LaserMappingNode::initialize()`。
3. 原来 `nh->create_subscription` 改为 `this->create_subscription`。
4. 原来 `nh->create_publisher` 改为 `this->create_publisher`。
5. 原来 `nh->create_client` 改为 `this->create_client`。
6. 如果 `readParameters(nh)` 只接受 `std::shared_ptr<rclcpp::Node>`，不要在构造函数中强行 `shared_from_this()`；请改成接受 `rclcpp::Node *`、`rclcpp::Node &`，或增加重载。
7. 不允许在 component 内部再创建一个新的 `rclcpp::Node("laserMapping")`。
8. 不要破坏已有参数名、topic 名、frame 名、初始化顺序、GICP recall client、IMU callback、地图发布、里程计发布等功能。

============================================================
四、Point-LIO callback 与 LIO 主处理解耦
================================

当前问题：
原代码在 while 循环中：

```cpp
executor.spin_some();
if (sync_packages(Measures)) {
  ...
  LIO 主计算
}
```

这会导致 LIO 主处理耗时较长时，订阅 callback 无法及时执行。

改造目标：

1. component 节点不再自己创建 executor。
2. component 节点不再在 LIO 主循环里调用 `spin_some()`。
3. ROS callback 由外部 `component_container_mt` 或普通 executable 的 executor 持续 spin。
4. LIO 主处理放到 `processingLoop()` worker thread 中。
5. callback 只做：

   * 频率统计
   * 调用原始点云/IMU callback 或轻量入队
   * 不做重计算

worker thread 逻辑建议：

```cpp
void LaserMappingNode::processingLoop()
{
  rclcpp::Rate rate(500);
  while (rclcpp::ok() && running_.load()) {
    if (flg_exit) {
      break;
    }

    if (sync_packages(Measures)) {
      auto t0 = std::chrono::steady_clock::now();

      // 原 while 中 sync_packages 成功后的全部 LIO 主处理逻辑
      // 包括 crash monitor、p_imu->Process、downsample、map update、publish odom 等

      auto t1 = std::chrono::steady_clock::now();
      // 打印 process_ms 和 sync_in Hz
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}
```

析构要求：

```cpp
LaserMappingNode::~LaserMappingNode()
{
  stopWorker();
}
```

`stopWorker()`：

* `running_ = false`
* 唤醒可能阻塞的条件变量
* join worker thread

注意：

1. 如果原代码依赖 `signal(SIGINT, SigHandle)`，component 中尽量不要全局抢占 signal；普通 executable 可保留，component 模式下优先依赖 rclcpp shutdown。
2. 如果保留原 crash monitor 中 `exit(EXIT_FAILURE)`，请评估 component container 下是否会杀死整个 container。不要擅自删除，但在文档中说明风险；如果改为 `rclcpp::shutdown()` 或抛异常，要保证原有 respawn 策略可解释。
3. 所有共享队列访问要确认线程安全。如果原始点云/IMU callback 已经有 mutex，复用；如果没有，补充必要锁。
4. `record_runtime_rate(RuntimeRateEvent::CloudInput)` 必须仍然在点云 subscriber callback 第一行附近调用，用于统计 callback 到达频率。
5. 新增 `sync_packages` 成功频率统计，建议事件名：

   * `RuntimeRateEvent::CloudInput`
   * `RuntimeRateEvent::SyncInput`
   * `RuntimeRateEvent::OdomPublish`
   * `RuntimeRateEvent::PoseUpdate`
6. 日志格式建议：

```text
[Point-LIO] rates: cloud_cb=9.98Hz, sync_in=9.95Hz, odom_pub=xxHz, pose_update=xxHz, avg_process=xx.xms, max_process=xx.xms
```

============================================================
五、Point-LIO 点云订阅 UniquePtr / ConstSharedPtr 适配
==============================================

目标：
为同进程 intra-process 减少复制。

优先方案：

1. `PointCloud2` callback 使用 `UniquePtr`：

```cpp
sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  lid_topic,
  rclcpp::SensorDataQoS(),
  [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
    record_runtime_rate(RuntimeRateEvent::CloudInput);
    standard_pcl_cbk(std::move(msg));
  });
```

2. `CustomMsg` callback 使用 `UniquePtr`：

```cpp
sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
  lid_topic,
  rclcpp::SensorDataQoS(),
  [this](livox_ros_driver2::msg::CustomMsg::UniquePtr msg) {
    record_runtime_rate(RuntimeRateEvent::CloudInput);
    livox_pcl_cbk(std::move(msg));
  });
```

适配要求：

1. 如果 `standard_pcl_cbk` / `livox_pcl_cbk` 当前只支持 `SharedPtr`，优先增加重载：

   * `void standard_pcl_cbk(sensor_msgs::msg::PointCloud2::UniquePtr msg)`
   * `void livox_pcl_cbk(livox_ros_driver2::msg::CustomMsg::UniquePtr msg)`
2. 如果内部实际只读消息，建议把核心处理拆成：

```cpp
void standard_pcl_cbk_impl(const sensor_msgs::msg::PointCloud2 & msg);
void livox_pcl_cbk_impl(const livox_ros_driver2::msg::CustomMsg & msg);
```

然后 SharedPtr 和 UniquePtr 都调用 impl，避免复制。
3. 如果内部必须把原消息存进队列，确保 move 所有权，不要保存悬空引用。
4. 如果 UniquePtr 改动风险过高，允许第一阶段使用 `ConstSharedPtr`，但必须：

* 完成 component 化和 intra-process launch
* 在文档中明确说明 UniquePtr 未完成的原因
* 保留后续 TODO

5. 不要写出为了适配 UniquePtr 却又复制整帧点云的代码。

============================================================
六、检查和优化 Livox driver 发布路径
=========================

已知 internal merge 路径大概率已经是 unique_ptr 构造和 move publish。请检查但不要破坏。

要求：

1. internal merge 路径：

   * 确认 `BuildMergedCustomMsg()` 返回 `std::unique_ptr<CustomMsg>`
   * 确认 `BuildMergedPointCloud2()` 返回 `std::unique_ptr<PointCloud2>`
   * 确认 publish 使用 `publisher->publish(std::move(msg))`
2. 普通非 merge 路径：

   * 如果 ROS2 分支仍使用 `PointCloud2 cloud;ox driver 发布路径
     ============================================================

已知 internal merge 路径大概率已经是 unique_ptr 构造和 move publish。请检查但不要破坏。

要求：

1. internal merge 路径：

   * 确认 `BuildMergedCustomMsg()` 返回 `std::unique_ptr<CustomMsg>`
   * 确认 `BuildMergedPointCloud2()` 返回 `std::unique_ptr<PointCloud2>`
   * 确认 publish 使用 `publisher->publish(std::move(msg))`
2. 普通非 merge 路径：
   `+`publisher->publish(cloud)`，请新增 unique_ptr 发布路径

   * 如果 ROS2 分支仍使用 `CustomMsg livox_msg;` + `publisher->publish(livox_msg)`，请新增 unique_ptr 发布路径
3. ROS1 分支不要改坏。
4. ROS2 下构造 `PointCloud2` 时尽量直接写入 `cloud->data`，避免：

   * 先构造临时 `std::vector<LivoxPointXyzrtlt>`
   * 再 `memcpy` 到 `cloud.data`
5. 如果这部分改动风险过大，至少在文档中标明：

   * internal merge 已满足 unique_ptr
   * 普通非 merge 路径仍可能复制
   * 当前实机测试主要使用哪个路径

============================================================
七、新增同进程 component launch
========================

新增 launch 文件：

```text
launch/livox_pointlio_intra_process.launch.py
```

要求：

1. 使用 `ComposableNodeContainer`
2. executable 使用：

```text
component_container_mt
```

3. 同一个 container 内加载：

   * `livox_ros::DriverNode`
   * `point_lio::LaserMappingNode`
4. 两个 node 都设置：

```python
extra_arguments=[{'use_intra_process_comms': True}]
```

5. 支持 launch arguments：

   * `driver_params_file`
   * `pointlio_params_file`
   * `use_intra_process`，默认 `true`
   * `container_name`，默认 `livox_pointlio_container`
   * `log_level`，默认 `info`
6. 参数文件可直接复用现有 yaml。
7. 文档中提醒：driver 输出 topic 必须与 Point-LIO 的 `lid_topic` 一致。
8. 不要默认启动 RViz、rosbag、`ros2 topic hz` 或 min subscriber。
9. 可另增一个测试 launch：

```text
launch/min_lidar_subscriber.launch.py
```

但它必须作为可选外部跨进程测试工具，不得混进 intra-process container 的真实性能判断。

示例结构，仅供参考，需按真实包名修正：

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    driver_params_file = LaunchConfiguration('driver_params_file')
    pointlio_params_file = LaunchConfiguration('pointlio_params_file')
    use_intra_process = LaunchConfiguration('use_intra_process')
    container_name = LaunchConfiguration('container_name')
    log_level = LaunchConfiguration('log_level')

    container = ComposableNodeContainer(
        name=container_name,
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        composable_node_descriptions=[
            ComposableNode(
                package='livox_ros_driver2',
                plugin='livox_ros::DriverNode',
                name='livox_driver_node',
                parameters=[driver_params_file],
                extra_arguments=[{'use_intra_process_comms': use_intra_process}],
            ),
            ComposableNode(
                package='point_lio',
                plugin='point_lio::LaserMappingNode',
                name='laserMapping',
                parameters=[pointlio_params_file],
                extra_arguments=[{'use_intra_process_comms': use_intra_process}],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('driver_params_file'),
        DeclareLaunchArgument('pointlio_params_file'),
        DeclareLaunchArgument('use_intra_process', default_value='true'),
        DeclareLaunchArgument('container_name', default_value='livox_pointlio_container'),
        DeclareLaunchArgument('log_level', default_value='info'),
        container,
    ])
```

注意：

* 某些 ROS2 版本中 LaunchConfiguration 作为 `use_intra_process_comms` 的 bool 可能不直接工作。请按当前 ROS2 版本修正，确保实际 launch 可用。
* 如果需要写成固定 `True`，也可以，但要在文档中说明。

============================================================
八、新增《操作流程.md》
=============

必须新增：

```text
docs/livox_pointlio_intra_process_操作流程.md
```

这个文档是给我在实机上手动验证用的，必须足够具体。

文档必须包含以下章节：

1. 改造目的

   * 解释当前问题：driver 10Hz，但跨进程订阅 2~7Hz 跳变
   * 解释目标：用最小 subscriber 定位，再用 component + intra-process 绕开大点云跨进程拷贝

2. 编译步骤

```bash
cd <workspace>
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

3. 参数检查

   * driver 输出 topic
   * Point-LIO `lid_topic`
   * `xfer_format`
   * 是否启用 internal lidar merge
   * 是否关闭 Point-LIO 大点云发布
   * 是否关闭 PCD 保存

4. 实机测试 A：只测 driver + 最小 C++ subscriber

   * 启动 driver 的命令
   * 启动 `min_lidar_subscriber` 的命令
   * `pointcloud2` 和 `custom` 两种命令都给出
   * 记录表格：

```text
driver日志发布Hz：
min_sub接收Hz：
ros2 topic hz：
CPU占用：
结论：
```

5. 实机测试 B：driver + 原跨进程 Point-LIO

   * 启动原 driver
   * 启动原 Point-LIO
   * 可选启动 min subscriber
   * 记录：

```text
Point-LIO cloud_cb Hz：
Point-LIO sync_in Hz：
Point-LIO process_ms：
外部 min_sub Hz：
结论：
```

6. 实机测试 C：component intra-process

   * 启动新 launch：

```bash
ros2 launch <pkg> livox_pointlio_intra_process.launch.py \
  driver_params_file:=<driver_yaml> \
  pointlio_params_file:=<pointlio_yaml> \
  use_intra_process:=true
```

* 不启动 RViz、不录包、不启动外部 topic hz
* 只看 Point-LIO 内部日志
* 记录：

```text
driver publish Hz：
Point-LIO cloud_cb Hz：
Point-LIO sync_in Hz：
Point-LIO avg/max process_ms：
是否仍 2~7Hz 跳变：
结论：
```

7. 实机测试 D：component intra-process + 外部 subscriber 对照

   * 启动 component 后，再启动 min subscriber
   * 明确说明：外部 min subscriber 仍是跨进程订阅，不代表 Point-LIO 内部同进程接收频率
   * 记录内外差异

8. 判定表
   必须写清楚以下判断：

| 现象                                               | 判断                                            |
| ------------------------------------------------ | --------------------------------------------- |
| driver + min_sub 稳定 10Hz，ros2 topic hz 低         | ros2 topic hz 对大点云不可靠                         |
| driver + min_sub 也只有 2~7Hz                       | 普通跨进程 DDS 链路接收不稳                              |
| 只开 driver + min_sub 10Hz，开 Point-LIO 后 min_sub 掉 | Point-LIO 抢占 CPU / 内存带宽 / DDS 线程              |
| component 后 Point-LIO cloud_cb 接近 10Hz           | intra-process 改造有效                            |
| component 后 cloud_cb 10Hz，但 sync_in 低            | 瓶颈转移到 Point-LIO 同步/IMU/处理逻辑                   |
| component 后 cloud_cb 仍 2~7Hz                     | 不是单纯跨进程问题，需要检查 callback、锁、worker、CPU、时间戳和 QoS |

9. 常见问题

   * topic 名不一致
   * `use_intra_process_comms` 没生效
   * 同时启动 RViz / rosbag 导致额外压力
   * reliable QoS 带来延迟堆积
   * Point-LIO worker thread 未启动
   * component plugin 名写错
   * 参数文件没加载
   * CustomMsg / PointCloud2 类型不匹配

10. 回滚方式

* 如何恢复原普通 launch
* 如何只运行原 Point-LIO executable
* 如何关闭 component intra-process

============================================================
九、新增诊断结果模板
==========

新增：

```text
docs/livox_pointlio_rate_diagnosis_template.md
```

内容是空白表格模板，让我实机填结果。至少包含：

```markdown
# Livox + Point-LIO 频率诊断记录

## 测试环境
- 日期：
- 机器：
- ROS2 版本：
- RMW：
- CPU：
- 雷达：
- xfer_format：
- 点云 topic：
- IMU topic：
- 是否 internal merge：

## 测试 A：driver + min_sub
| 项目 | 结果 |
|---|---|
| driver publish Hz | |
| min_sub Hz | |
| ros2 topic hz | |
| CPU | |
| 结论 | |

## 测试 B：driver + 原跨进程 Point-LIO
...

## 测试 C：component intra-process
...

## 最终判断
- DDS / 跨进程是否是主因：
- Point-LIO 调度是否是主因：
- Point-LIO 计算是否是主因：
- 后续需要继续优化的点：
```

============================================================
十、最终输出要求
========

完成后请在最终回复中给出：

1. 修改文件列表
2. 新增文件列表
3. 关键改造摘要
4. 编译结果

   * 如果编译成功，说明成功
   * 如果无法编译，列出具体错误和原因
   * 如果当前环境无 ROS2 或缺依赖，说明“未能在当前环境编译”，但必须保证已完成代码层面改造
5. 实机需要执行的最短命令清单
6. 明确提醒：真实性能结果必须以实机《操作流程.md》测试为准，不能用当前电脑端结果替代

============================================================
十一、禁止事项
=======

1. 不要因为当前电脑无法连接雷达而停止改造。
2. 不要声称已经验证实机 10Hz。
3. 不要删除原有 Point-LIO 核心算法逻辑。
4. 不要把 LIO 重计算放进 subscriber callback。
5. 不要在 component 内部再创建新的 `rclcpp::Node("laserMapping")`。
6. 不要默认启动 RViz、rosbag、topic hz 参与性能测试。
7. 不要把 reliable QoS 当成唯一最终修复。
8. 不要破坏 ROS1 条件编译分支。
9. 不要破坏 Livox driver 已有 internal merge 功能。
10. 不要为了适配 UniquePtr 反而复制整帧大点云。
