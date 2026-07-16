# README 开源项目化升级改造记录

## User Intent

用户希望结合 DJI RM AWARD 申请材料、当前仓库代码和优秀论文/开源项目 README 风格，完善主 README。文档需要美观、详尽、专业，并覆盖配置、安装、启动、功能亮点，以及 Livox IP、Point-LIO 坐标系发布、planner/controller 杆臂补偿、ROGMap 参数与雷达安装高度关系等关键内容。

## Scope

- 重构仓库根目录 `README.md`。
- 以当前生效的 launch、YAML、JSON 和源码为事实来源。
- 补充架构、数据流、模块亮点、安装、构建、启动、参数标定、TF、topics、诊断、限制、致谢和许可证说明。
- 仅修改文档，不改变运行行为。

## Out of Scope

- 不修改任何源码、launch、YAML、JSON 或比赛参数。
- 不抽象硬编码 TF、IP 或硬件 profile。
- 不执行编译、构建、实车运行或提交操作。
- 不重构现有模块和比赛验证逻辑。

## Explorer Findings

### Files inspected

- `README.md`
- `/home/alioth/DJI RM AWARD 2026 申请材料-北京理工大学-喻衡-算法.pdf`
- `/home/alioth/NavDP/README.md`
- `start.bash`、`build.bash`、`play.bash`
- `src/perception/livox_ros_driver2/config/*.json`
- `src/perception/livox_ros_driver2/config/*MID360_component.yaml`
- `src/perception/Point-LIO/config/mid360.yaml`
- `src/perception/Point-LIO/launch/*intra_process.launch.py`
- `src/perception/Point-LIO/src/parameters.cpp`
- `src/perception/Point-LIO/src/preprocess.cpp`
- `src/perception/Point-LIO/src/laserMapping.cpp`
- `src/perception/rog_map/include/rog_map/rog_map_core/config.hpp`
- `src/perception/rog_map/include/rog_map_ros/rog_map_ros2.hpp`
- `src/navigation/navi2_bringup/params/sentry1.yaml`
- `src/navigation/navi2_bringup/launch/navigation2.launch.py`
- `src/navigation/minco_planner/README.md` 及杆臂补偿实现
- `src/navigation/minco_controller/README.md` 及杆臂补偿实现
- `src/decision/bt_manager/launch/bt_manager.launch.py`
- `src/navigation/communication/launch/com.launch.py`
- 各主要功能包 `package.xml` 与 `CMakeLists.txt`
- 根目录 `LICENSE`、`CONTRIBUTING.md`、`AGENTS.md`

外部结构参考包括 Nav2、Point-LIO、GCOPTER、BehaviorTree.CPP、Livox-SDK2 等上游项目 README 或官方页面。

### Active logic path

`start.bash` 当前依次启动 PTP、双 MID-360 + Point-LIO 组件容器、Nav2/ROGMap/MINCO/MPC、bt_manager、communication 和 rosbag。ICP 与点云裁剪默认注释。

默认感知入口为 `mixed_livox_pointlio_intra_process.launch.py`，Livox driver 与 Point-LIO 位于同一多线程 component container。默认导航参数为 `sentry1.yaml`，planner 模式为 `PRIORMAP`。

### Data flow

双雷达经 Livox driver 合并为 `/livox/lidar`，Point-LIO 输出 `/aft_mapped_to_init`、`/cloud_registered` 和 `/cloud_registered_full`。ROGMap 消费稠密点云与 odom，通过进程内 `MapQueryInterface` 向 MincoPlanner 提供占据、投影和 Signed ESDF 查询。MincoPlanner 发布 `/opt_path`，MincoMpcController 发布 `/cmd_vel_mpc`，最终由 communication 接入底盘。

### Risk notes

- 主机 IP、雷达 IP、IMU topic 和 merge 前后雷达 IP 存在同步关系。
- Point-LIO `blind_center`、硬编码 `camera_init → base_link` 偏置、ROGMap `center_offset` 与 planner/controller 杆臂参数描述同一车辆几何的不同方向/用途，符号不可机械照抄。
- `projection.scan_z_min_abs/max_abs` 是 ROGMap frame 中的绝对 Z 坐标，不是离地高度。
- 当前投影 Z 请求跨度为 `1.70 m`，大于 `map_size.z=1.50 m`；源码会将投影索引裁剪到当前滑动地图边界，README 已将其列为显式风险，而非把现有数值描述为通用正确配置。
- `navigation2.launch.py` 包含赛场相关静态 TF；与 ICP 同时启用会有重复发布风险。
- `start.bash` 固定工作空间路径，依赖 GNOME Terminal 和 sudo，并自动启动实车通信与 rosbag。
- 申请材料性能数据是指定硬件和配置下的阶段实测，不应表述为跨平台保证。

### Recommended modification boundary

仅重写根 README，清晰标注当前基线、配置耦合、硬编码边界和实车风险；不修改任何生效逻辑或参数。

## Modifier Changes

### Files changed

- `README.md`
- `docs/ai_refactor_records/20260716_readme_open_source_upgrade.md`

### Key changes

- 采用论文展示与工程手册混合结构，增加徽章、快速链接、Mermaid 架构图、数据流表和模块亮点。
- 补充安装依赖、Livox-SDK2、rosdep、构建脚本和内存预算说明。
- 新增完整“上车前关键配置”，覆盖 Livox 网络/IP、双雷达融合、Point-LIO 外参与盲区中心、固定坐标系发布、planner/controller 杆臂补偿、ROGMap 中心与雷达高度换算。
- 新增分模块、一键、单雷达、建图与先验地图启动流程。
- 新增参数索引、TF 树、核心 topics、诊断、常见问题、当前限制、安全、贡献、致谢和许可证章节。
- 将性能数字限定为申请材料记录的特定平台阶段实测。
- 根据用户复审移除已弃用且无实质用途的 corridor、ICP 和 cloud filter 说明。
- 新增点云链路性能改造专章，覆盖驱动侧融合、component container、UniquePtr、稠密去畸变队列、latest-state QoS、增量地图更新和进程内查询。
- 增加 emoji 与 ROGMap、Signed ESDF、MINCO、SE(2) MPC 技术栈徽章。
- 将克隆地址更新为 `Walker152/navi_minco_bit.git` 的 `rog_map_work` 分支。
- 增加中国科学技术大学 2025 哨兵技术报告致谢。
- 保留 Apache-2.0，并增加不与其授权冲突的反抄袭、来源标注和学术诚信声明。

### Behavior preserved

所有源码、参数、topic、frame、QoS、launch 组合、行为树和比赛策略保持不变。

### Behavior intentionally adjusted

无运行行为调整；仅调整文档信息架构和说明内容。

### Notes

README 中的示例构建命令仅作为用户操作说明，本次任务没有实际执行这些命令。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查
- [x] grep 检查
- [x] XML / launch / yaml 检查
- [x] 用户允许范围内的测试或静态检查
- [x] 如需构建，已取得用户明确许可（本次不需要且未执行构建）

### Issues found

- README 本地相对链接检查通过，无缺失目标。
- Livox JSON、Point-LIO YAML、Nav2 YAML 静态解析通过。
- 关键 IP、topic、frame、blind center、map center 和杆臂参数已与当前文件逐项比对。
- 发现并记录 ROGMap 投影 Z 请求区间超出当前 Z 窗口的配置风险；本次只完善文档，未调整比赛参数。
- 工作区原有 `com_interface_ros.hpp` 修改和测试脚本删除未被触碰。
- 用户复审后再次 grep，根 README 中已无 corridor、ICP、cloud filter 或 `msg_convert` 有效说明。
- 点云性能章节的 UniquePtr 发布/订阅、同容器 intra-process、深度 1 SensorDataQoS、有序稠密点队列、状态快照补偿、dirty-column 与 MapQueryInterface 均已由当前源码交叉验证。
- 仓库地址、`rog_map_work` 分支、技术栈徽章、中科大技术报告致谢和 Apache-2.0 学术诚信声明检查通过。

### Final result

PASS
