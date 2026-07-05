# Driver 与 Point-LIO 单双雷达适配改造记录

## User Intent

在不修改 Point-LIO C++ 的前提下，为单/双 MID360 提供 driver 参数适配和同容器
intra-process 启动入口。用户明确选择单雷达订阅 `livox/imu`，双雷达订阅
`livox/imu_192_168_1_135`；用户明确要求保持现有固定容量融合队列。

## Scope

- 模块：odom / pointcloud、launch / communication、parameter / yaml
- 目标：bug 修复、链路适配、退出安全
- 运行行为：允许修正错误参数组合、初始化失败、线程退出；正常点云融合算法不变
- 比赛逻辑：本次修改涉及比赛验证逻辑，采用最小改动策略。

## Out of Scope

- Point-LIO C++ 与 `mid360.yaml`
- IMU alias、融合队列结构/容量/孤帧清理
- planner、controller、MPC、ROGMap、QoS、timer、callback group、外参和频率

## Explorer Findings

### Files inspected

- `src/perception/livox_ros_driver2/src/livox_ros_driver2.cpp`
- `src/perception/livox_ros_driver2/src/driver_node.h`
- `src/perception/livox_ros_driver2/src/driver_node.cpp`
- `src/perception/livox_ros_driver2/src/lddc.h`
- `src/perception/livox_ros_driver2/src/lddc.cpp`
- `src/perception/livox_ros_driver2/src/internal_lidar_merger.h`
- `src/perception/livox_ros_driver2/src/internal_lidar_merger.cpp`
- `src/perception/livox_ros_driver2/src/lds.h`
- `src/perception/livox_ros_driver2/src/lds.cpp`
- `src/perception/livox_ros_driver2/src/comm/ldq.h`
- `src/perception/livox_ros_driver2/src/comm/ldq.cpp`
- `src/perception/livox_ros_driver2/src/comm/comm.cpp`
- `src/perception/livox_ros_driver2/src/comm/semaphore.h`
- `src/perception/livox_ros_driver2/src/comm/semaphore.cpp`
- `src/perception/livox_ros_driver2/config/MID360_config.json`
- `src/perception/livox_ros_driver2/config/mixed_MID360_config.json`
- `src/perception/livox_ros_driver2/config/mixed_MID360_component.yaml`
- `src/perception/livox_ros_driver2/CMakeLists.txt`
- `src/perception/livox_ros_driver2/package.xml`
- `src/perception/Point-LIO/launch/livox_pointlio_intra_process.launch.py`
- `src/perception/Point-LIO/config/mid360.yaml`
- `src/perception/Point-LIO/CMakeLists.txt`

### Active logic path

ROS 2 component 构造时由 `DriverNode` 读取参数并创建 `Lddc`。当前代码直接依据
`enable_internal_lidar_merge` 配置 `InternalLidarMerger`；点云线程进入
`Lddc::DistributePointCloudData()` 后，融合开关启用时执行
`DistributeMergedPointCloudData()`，否则按 `multi_topic` 选择全局或逐雷达 publisher。

单雷达 `multi_topic=0` 时，现有 global publisher 会分别创建 `livox/lidar` 和
`livox/imu`。双雷达 `multi_topic=1` 时保留逐 IP 原始 topic，内部 merger 额外发布融合
点云 `livox/lidar`。

### Data flow

- 单雷达：MID360 点云和 IMU 经 driver 直接发布到 `livox/lidar`、`livox/imu`。
- 双雷达：front/back 点云经 driver 内部时间匹配和 back-to-front 变换后发布
  `livox/lidar`；Point-LIO 使用 front 原始 IMU `livox/imu_192_168_1_135`，back 原始 IMU
  仍保留。
- Point-LIO 的 odom、`cloud_registered` 和 `cloud_registered_full` 发布路径不在本次范围。

### Risk notes

- 当前 merge 配置只看 `enable_internal_lidar_merge`，`multi_topic=0` 仍可能错误进入融合等待。
- 当前融合校验未覆盖空 IP、相同 IP 和空输出 topic。
- `Lds::RequestExit()` 只设置标志，不唤醒阻塞在 semaphore 的两个 poll thread。
- `DriverNode` 析构直接解引用 `lddc_ptr_`、`lds_` 和线程指针，并无 joinable 检查。
- 融合成功后先推进队列读指针，后续日志仍读取 `QueuePeek()` 返回的元素指针。
- `LidarDataQueue` 已是固定容量环形队列；20 Hz 时容量向上取整为 32，满后不继续分配。
- 当前 Point-LIO YAML 的 IMU topic 是双雷达 front topic，不能直接作为单雷达默认接口。

### Recommended modification boundary

仅修改 driver 参数选择/校验、线程退出、融合元素生命周期、Point-LIO 通用 component
launch、单/双 wrapper、单雷达 YAML 和测试接入。不改变内部融合数学、底层固定队列、
Point-LIO 源码或其他导航模块。

## Modifier Changes

### Files changed

Explorer 阶段尚无实现文件变更。

### Key changes

Explorer 阶段尚无实现变更。

### Behavior preserved

当前确认保持点云融合算法、固定容量队列、外参、频率、QoS 与 Point-LIO C++ 不变。

### Behavior intentionally adjusted

当前确认将调整错误参数组合、初始化失败、线程退出及单/双 launch 选择行为。

### Notes

构建与真机测试未经许可，不在 Explorer 阶段执行。

## Auditor Review

### Checks performed

- [ ] 关键路径检查
- [ ] diff 检查
- [ ] grep 检查
- [ ] XML / launch / yaml 检查
- [ ] 用户允许范围内的测试或静态检查
- [ ] 如需构建，已取得用户明确许可

### Issues found

尚未进入 Modifier 与 Auditor 阶段。

### Final result

NEEDS_FIX
