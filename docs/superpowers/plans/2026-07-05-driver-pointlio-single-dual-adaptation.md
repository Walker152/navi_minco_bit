# Livox Driver Single/Dual MID360 Adaptation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the existing Livox ROS 2 driver safely select single-lidar direct output or dual-lidar internal merge, and add reusable intra-process launch entries with mode-specific Point-LIO IMU topics.

**Architecture:** Keep the existing `DriverNode -> Lddc -> InternalLidarMerger` path and fixed-capacity SDK queues. Add one pure effective-merge predicate plus one pure validation helper, harden the existing thread shutdown path, and use lightweight launch wrappers that reuse the current component container.

**Tech Stack:** ROS 2 Humble, rclcpp components, C++14/C++17, ament CMake, GoogleTest, Python launch, YAML/JSON, Python unittest static checks.

---

## Constraints Applied Throughout

- Do not modify Point-LIO C++ or `src/perception/Point-LIO/config/mid360.yaml`.
- Do not modify planner, controller, MPC, ROGMap, topic names, frames, QoS, timers, callback groups, fusion extrinsics, or publish frequencies.
- Point-LIO IMU input is mode-specific by explicit user decision: `livox/imu` for single lidar and `livox/imu_192_168_1_135` for dual lidar.
- Do not add an IMU alias publisher.
- Do not change the fixed-capacity `LidarDataQueue`, add `merge_max_queue_size`, or add orphan-age cleanup.
- Do not run any build, compile, CMake configure, colcon test, ROS launch, ASan, or valgrind command without explicit user permission.
- Preserve unrelated worktree changes in `AGENTS.md`, the deleted push-script record, and the untracked prompt file.

## File Map

- Create `docs/ai_refactor_records/20260705_driver_pointlio_single_dual_adaptation.md`: required auditable task record.
- Create `src/perception/livox_ros_driver2/test/internal_lidar_merger_test.cpp`: unit tests for merge behavior, effective-mode selection, validation, and exit wakeup.
- Modify `src/perception/livox_ros_driver2/CMakeLists.txt`: restore the focused gtest target only.
- Modify `src/perception/livox_ros_driver2/package.xml`: restore `ament_cmake_gtest` test dependency only.
- Modify `src/perception/livox_ros_driver2/src/internal_lidar_merger.h`: declare pure selection and validation helpers.
- Modify `src/perception/livox_ros_driver2/src/internal_lidar_merger.cpp`: implement the pure helpers without changing point transformation/message building.
- Modify `src/perception/livox_ros_driver2/src/livox_ros_driver2.cpp`: use effective merge, validate only effective mode, reject failed initialization, and harden poll loops.
- Modify `src/perception/livox_ros_driver2/src/driver_node.h`: add ROS 2 stop state.
- Modify `src/perception/livox_ros_driver2/src/driver_node.cpp`: null-safe, wakeable, join-safe destructor.
- Modify `src/perception/livox_ros_driver2/src/lds.cpp`: wake both semaphores when exit is requested.
- Modify `src/perception/livox_ros_driver2/src/lddc.cpp`: recheck exit after semaphore waits and stop using queue element pointers after pop.
- Create `src/perception/livox_ros_driver2/config/single_MID360_component.yaml`: single MID360 direct-output parameters.
- Inspect `src/perception/livox_ros_driver2/config/mixed_MID360_component.yaml`: verify and retain dual values; no expected edit.
- Modify `src/perception/Point-LIO/launch/livox_pointlio_intra_process.launch.py`: expose a mode-specific Point-LIO IMU override.
- Create `src/perception/Point-LIO/launch/single_livox_pointlio_intra_process.launch.py`: single-lidar wrapper.
- Create `src/perception/Point-LIO/launch/mixed_livox_pointlio_intra_process.launch.py`: dual-lidar wrapper.
- Create `src/perception/livox_ros_driver2/test/test_launch_config.py`: non-build static regression tests for launch/config artifacts.

### Task 1: Establish the audit record before code changes

**Files:**
- Create: `docs/ai_refactor_records/20260705_driver_pointlio_single_dual_adaptation.md`

- [ ] **Step 1: Write the record header and Explorer findings**

Create the required sections with this exact scope statement:

```markdown
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

列出本计划 File Map 中现有的 driver、launch、YAML、JSON、CMake 与 package 文件。

### Active logic path

记录 `DriverNode -> Lddc::DistributePointCloudData() -> InternalLidarMerger`，以及单雷达
`multi_topic=0` 的 global publisher 路径。

### Data flow

记录单雷达 `livox/lidar + livox/imu`，双雷达融合点云加 front 原始 IMU topic。

### Risk notes

记录当前 raw merge 开关、参数校验不完整、semaphore 退出不唤醒、队列元素 pop 后仍访问。

### Recommended modification boundary

仅修改 File Map 中列出的文件，不改变内部融合数学、底层固定队列或 Point-LIO 源码。

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
```

- [ ] **Step 2: Confirm only the record is changed**

Run:

```bash
git diff -- docs/ai_refactor_records/20260705_driver_pointlio_single_dual_adaptation.md
git status --short
```

Expected: the new record is the only task file; pre-existing unrelated changes remain untouched.

- [ ] **Step 3: Commit the Explorer record**

```bash
git add -f docs/ai_refactor_records/20260705_driver_pointlio_single_dual_adaptation.md
git commit -m "docs: record lidar adaptation scope"
```

### Task 2: Restore focused tests and define effective-merge behavior

**Files:**
- Create: `src/perception/livox_ros_driver2/test/internal_lidar_merger_test.cpp`
- Modify: `src/perception/livox_ros_driver2/CMakeLists.txt:134-142`
- Modify: `src/perception/livox_ros_driver2/package.xml:23-29`
- Modify: `src/perception/livox_ros_driver2/src/internal_lidar_merger.h:31-52`
- Modify: `src/perception/livox_ros_driver2/src/internal_lidar_merger.cpp:31-41`

- [ ] **Step 1: Add failing unit tests before helper implementation**

Restore the historical transform/message/time-window tests from commit `6bbb1a46^`, then add these tests near the transfer-format test:

```cpp
TEST(InternalLidarMergerTest, EnablesMergeOnlyForRequestedMultiTopicMode)
{
  EXPECT_FALSE(IsInternalLidarMergeEffective(false, 0));
  EXPECT_FALSE(IsInternalLidarMergeEffective(true, 0));
  EXPECT_FALSE(IsInternalLidarMergeEffective(false, 1));
  EXPECT_TRUE(IsInternalLidarMergeEffective(true, 1));
}

TEST(InternalLidarMergerTest, ValidatesOnlyMergeSpecificValues)
{
  EXPECT_TRUE(GetInternalLidarMergeParameterError(
    1, 6, 5.0, "192.168.1.135", "192.168.1.122", "livox/lidar").empty());
  EXPECT_EQ(GetInternalLidarMergeParameterError(
    2, 6, 5.0, "192.168.1.135", "192.168.1.122", "livox/lidar"),
    "unsupported transfer format for internal lidar merge");
  EXPECT_EQ(GetInternalLidarMergeParameterError(
    1, 5, 5.0, "192.168.1.135", "192.168.1.122", "livox/lidar"),
    "invalid merge extrinsic size");
  EXPECT_EQ(GetInternalLidarMergeParameterError(
    1, 6, 0.0, "192.168.1.135", "192.168.1.122", "livox/lidar"),
    "invalid merge max interval");
  EXPECT_EQ(GetInternalLidarMergeParameterError(
    1, 6, 5.0, "", "192.168.1.122", "livox/lidar"),
    "invalid merge lidar ip configuration");
  EXPECT_EQ(GetInternalLidarMergeParameterError(
    1, 6, 5.0, "192.168.1.135", "192.168.1.135", "livox/lidar"),
    "invalid merge lidar ip configuration");
  EXPECT_EQ(GetInternalLidarMergeParameterError(
    1, 6, 5.0, "192.168.1.135", "192.168.1.122", ""),
    "invalid merge output topic");
}
```

Add the minimal test target:

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(internal_lidar_merger_test
    test/internal_lidar_merger_test.cpp
  )
  if(TARGET internal_lidar_merger_test)
    target_include_directories(internal_lidar_merger_test PRIVATE
      src
      ${PCL_INCLUDE_DIRS}
      ${LIVOX_LIDAR_SDK_INCLUDE_DIR}
    )
    target_link_libraries(internal_lidar_merger_test ${PROJECT_NAME})
  endif()
endif()
```

Add:

```xml
<test_depend>ament_cmake_gtest</test_depend>
```

- [ ] **Step 2: Request build permission and verify RED**

This step is forbidden until the user explicitly permits building.

After permission, run:

```bash
colcon build --packages-select livox_ros_driver2 --cmake-args -DBUILD_TESTING=ON
```

Expected: compilation fails because `IsInternalLidarMergeEffective` and
`GetInternalLidarMergeParameterError` are not declared.

- [ ] **Step 3: Add the minimal pure helper declarations**

Add to `internal_lidar_merger.h`:

```cpp
bool IsInternalLidarMergeEffective(bool merge_requested, int multi_topic);

std::string GetInternalLidarMergeParameterError(
  int transfer_format,
  std::size_t extrinsic_size,
  double max_interval_ms,
  const std::string & front_ip,
  const std::string & back_ip,
  const std::string & output_topic);
```

Add `<cstddef>` for `std::size_t`.

- [ ] **Step 4: Implement only the tested behavior**

Add to `internal_lidar_merger.cpp`:

```cpp
bool IsInternalLidarMergeEffective(bool merge_requested, int multi_topic)
{
  return merge_requested && multi_topic == 1;
}

std::string GetInternalLidarMergeParameterError(
  int transfer_format,
  std::size_t extrinsic_size,
  double max_interval_ms,
  const std::string & front_ip,
  const std::string & back_ip,
  const std::string & output_topic)
{
  if (!IsInternalLidarMergeTransferFormatSupported(transfer_format)) {
    return "unsupported transfer format for internal lidar merge";
  }
  if (extrinsic_size != 6) {
    return "invalid merge extrinsic size";
  }
  if (max_interval_ms <= 0.0) {
    return "invalid merge max interval";
  }
  if (front_ip.empty() || back_ip.empty() || front_ip == back_ip) {
    return "invalid merge lidar ip configuration";
  }
  if (output_topic.empty()) {
    return "invalid merge output topic";
  }
  return "";
}
```

- [ ] **Step 5: Verify GREEN after build permission**

```bash
colcon build --packages-select livox_ros_driver2 --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select livox_ros_driver2 --event-handlers console_direct+
colcon test-result --verbose
```

Expected: build succeeds and `internal_lidar_merger_test` passes.

- [ ] **Step 6: Commit focused helper and tests**

```bash
git add src/perception/livox_ros_driver2/CMakeLists.txt \
  src/perception/livox_ros_driver2/package.xml \
  src/perception/livox_ros_driver2/src/internal_lidar_merger.h \
  src/perception/livox_ros_driver2/src/internal_lidar_merger.cpp \
  src/perception/livox_ros_driver2/test/internal_lidar_merger_test.cpp
git commit -m "test: restore internal lidar merge coverage"
```

### Task 3: Apply effective merge and startup validation

**Files:**
- Modify: `src/perception/livox_ros_driver2/src/livox_ros_driver2.cpp:127-243`

- [ ] **Step 1: Replace the raw merge branch with the tested predicate**

Immediately after parameter reads, add:

```cpp
const bool effective_internal_merge =
  IsInternalLidarMergeEffective(enable_internal_lidar_merge, multi_topic);
if (enable_internal_lidar_merge && !effective_internal_merge) {
  RCLCPP_WARN(
    get_logger(),
    "[LivoxDriver] enable_internal_lidar_merge=true is ignored because multi_topic=0. "
    "Use direct single-lidar path.");
}
```

Replace `if (enable_internal_lidar_merge)` with `if (effective_internal_merge)`.

- [ ] **Step 2: Use the pure validation result only in effective mode**

At the start of the effective branch, use:

```cpp
const std::string merge_parameter_error = GetInternalLidarMergeParameterError(
  xfer_format,
  merge_extrinsic_back_to_front.size(),
  merge_max_interval_ms,
  merge_front_ip,
  merge_back_ip,
  merge_output_topic);
if (!merge_parameter_error.empty()) {
  RCLCPP_ERROR(get_logger(), "%s", merge_parameter_error.c_str());
  throw std::invalid_argument(merge_parameter_error);
}
if (merge_frame_id.empty()) {
  merge_frame_id = frame_id;
}
```

Delete the duplicated inline checks. Keep the existing config fields, IP conversion, external transform
copy, and `ConfigureInternalLidarMerge()` call unchanged.

- [ ] **Step 3: Fail before thread startup when data-source initialization fails**

Use explicit failures:

```cpp
if (data_src != kSourceRawLidar) {
  RCLCPP_ERROR(get_logger(), "Invalid data src (%d), please check the launch file", data_src);
  throw std::invalid_argument("invalid lidar data source");
}

LdsLidar * read_lidar = LdsLidar::GetInstance(publish_freq);
if (!read_lidar || lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar)) != 0) {
  throw std::runtime_error("failed to register lidar data source");
}
if (!read_lidar->InitLdsLidar(user_config_path)) {
  RCLCPP_ERROR(get_logger(), "Init lds lidar failed");
  throw std::runtime_error("failed to initialize lidar data source");
}
```

Only create the two poll threads after these checks pass.

- [ ] **Step 4: Verify source-level branch invariants without building**

```bash
rg -n "effective_internal_merge|enable_internal_lidar_merge=true is ignored|GetInternalLidarMergeParameterError" \
  src/perception/livox_ros_driver2/src/livox_ros_driver2.cpp
rg -n "if \(enable_internal_lidar_merge\)" \
  src/perception/livox_ros_driver2/src/livox_ros_driver2.cpp
```

Expected: effective predicate, warning, and validation call are present; the raw merge branch is absent.

- [ ] **Step 5: Run tests after build permission**

```bash
colcon build --packages-select livox_ros_driver2 --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select livox_ros_driver2 --event-handlers console_direct+
```

Expected: build and tests pass.

- [ ] **Step 6: Commit startup selection and validation**

```bash
git add src/perception/livox_ros_driver2/src/livox_ros_driver2.cpp
git commit -m "fix: gate internal lidar merge by topic mode"
```

### Task 4: Make poll-thread shutdown wakeable and null-safe

**Files:**
- Modify: `src/perception/livox_ros_driver2/test/internal_lidar_merger_test.cpp`
- Modify: `src/perception/livox_ros_driver2/src/driver_node.h:25-73`
- Modify: `src/perception/livox_ros_driver2/src/driver_node.cpp:35-41`
- Modify: `src/perception/livox_ros_driver2/src/lds.cpp:77-80`
- Modify: `src/perception/livox_ros_driver2/src/lddc.cpp:135-186`
- Modify: `src/perception/livox_ros_driver2/src/livox_ros_driver2.cpp:252-270`

- [ ] **Step 1: Add a failing exit-wakeup test**

Include `lds_lidar.h`, then add:

```cpp
TEST(LdsTest, RequestExitWakesBothPollSemaphores)
{
  LdsLidar * lds = LdsLidar::GetInstance(20.0);
  ASSERT_NE(lds, nullptr);
  lds->CleanRequestExit();

  lds->RequestExit();

  EXPECT_TRUE(lds->IsRequestExit());
  EXPECT_GT(lds->pcd_semaphore_.GetCount(), 0);
  EXPECT_GT(lds->imu_semaphore_.GetCount(), 0);
  lds->CleanRequestExit();
}
```

- [ ] **Step 2: Verify RED after build permission**

```bash
colcon build --packages-select livox_ros_driver2 --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select livox_ros_driver2 --event-handlers console_direct+
```

Expected: `RequestExitWakesBothPollSemaphores` fails because both counts remain zero.

- [ ] **Step 3: Wake both waits from the existing exit method**

Change `Lds::RequestExit()` to:

```cpp
void Lds::RequestExit()
{
  request_exit_ = true;
  pcd_semaphore_.Signal();
  imu_semaphore_.Signal();
}
```

Immediately after each `Wait()` in `Lddc`, return if `IsRequestExit()` is now true.

- [ ] **Step 4: Add ROS 2 stop state and a safe destructor**

Add `<atomic>` and this ROS 2 member in `driver_node.h`:

```cpp
std::atomic_bool stop_requested_{false};
```

Replace the destructor body with:

```cpp
DriverNode::~DriverNode()
{
  stop_requested_.store(true);
  try {
    exit_signal_.set_value();
  } catch (const std::future_error &) {
  }

  if (lddc_ptr_ && lddc_ptr_->lds_) {
    lddc_ptr_->lds_->RequestExit();
  }
  if (pointclouddata_poll_thread_ && pointclouddata_poll_thread_->joinable()) {
    pointclouddata_poll_thread_->join();
  }
  if (imudata_poll_thread_ && imudata_poll_thread_->joinable()) {
    imudata_poll_thread_->join();
  }
  lddc_ptr_.reset();
}
```

- [ ] **Step 5: Make the initial wait interruptible and stop exceptions at thread boundaries**

Use this structure for both poll methods, substituting the distribution call and message:

```cpp
if (future_.wait_for(std::chrono::seconds(3)) != std::future_status::timeout) {
  return;
}
while (rclcpp::ok() && !stop_requested_.load()) {
  Lddc * lddc = lddc_ptr_.get();
  if (!lddc) {
    return;
  }
  try {
    lddc->DistributePointCloudData();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Point cloud poll thread stopped: %s", error.what());
    return;
  }
  if (future_.wait_for(std::chrono::microseconds(0)) != std::future_status::timeout) {
    return;
  }
}
```

- [ ] **Step 6: Verify GREEN after build permission**

```bash
colcon build --packages-select livox_ros_driver2 --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select livox_ros_driver2 --event-handlers console_direct+
```

Expected: all focused tests pass.

- [ ] **Step 7: Commit thread lifecycle hardening**

```bash
git add src/perception/livox_ros_driver2/test/internal_lidar_merger_test.cpp \
  src/perception/livox_ros_driver2/src/driver_node.h \
  src/perception/livox_ros_driver2/src/driver_node.cpp \
  src/perception/livox_ros_driver2/src/lds.cpp \
  src/perception/livox_ros_driver2/src/lddc.cpp \
  src/perception/livox_ros_driver2/src/livox_ros_driver2.cpp
git commit -m "fix: stop livox poll threads safely"
```

### Task 5: Stop accessing merge queue elements after pop

**Files:**
- Modify: `src/perception/livox_ros_driver2/src/lddc.cpp:198-277`

- [ ] **Step 1: Capture all post-publish metadata before queue pop**

Evaluate the two queue lookups independently and keep the existing throttled counter for a missing-side warning:

```cpp
const bool front_ready =
  GetMergeQueue(internal_lidar_merger_.GetConfig().front_handle, &front_queue);
const bool back_ready =
  GetMergeQueue(internal_lidar_merger_.GetConfig().back_handle, &back_queue);
if (!front_ready || !back_ready) {
  if ((merge_drop_warn_count_++ % 30) == 0) {
    DRIVER_WARN(*cur_node_, "Internal lidar merge is waiting for front/back lidar data");
  }
  return;
}
```

After successful `QueuePeek()`, capture:

```cpp
const uint64_t front_base_time = front->base_time;
const uint64_t back_base_time = back->base_time;
const uint64_t merged_base_time = std::min(front_base_time, back_base_time);
const uint32_t front_points_num = front->points_num;
const uint32_t back_points_num = back->points_num;
```

Use these values for matching, warnings, duration logging and debug deltas. Keep dereferencing `front`
and `back` only while building the merged message, before either `QueuePopUpdate()`.

- [ ] **Step 2: Verify no queue-element dereference remains after pop**

```bash
nl -ba src/perception/livox_ros_driver2/src/lddc.cpp | sed -n '188,285p'
```

Expected: after the two successful `QueuePopUpdate()` calls, all log/debug code uses captured scalars;
there are no `front->` or `back->` expressions.

- [ ] **Step 3: Run focused tests after build permission**

```bash
colcon build --packages-select livox_ros_driver2 --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select livox_ros_driver2 --event-handlers console_direct+
```

Expected: existing merge output and time-window tests remain green.

- [ ] **Step 4: Commit the lifetime fix**

```bash
git add src/perception/livox_ros_driver2/src/lddc.cpp
git commit -m "fix: preserve merge metadata before queue pop"
```

### Task 6: Add failing static tests for launch and YAML artifacts

**Files:**
- Create: `src/perception/livox_ros_driver2/test/test_launch_config.py`

- [ ] **Step 1: Write the static test before creating launch/YAML files**

```python
import ast
import json
import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[4]
DRIVER = ROOT / "src/perception/livox_ros_driver2"
POINT_LIO = ROOT / "src/perception/Point-LIO"


class LaunchConfigTest(unittest.TestCase):
    def test_launch_files_parse_and_keep_intra_process_container(self):
        common = POINT_LIO / "launch/livox_pointlio_intra_process.launch.py"
        single = POINT_LIO / "launch/single_livox_pointlio_intra_process.launch.py"
        mixed = POINT_LIO / "launch/mixed_livox_pointlio_intra_process.launch.py"
        for launch_file in (common, single, mixed):
            ast.parse(launch_file.read_text(encoding="utf-8"))

        common_text = common.read_text(encoding="utf-8")
        self.assertIn('LaunchConfiguration("pointlio_imu_topic")', common_text)
        self.assertIn('"common.imu_topic": pointlio_imu_topic', common_text)
        self.assertEqual(common_text.count('"use_intra_process_comms"'), 2)
        self.assertIn('executable="component_container_mt"', common_text)

    def test_wrappers_select_mode_specific_driver_and_imu(self):
        single_text = (POINT_LIO / "launch/single_livox_pointlio_intra_process.launch.py").read_text()
        mixed_text = (POINT_LIO / "launch/mixed_livox_pointlio_intra_process.launch.py").read_text()
        self.assertIn("single_MID360_component.yaml", single_text)
        self.assertIn('"pointlio_imu_topic": "livox/imu"', single_text)
        self.assertIn("mixed_MID360_component.yaml", mixed_text)
        self.assertIn('"pointlio_imu_topic": "livox/imu_192_168_1_135"', mixed_text)

    def test_driver_yaml_matches_single_and_dual_modes(self):
        single = yaml.safe_load((DRIVER / "config/single_MID360_component.yaml").read_text())
        mixed = yaml.safe_load((DRIVER / "config/mixed_MID360_component.yaml").read_text())
        single_params = single["/**"]["ros__parameters"]
        mixed_params = mixed["/**"]["ros__parameters"]
        self.assertEqual(single_params["multi_topic"], 0)
        self.assertFalse(single_params["enable_internal_lidar_merge"])
        self.assertTrue(single_params["user_config_path"].endswith("/config/MID360_config.json"))
        self.assertEqual(mixed_params["multi_topic"], 1)
        self.assertTrue(mixed_params["enable_internal_lidar_merge"])
        self.assertEqual(mixed_params["merge_output_topic"], "livox/lidar")

        config = json.loads((DRIVER / "config/mixed_MID360_config.json").read_text())
        ips = {entry["ip"] for entry in config["lidar_configs"]}
        self.assertIn(mixed_params["merge_front_ip"], ips)
        self.assertIn(mixed_params["merge_back_ip"], ips)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Verify RED without building**

```bash
python3 -m unittest src/perception/livox_ros_driver2/test/test_launch_config.py -v
```

Expected: failure because the single/mixed wrapper and single YAML do not yet exist.

- [ ] **Step 3: Commit the failing static regression test**

```bash
git add src/perception/livox_ros_driver2/test/test_launch_config.py
git commit -m "test: define single dual lidar launch configuration"
```

### Task 7: Add mode-specific YAML and reusable launch wrappers

**Files:**
- Create: `src/perception/livox_ros_driver2/config/single_MID360_component.yaml`
- Inspect: `src/perception/livox_ros_driver2/config/mixed_MID360_component.yaml`
- Modify: `src/perception/Point-LIO/launch/livox_pointlio_intra_process.launch.py`
- Create: `src/perception/Point-LIO/launch/single_livox_pointlio_intra_process.launch.py`
- Create: `src/perception/Point-LIO/launch/mixed_livox_pointlio_intra_process.launch.py`

- [ ] **Step 1: Add the single-lidar YAML by copying only the established parameter shape**

Use the existing mixed YAML fields with these mode-specific values:

```yaml
/**:
  ros__parameters:
    xfer_format: 1
    multi_topic: 0
    data_src: 0
    publish_freq: 20.0
    output_data_type: 0
    frame_id: livox_frame
    lvx_file_path: /home/livox/livox_test.lvx
    user_config_path: "$(find-pkg-share livox_ros_driver2)/config/MID360_config.json"
    cmdline_input_bd_code: livox0000000001
    enable_internal_lidar_merge: false
    enable_merge_debug: false
    merge_front_ip: 192.168.1.196
    merge_back_ip: 192.168.1.122
    merge_output_topic: livox/lidar
    merge_frame_id: livox_frame
    merge_max_interval_ms: 5.0
    merge_extrinsic_back_to_front: [0.0, 0.4, 0.0, -0.35453, 0.0, 0.0]
```

Do not add a queue-size parameter.

- [ ] **Step 2: Verify the existing mixed YAML without changing it**

Confirm it already contains:

```yaml
multi_topic: 1
enable_internal_lidar_merge: true
merge_front_ip: 192.168.1.135
merge_back_ip: 192.168.1.122
merge_output_topic: livox/lidar
merge_frame_id: livox_frame
```

- [ ] **Step 3: Add the Point-LIO IMU override to the common launch**

Declare:

```python
pointlio_imu_topic = LaunchConfiguration("pointlio_imu_topic")
```

Change Point-LIO parameters to:

```python
parameters=[
    pointlio_params_file,
    {"common.imu_topic": pointlio_imu_topic},
],
```

Add the launch argument with the current dual default:

```python
DeclareLaunchArgument(
    "pointlio_imu_topic",
    default_value="livox/imu_192_168_1_135",
),
```

- [ ] **Step 4: Add the single wrapper**

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("point_lio"),
                    "launch",
                    "livox_pointlio_intra_process.launch.py",
                ])
            ),
            launch_arguments={
                "driver_params_file": PathJoinSubstitution([
                    FindPackageShare("livox_ros_driver2"),
                    "config",
                    "single_MID360_component.yaml",
                ]),
                "pointlio_params_file": PathJoinSubstitution([
                    FindPackageShare("point_lio"), "config", "mid360.yaml"
                ]),
                "pointlio_imu_topic": "livox/imu",
                "use_intra_process": "true",
                "container_name": "single_livox_pointlio_container",
                "log_level": "info",
            }.items(),
        )
    ])
```

- [ ] **Step 5: Add the mixed wrapper with only mode-specific substitutions**

Use the same wrapper code, changing exactly:

```python
"mixed_MID360_component.yaml"
"livox/imu_192_168_1_135"
"mixed_livox_pointlio_container"
```

- [ ] **Step 6: Verify GREEN without building**

```bash
python3 -m unittest src/perception/livox_ros_driver2/test/test_launch_config.py -v
```

Expected: all three static tests pass.

- [ ] **Step 7: Parse every touched launch/YAML/JSON artifact**

```bash
python3 -c 'import ast,pathlib; [ast.parse(p.read_text()) for p in pathlib.Path("src/perception/Point-LIO/launch").glob("*livox_pointlio_intra_process.launch.py")]'
python3 -c 'import json,yaml,pathlib; [yaml.safe_load(p.read_text()) for p in pathlib.Path("src/perception/livox_ros_driver2/config").glob("*MID360_component.yaml")]; [json.loads(p.read_text()) for p in pathlib.Path("src/perception/livox_ros_driver2/config").glob("*MID360_config.json")]'
```

Expected: both commands exit zero with no output.

- [ ] **Step 8: Commit launch and YAML artifacts**

```bash
git add src/perception/livox_ros_driver2/config/single_MID360_component.yaml \
  src/perception/Point-LIO/launch/livox_pointlio_intra_process.launch.py \
  src/perception/Point-LIO/launch/single_livox_pointlio_intra_process.launch.py \
  src/perception/Point-LIO/launch/mixed_livox_pointlio_intra_process.launch.py
git commit -m "feat: add single dual lidar pointlio launches"
```

### Task 8: Complete the record and perform an independent static audit

**Files:**
- Modify: `docs/ai_refactor_records/20260705_driver_pointlio_single_dual_adaptation.md`

- [ ] **Step 1: Fill Modifier Changes from the actual diff**

Record every task file, the effective-mode matrix, mode-specific IMU behavior, preserved queue, thread
shutdown changes, and the deliberate absence of an IMU alias.

- [ ] **Step 2: Re-read the authoritative paths independently**

```bash
rg -n "effective_internal_merge|enable_internal_lidar_merge|GetInternalLidarMergeParameterError|RequestExit|stop_requested|QueuePopUpdate" \
  src/perception/livox_ros_driver2/src
rg -n "pointlio_imu_topic|use_intra_process_comms|component_container_mt|single_MID360_component|mixed_MID360_component" \
  src/perception/Point-LIO/launch src/perception/livox_ros_driver2/config
rg -n "cloud_registered_full|aft_mapped_to_init|imu_topic|lid_topic" src/perception/Point-LIO
```

Expected: all new behavior is confined to the approved driver/launch/config boundary; Point-LIO C++ is unchanged.

- [ ] **Step 3: Run non-build checks**

```bash
python3 -m unittest src/perception/livox_ros_driver2/test/test_launch_config.py -v
git diff --check
git status --short
git diff -- src/perception/Point-LIO/src src/planning src/navigation
```

Expected: static tests pass, no whitespace errors, unrelated user changes remain, and the out-of-scope diff is empty.

- [ ] **Step 4: Run build-backed checks only if permission was granted**

```bash
colcon build --packages-select livox_ros_driver2 point_lio --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select livox_ros_driver2 --event-handlers console_direct+
colcon test-result --verbose
```

Expected: both packages build and the focused driver tests pass. If permission was not granted, do not run these commands and record that fact verbatim.

- [ ] **Step 5: Record Auditor result**

Mark every checklist item with evidence. Set `PASS` only if all permitted static checks pass and no known
code issue remains; clearly state that hardware/runtime behavior was not verified. Set `NEEDS_FIX` for any
unresolved static or code defect.

- [ ] **Step 6: Commit the completed record**

```bash
git add -f docs/ai_refactor_records/20260705_driver_pointlio_single_dual_adaptation.md
git commit -m "docs: audit lidar adaptation"
```

## Final Completion Audit

- [ ] Single mode uses `multi_topic=0`, merge false, `MID360_config.json`, `livox/lidar`, and `livox/imu`.
- [ ] `multi_topic=0` plus merge true warns once and never configures the merger.
- [ ] Dual mode uses `multi_topic=1`, merge true, configured front/back IPs, and merged `livox/lidar`.
- [ ] Dual Point-LIO uses `livox/imu_192_168_1_135`; both original IMU topics remain available.
- [ ] Merge validation runs only in effective mode and covers every approved parameter invariant.
- [ ] Merged messages still publish their existing `std::unique_ptr` exactly once.
- [ ] Existing fixed-capacity queues are unchanged and queue elements are not accessed after pop.
- [ ] Failed initialization does not start poll threads.
- [ ] Exit wakes semaphore waits; destructor checks pointers and joinability.
- [ ] Common/single/mixed launch files keep one multi-threaded component container and intra-process for both components.
- [ ] Point-LIO C++, `mid360.yaml`, planner, controller, MPC, and ROGMap have no task diff.
- [ ] The audit record lists build/runtime checks not executed because permission or hardware was unavailable.
