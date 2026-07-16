# Communication 下发数据 CSV 记录器改造记录

## User Intent

为 communication 增加数据记录器，从节点启动到关闭期间，将向下位机发送的消息字段值保存为 CSV。默认目录为 `/tmp/communication_logs`，尽量保留 Ctrl+C、关闭终端、进程异常退出或异常断电前已经记录的数据。

## Scope

- `src/navigation/communication` 的统一 STM32 发送入口。
- 当前实际发送的 `ChassisTarget` 与 `BehaviorData` 字段。
- 不改变串口协议、发送频率、发送顺序、topic、QoS 或行为树逻辑。

## Out of Scope

- 下位机上行数据记录。
- 当前被注释、未实际发送的全局路径分包。
- 对突然断电时仍处于内核或存储设备缓存中的数据提供绝对持久性保证。
- communication 之外的模块修改。

## Explorer Findings

### Files inspected

- `src/navigation/communication/include/com.hpp`
- `src/navigation/communication/src/com.cpp`
- `src/navigation/communication/include/com_interface_ros.hpp`
- `src/navigation/communication/include/utils/custom_protocol.hpp`
- `src/navigation/communication/include/MyUtils/Net/FdManager.hpp`
- `src/navigation/communication/CMakeLists.txt`

### Active logic path

`ComInterfaceRos::communicationLoop()` 每轮构造一个 `ChassisTarget` 和一个 `BehaviorData`，依次调用 `Communication::send2stm32()`。模板最终进入 `Communication::__send2stm32()`，由 `FdManager::send()` 写入串口缓冲区。全局路径发送代码当前处于注释状态。

### Data flow

ROS 订阅状态 → `communicationLoop()` 快照并组包 → `send2stm32()` → `__send2stm32()` → `FdManager::send()` → CSV 记录发送返回值及字段。

### Risk notes

- 通信循环频率高，逐行执行 `fsync` 可能阻塞实时发送，因此仅逐行 `flush` 用户态流。
- 瞬间掉电时，操作系统或存储设备尚未持久化的最后少量数据仍可能丢失。
- 日志目录或文件打开失败不能影响原通信链路。

### Recommended modification boundary

仅在统一发送模板取得发送返回值后调用独立记录器，并在 `Communication::init()` 时初始化记录文件和相对计时起点。

## Modifier Changes

### Files changed

- `src/navigation/communication/include/csv_recorder.hpp`
- `src/navigation/communication/src/csv_recorder.cpp`
- `src/navigation/communication/include/com.hpp`
- `src/navigation/communication/src/com.cpp`
- `src/navigation/communication/CMakeLists.txt`
- `src/navigation/communication/test/test_csv_recorder_static.py`

### Key changes

- 每次启动在 `/tmp/communication_logs` 创建带时间戳的 `sent_messages_*.csv`。
- 统一记录系统时间、启动后毫秒数、包类型、发送返回值、成功标志和消息字段值。
- 两种包使用同一个宽表，不适用字段留空。
- CSV 表头和每行写入后立即 `flush`。
- 目录或文件初始化异常只输出错误，不改变串口发送返回值。

### Behavior preserved

- 串口打包、校验和、写入顺序及返回值保持不变。
- ROS topic、QoS、timer 和 callback group 保持不变。
- 底盘与行为数据字段值及协议布局保持不变。

### Behavior intentionally adjusted

- 每次向下位机发送 `ChassisTarget` 或 `BehaviorData` 后增加一行 CSV 写入与 flush。

### Notes

`send_success=1` 对应 `FdManager::send()` 返回 0；-1 表示写入错误，-2 表示串口名称尚未注册。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查
- [x] grep 检查
- [x] XML / launch / yaml 检查（本次未修改这些文件）
- [x] 用户允许范围内的测试或静态检查
- [ ] 如需构建，已取得用户明确许可

### Issues found

初版相对时间从第一条发送开始计时，已修正为在 `Communication::init()` 时初始化记录器。

### Final result

PASS（静态检查范围内；未执行构建）
