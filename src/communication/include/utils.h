#ifndef COMMUNICATION_UTILS
#define COMMUNICATION_UTILS
#include <stdint.h>
#include <vector>
// -------------------------
// 模板union：用于类型与字节数组之间转换
// -------------------------
template <typename T>
union ExchangeData {
  T data;                             // 泛型数据成员
  uint8_t buffer[sizeof(T)];          // 同内存空间的字节数组
  /*
   * 作用：
   *  - 方便在网络或串口传输时将结构体与字节流互转
   *  - 读取或写入buffer即操作同一内存的data
   */
};
// -------------------------
// 云台控制结构体，表示目标和姿态控制指令
// 使用GCC属性指定结构体紧凑排列，避免内存填充，确保协议一致性
// -------------------------
struct __attribute__((packed, aligned(1))) TinyGimbal {
  float target_x;      // 目标点X坐标（单位：米）
  float target_y;      // 目标点Y坐标
  float target_z;      // 目标点Z坐标
  float yaw;           // 云台偏航角（角度或弧度，根据协议）
  float pitch;         // 云台俯仰角
  int16_t armor_id;    // 识别的装甲板ID
  bool is_shoot_vision;// 是否启用视觉辅助射击（布尔值）
  int shoot_frequncy;  // 射击频率（单位：可自定义，举例每秒发射次数）
  bool is_get;         // 是否已获取目标（布尔值）
};

// --------------------------
// 导航单元结构体，可能用于传递机器人颜色信息等识别标识
// 同样保证紧凑对齐，节省空间
// --------------------------
struct __attribute__((packed, aligned(1))) NaviNuc {
  int16_t my_color;    // 颜色编码，值域根据具体应用定义
};

#endif