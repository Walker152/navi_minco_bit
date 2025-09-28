#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include "custom_protocol.hpp"

// 设置结构体字节对齐为1字节，避免内存填充
#pragma pack(push, 1)

// 数据包-包头结构体
struct PacketHeader {
  // 
  uint8_t start1 = 0xa5;           // 起始符1，固定0xA5
  uint8_t start2 = 0x5a;           // 起始符2，固定0x5A
  uint8_t from = ENUM_ARM_SENTRY;  // 发送方ID
  uint8_t to = ENUM_ARM_SENTRY;    // 接收方ID
  uint8_t packet_type;             // 报文类型，取自PacketTypeEnum
  uint8_t data_len;                // 数据段长度，单位为字节
  uint16_t checksum;               // 校验和，计算包头+数据区
  // 包头构造函数
  // ？？？
  PacketHeader(PacketTypeEnum p_type, uint8_t _data_len)
      : start1(),
        start2(),
        from(),
        to(),
        packet_type(static_cast<uint8_t>(p_type)),
        // data_len = 传入长度 - 包头大小，表示仅数据长度
        data_len(_data_len - sizeof(PacketHeader)),
        checksum(0) {}

  //   设置目标类型
  // STM32 or Radar
  void setTo(const ArmEnum _to) { to = static_cast<int>(_to); }

  //   设置数据长度
  void setDataLen(uint8_t _data_len) { data_len = _data_len; }

  //   报文类型
  PacketTypeEnum type() const { return PacketTypeEnum(packet_type); }

  //   校验数据起始位 0xa5 0x5a
  bool check() const {
    return (start1 == uint8_t(0xa5)) && (start2 == uint8_t(0x5a));
  }
};

#pragma pack(pop)

//  CRC校验和函数，用于校验数据正确性
//  __data 待校验数据指针
//  __len  数据长度（字节数）

static inline uint16_t calChecksum(const char* __data, const size_t __len) {
  uint16_t my_checksum = 0;
  const char* ptr = __data;
  // 以2个字节为单位累加求和
  for (int i = 0; i < __len / 2; i++) {
    // 将当前2字节数据强制转换为uint16_t，累加
    my_checksum += *((uint16_t*)ptr);
    ptr += 2;
    // 可选调试输出：printf("check %d:%x ",i,my_checksum);
  }
  // 若长度为奇数，最后单字节加到校验和
  if (__len & 0x01) {
    uint16_t tmp = uint16_t(__data[__len - 1]);
    my_checksum += tmp;
  }
  // 可选调试输出：printf("checksum:%x\n",my_checksum);

  return my_checksum;
}

// 校验函数指针类型定义，用于提供校验回调
using cal_checksum_cb = uint16_t (*)(const char* data, const size_t data_len);
/**
 * @brief 将数据和包头打包成完整的报文字节流
 *
 * @param [in,out] packet_header 指向预先构造好的包头（除校验码外）
 * @param [in] data 待发送的数据体指针
 * @param [in] cb 校验和计算函数，如果为空指针使用默认calChecksum
 * @return char* 返回新分配的字节流指针，调用者负责手动释放delete[]
 *
 * 说明：
 *     1) 先用回调函数计算校验码，填充包头checksum字段
 *     2) 动态分配一块内存，连续存放包头和数据区
 *     3) 复制包头和数据内容进去返回
 */
static inline char* packet(PacketHeader* packet_header, const char* data,
                           cal_checksum_cb cb = nullptr) {
  if (cb == nullptr)
    cb = calChecksum;

  // 计算校验和，校验范围含包头和数据区
  packet_header->checksum =
      cb(data, packet_header->data_len + sizeof(PacketHeader));

  // 动态分配内存，空间足够包头和数据部分
  char* ptr = new char[sizeof(PacketHeader) + packet_header->data_len];

  char* d = ptr;
  // 复制包头数据（含校验码）
  memcpy(d, packet_header, sizeof(PacketHeader));
  d += sizeof(PacketHeader);
  // 复制数据体
  memcpy(d, data, packet_header->data_len);

  return ptr;
}

/**
 * @brief 校验接收到的完整数据包是否有效
 *
 * @param [in] data 指向数据包字节流（包含包头+数据）
 * @param [in] data_len 数据包长度（字节）
 * @param [in] cb 校验和计算函数回调，默认为nullptrptr使用内置函数
 * @return true 校验通过
 * @return false 校验失败
 *
 * 说明：
 *     1) 先强制转换指针为包头，做包长度和起始符校验
 *     2) 从尾部读取包内校验和，与重新计算出来的校验和对比
 */
static inline bool check(const char *data, const size_t data_len, cal_checksum_cb cb = nullptr)
{
    // 将字节流解释为包头结构体指针，方便访问字段
    PacketHeader *header = (PacketHeader *)data;

    // 判断包头中的数据长度是否和实际去除包头长度相等，不等直接丢弃
    if (((int)header->data_len) != (data_len - sizeof(PacketHeader)))
    {
        return false;
    }

    // 检查包头起始符是否正确
    if (!header->check())
        return false;

    // 选择校验函数，默认使用内置的calChecksum
    if (cb == nullptr)
        cb = calChecksum;

    // 从包尾阶段取出传输过来的校验和（位于包头checksum字段）
    uint16_t checksum = *((uint16_t *)(data + sizeof(PacketHeader) - 2));

    // 重新计算校验和
    uint16_t my_checksum = cb(data, data_len);

    // 核心校验逻辑
    // 将传输校验和左移1位并截断16位，与计算校验和比较是否相等
    // 若不等，校验失败
    if (((checksum << 1) & 0xffff) != my_checksum)
        return false;

    return true;
}
