#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace MyUtils {
namespace Net {

enum PacketTypeEnum
{
  ENUM_ARM_RESPONSE = 0x01,
  ENUM_PUNCTURE_RESPONSE = 0x02,
  ENUM_ARM_FROM_PC = 0x03,
  ENUM_PUNCTURE_FROM_PC = 0x04,
  ENUM_PACKET_TYPE_UNDEFINED
};
#pragma pack(push, 1)  //设置内存对齐格式为1个字节
struct PacketHeader
{
  uint8_t start[4] = {0x80, 0xff, 0x0a, 0x0d};
  uint8_t packet_type;  // PacketTypeEnum
  uint8_t data_len;     //数据字段长度
  uint16_t seq_num;     //心跳序号
  PacketHeader() : packet_type(static_cast<uint8_t>(ENUM_PACKET_TYPE_UNDEFINED)), data_len(0){};

  PacketHeader(PacketTypeEnum p_type, uint8_t _data_len)
  : packet_type(static_cast<uint8_t>(p_type)), data_len(_data_len)
  {
  }

  PacketTypeEnum type() const { return PacketTypeEnum(packet_type); }
  bool check() const
  {
    return (this->start[0] == uint8_t(0x80) && this->start[1] == uint8_t(0xff) &&
            this->start[2] == uint8_t(0x0a) && this->start[3] == uint8_t(0x0d));
  }
};
#pragma pack(pop)

uint16_t calChecksum(const char * __data, const size_t __len)
{
  uint16_t my_checksum = 0;
  const char * ptr = __data;
  for (int i = 0; i < __len / 2; i++) {
    my_checksum += *((uint16_t *)ptr);
    ptr += 2;
  }
  if (__len & 0x01) {
    uint16_t tmp = uint16_t(__data[__len - 1]);
    my_checksum += tmp;
  }
  return my_checksum;
}

typedef uint16_t cal_checksum_cb(const char * data, const size_t data_len);
/**
 * @brief 对数据包进行封装。如果需要指定计算checksum的方式请传入callback函数，
 * 否则将使用默认函数计算校验值。值得注意的是，这里没有输入data长度的参数，因为
 * packet_header内含就有了，因此不需要再传入。
 * @param [in|out] packet_header
 * 提前准备好的报文头。除了校验位之外其他的所有数据都应当填充完毕
 * @param [in] data 实际要发送的数据
 * @param [in] cb 计算校验值的回调函数。如果为NULL则使用默认函数
 * @return 返回值是打包好的数据指针，需要用户手动delete
 */
char * packet(PacketHeader * packet_header, const char * data, cal_checksum_cb cb = NULL)
{
  char * ptr = new char[sizeof(PacketHeader) + packet_header->data_len + 2];
  if (cb == NULL)
    cb = calChecksum;

  char * d = ptr;
  memcpy(d, packet_header, sizeof(PacketHeader));
  d += sizeof(PacketHeader);
  memcpy(d, data, packet_header->data_len);
  d += packet_header->data_len;

  uint16_t checksum = cb(ptr, packet_header->data_len + sizeof(PacketHeader));
  memcpy(d, &checksum, 2);
  return ptr;
}

/**
 * @brief 对整条报文进行校验。该字节流含报文头和实际数据部分
 * @param [in] data 对收到的报文字节流进行校验。该字节流包含报文头部分
 * @param [in] data_len data的字节长度
 * @param [in] cb 计算校验值的函数
 * @return true 校验通过；false校验失败
 */
bool check(const char * data, const size_t data_len, cal_checksum_cb cb = NULL)
{
  PacketHeader * header = (PacketHeader *)data;
  if (header->data_len != data_len - sizeof(PacketHeader) - 2)
    return false;
  if (!header->check())
    return false;
  if (cb == NULL)
    cb = calChecksum;

  uint16_t checksum = *((uint16_t *)(data + data_len - 2));
  if (checksum != cb(data, data_len - 2))
    return false;
  return true;
}

}  // namespace Net

}  // namespace MyUtils
