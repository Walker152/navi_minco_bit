#pragma once
#ifndef __PROTOCOL_HPP__
#define __PROTOCOL_HPP__
#include <cstddef>
#include <cstdint>
#include <cstring>
// #include <SentryGimbal.pb.h>

// 报文类型定义
enum PacketTypeEnum
{
    ENUM_PACKET_HEART_BEAT = 0,
    ENUM_PACKET_SENTRY_ACCEPT_STATUS_DATA = 1,
    ENUM_PACKET_ARMOR_DATA = 2,
    ENUM_PACKET_SENTRY_SEND_CONTROL_DATA, // 哨兵--装甲板信息
    ENUM_PACKET_GIMBAL_RESET,             // 云台复位
    ENUM_PACKET_RADAR_SMALL_MAP,          // 雷达发送的消息
    ENUM_PACKET_OBSTACLE_DATE,            // 障碍快信息
    ENUM_PACKET_UNDEFINED,
    ENUM_PACKET_INFANTRY_ACCEPT_STATUS_DATA,
    ENUM_PACKET_BUFF_DATA,
    ENUM_PACKET_LOCATION,
    ENUM_PACKET_CMDCONTROL,
    ENUM_PACKET_DETECTOR_DATA,
    ENUM_PACKET_GAMESTATUS_DATA,
    ENUM_PACKET_NAV_DATA
};

#pragma pack(push, 1) // 设置内存对齐格式为1个字节
struct PacketHeader
{
    uint8_t start1 = 0xa5;
    uint8_t start2 = 0x5a;
    uint8_t from;
    uint8_t to;
    uint8_t packet_type;
    uint8_t data_len; // 数据字段长度
    uint16_t checksum = 0;

    PacketHeader() {};

    PacketHeader(PacketTypeEnum p_type, uint8_t _data_len)
        : start1(0xa5), start2(0x5a),
          from(7), to(0),
          packet_type(static_cast<uint8_t>(p_type)),
          data_len(_data_len - sizeof(PacketHeader)), checksum(0)
    {
    }

    /**
     * @brief 设置to字段
     * @param _to [in] 需要发送给谁
     */
    void setTo(const ArmEnum _to) { to = static_cast<int>(_to); }

    /**
     * @brief 设置数据段长度
     * @param _data_len [in] 数据长度
     */
    void setDataLen(uint8_t _data_len) { data_len = _data_len; }

    PacketTypeEnum type() const { return PacketTypeEnum(packet_type); }

    bool check() const
    {
        return (start1 == uint8_t(0xa5)) && (start2 == uint8_t(0x5a));
    }
};

#pragma pack(pop)

static inline uint16_t calChecksum(const char *__data, const size_t __len)
{
    uint16_t my_checksum = 0;
    const char *ptr = __data;
    for (int i = 0; i < __len / 2; i++)
    {
        my_checksum += *((uint16_t *)ptr);
        ptr += 2;
        // printf("check %d:%x ",i,my_checksum);
    }
    if (__len & 0x01)
    {
        uint16_t tmp = uint16_t(__data[__len - 1]);
        my_checksum += tmp;
    }
    // printf("checksum:%x\n",my_checksum);

    return my_checksum;
}
 

typedef uint16_t (*cal_checksum_cb)(const char *data, const size_t data_len);

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
static inline char *packet(PacketHeader *packet_header, const char *data,
                           cal_checksum_cb cb = NULL)
{
    if (cb == NULL)
        cb = calChecksum;
    packet_header->checksum =
        cb(data, packet_header->data_len + sizeof(PacketHeader));

    char *ptr = new char[sizeof(PacketHeader) + packet_header->data_len];

    char *d = ptr;
    memcpy(d, packet_header, sizeof(PacketHeader));
    d += sizeof(PacketHeader);
    memcpy(d, data, packet_header->data_len);

    return ptr;
}

/**
 * @brief 对整条报文进行校验。该字节流含报文头和实际数据部分
 * @param [in] data 对收到的报文字节流进行校验。该字节流包含报文头部分
 * @param [in] data_len data的字节长度
 * @param [in] cb 计算校验值的函数
 * @return true 校验通过；false校验失败
 */
static inline bool check(const char *data, const size_t data_len, cal_checksum_cb cb = NULL)
{

    PacketHeader *header = (PacketHeader *)data;

    if (((int)header->data_len) != (data_len - sizeof(PacketHeader)))
    {

        return false;
    }

    if (!header->check())
        return false;
    if (cb == NULL)
        cb = calChecksum;

    uint16_t checksum = *((uint16_t *)(data + sizeof(PacketHeader) - 2));
    uint16_t my_checksum = cb(data, data_len);
    if (((checksum << 1) & 0xffff) != my_checksum)
        return false;

    return true;
}

#endif
