#include "../RM_encoder/communication.hpp"
#include <fmt/core.h>

// 静态成员变量定义（文件内全局作用域）
MyUtils::Net::FdManager Communication::fd_manager;
MyUtils::MyTimer::TimerManager Communication::timer_manager;

std::string Communication::STM32_PORT("/dev/ttyACM0");

// STM32串口数据读取回调函数，负责解析数据包并进行校验和转发
void Communication::stm32_read_cb(ByteArray arr)
{
    // 使用静态缓存存储读取到的字节流，持续追加新数据
    static MyUtils::DataType::ByteArray stm32_recv_buffer;

    // 将本次收到的数据追加到缓存末尾
    stm32_recv_buffer.append(arr.get(), arr.size());

    // 获取缓冲区内容指针，方便按字节遍历
    char *temp = (char *)stm32_recv_buffer.get();

    // 数据少于2字节，不可能包含完整包头特征，提前返回
    if (stm32_recv_buffer.size() < 2)
    {
        // 替换ROS_WARN为fmt打印（示例）
        fmt::print(stderr, "Warning: Data size less than 2 bytes\n");
        return;
    }

    bool flag = false;

    // 遍历缓冲区寻找数据包起始标志0xA5 0x5A
    for (int i = 0; i < stm32_recv_buffer.size() - 1; i++)
    {
        if (temp[i] == char(0xA5) && temp[i + 1] == char(0x5A))
        {
            // 找到起始符，从当前位置截取（删除之前无关数据）
            stm32_recv_buffer = stm32_recv_buffer.sub(i, stm32_recv_buffer.size() - i);
            flag = true;
            break;
        }
    }

    // 若未找到起始标志，打印警告并返回
    if (!flag)
    {
        fmt::print(stderr, "Warning: Start flag(0xa5/0x5a) not found\n");
        return;
    }

    // 判断缓存大小是否达到一个包头大小，否则数据不完整，等待更多数据
    if (stm32_recv_buffer.size() < sizeof(PacketHeader))
    {
        fmt::print(stderr, "Warning: Data size less than PacketHeader size\n");
        return;
    }

    // 缓存的起始指针
    char *msg = (char *)stm32_recv_buffer.get();

    // 将包头部分转换为PacketHeader结构体指针，方便读取包头字段
    PacketHeader *header = (PacketHeader *)msg;

    // 计算完整包长度 = 包头大小 + 数据区长度
    const int len = sizeof(PacketHeader) + header->data_len;

    // 如果当前缓存数据不够组成一个完整包，等待更多数据
    if (stm32_recv_buffer.size() < len)
    {
        fmt::print(stderr, "Warning: Data size less than full packet length\n");
        return;
    }

    // 指向整个数据包起始处
    char *info_rec = (char *)stm32_recv_buffer.get();

    // 进行包校验，返回true表示校验通过
    if (check(info_rec, len))
    {
        // 校验通过，根据包类型进行不同处理
        switch (header->packet_type)
        {
        
        // 导航数据包处理
        case ENUM_PACKET_NAV_DATA:
        {
            // 数据区紧接包头后
            NavRes *nav_data = (NavRes *)(info_rec + sizeof(PacketHeader));
            nav_publish(nav_data);
            break;
        }

        // 默认不处理其他类型包
        default:
            break;
        };
    }
    else
    {
        // 校验失败，打印错误信息
        fmt::print(stdout, "checksum incorrect\n");
    }

    // 将缓存去除本次处理的完整包数据，保留剩余数据
    stm32_recv_buffer = stm32_recv_buffer.sub(len);
    return;
}