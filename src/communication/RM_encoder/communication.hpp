#ifndef _COMMUNICATION_
#define _COMMUNICATION_
// 引入所需的头文件，包含自定义工具库及ROS相关消息定义
#include "../my-utils/MyUtils/DataType/ByteArray.hpp"         // 自定义字节数组类型
#include "../my-utils/MyUtils/Net/FdManager.hpp"              // 文件描述符管理器（事件驱动）
#include "../my-utils/MyUtils/Net/SerialPort.hpp"             // 串口操作封装
#include "../my-utils/MyUtils/Thread/ThreadManager.hpp"       // 线程统一管理
#include "../my-utils/MyUtils/Thread/ThreadSafeContainer.hpp" // 线程安全容器，未具体用到
#include "../my-utils/MyUtils/Timer/Timer.hpp"                // 定时器管理
#include <iostream>
#include <map>
#include <thread>
#include <stdio.h>

#include "custom_protocol.hpp"             // 自定义协议结构体和枚举定义
#include "protocol.hpp"                    // 可能包含通信协议常量等定义
#include <rclcpp/rclcpp.hpp>               // ROS2暂时注释
#include "robots_msgs/msg/motor_speed.hpp" // 自定义电机速度消息
#include "robots_msgs/msg/nav.hpp"         // 自定义导航消息

using namespace std;

// ----------------------------
// 模板函数，静态返回T类型常量引用（用于全局状态查询？）
// 因为内部static，fdb_data只构造一次，状态保持
// ----------------------------
template <typename T>
const T &getStatus()
{
    static T fdb_data;
    return fdb_data;
}

// ----------------------------
// Communication 类，封装串口通信、定时器和消息解析等功能
// ----------------------------
class Communication
{
    // 简化ByteArray类型访问
    typedef MyUtils::DataType::ByteArray ByteArray;

private:
    // 静态成员变量，文件描述符管理器，管理串口fd和读取回调
    static MyUtils::Net::FdManager fd_manager;

    // 原子变量，用于多线程安全，序列号相关，外部需要初始化
    static std::atomic<uint16_t> arm_seq_num;

    // 穿刺序号，普通无锁变量
    static uint16_t puncture_seq_num;

    // 定时器管理器，管理周期回调执行
    static MyUtils::MyTimer::TimerManager timer_manager;

// 常量定义串口名称，方便统一管理
#define STM32_NAME "stm32"
#define RADAR_NAME "radar"

    // 串口设备路径，允许启动时修改
    static std::string STM32_PORT;
    static std::string RADAR_PORT;

public:
    /**
     * @brief 初始化通信环境和串口连接
     *
     * 说明：该函数内部启动管理器和线程，不需另起线程调用
     */
    static void init()
    {
        // 添加定时器定期尝试打开STM32串口，周期1000ms，循环执行，用于定时重连
        timer_manager.addTimer(1000, true, []()
                               { Communication::__open(STM32_NAME, STM32_PORT, stm32_read_cb, 115200); });

        // 启动fd_manager，管理串口事件，放入线程并启动
        std::thread fd_manager_thread([&]()
                                      { fd_manager.run(); });

        // 线程加入统一线程管理器，方便统一回收、管理线程
        MyUtils::Thread::ThreadManager::getInstance().add(fd_manager_thread.native_handle());
        fd_manager_thread.detach(); // 分离线程，独立运行

        // 启动timer_manager，管理定时事件，放入线程并启动
        std::thread timer_manager_thread([&]()
                                         { timer_manager.run(); });

        MyUtils::Thread::ThreadManager::getInstance().add(timer_manager_thread.native_handle());
        timer_manager_thread.detach();

        return;
    }

    /**
     * @brief 发送底盘运动目标数据到STM32
     * @param data_packet 底盘目标结构体数据
     * @return 发送结果，一般为写入字节数或错误码
     *
     * 说明：构造协议包头并发送数据包（使用__send2stm32内部函数）
     */
    inline static int send2stm32(const _ChassisTarget &data_packet)
    {
        // 构造包头，设置为装甲数据包类型（暂时无实际意义，后续覆盖为导航包）
        PacketHeader header(ENUM_PACKET_ARMOR_DATA,
                            sizeof(PacketHeader) + sizeof(_ChassisTarget));

        // 设置包类型为导航数据包
        header.packet_type = static_cast<uint8_t>(ENUM_PACKET_NAV_DATA);

        // 发送者ID设为哨兵
        header.from = static_cast<uint8_t>(ArmEnum::ENUM_ARM_SENTRY);
        std::cout << "SUCCESS!" << endl;
        // 接收者ID设为下位机（电控）
        header.setTo(ArmEnum::ENUM_ARM_SLAVE_COMPUTER);

        // 置数据长度为底盘目标结构体大小
        header.setDataLen(sizeof(data_packet));

        // 通过串口发送（带上包头）
        return __send2stm32(header, &data_packet);
    }

private:
    /**
     * @brief 私有函数，打开指定串口设备并添加到fd_manager管理
     * @param name 设备名称标识（如"stm32"）
     * @param port 串口设备路径（如"/dev/ttyACM0"）
     * @param read_cb 读取数据的回调函数（收到数据触发）
     * @param _baud_rate 波特率，默认9600
     * @param _n_bits 数据位，默认8
     * @param _stop_length 停止位，默认1
     * @param _check_type 校验位，默认无校验'N'
     */
    static void __open(const string &name, const string &port,
                       MyUtils::Net::fd_read_cb read_cb, int _baud_rate = 9600,
                       int _n_bits = 8, int _stop_length = 1,
                       char _check_type = 'N')
    {
        // 如果已存在对应fd，则不重复打开
        if (fd_manager.exist(name))
            return;

        // 调用串口模块打开设备，返回fd，失败返回-1
        int fd = MyUtils::Net::SerialPort::open(port, _baud_rate, _n_bits,
                                                _stop_length, _check_type);

        if (fd != -1)
        {
            try
            {
                // 将打开的fd加入fd_manager管理，并绑定读回调函数
                fd_manager.add(name, fd, read_cb);
                std::cout << "Successfully added " << name << " with fd " << fd << std::endl;
            }
            catch (const char *e)
            {
                std::cerr << "Error adding " << name << ": " << e << std::endl;
            }
        }
        else
        {
            std::cerr << "Failed to open serial port for " << name << std::endl;
        }
    }

    /**
     * @brief 私有函数，通过fd_manager发送协议数据包到STM32
     * @param header 包头（包含协议元信息）
     * @param data 指向数据包体
     * @return 发送结果（写入字节数或错误码）
     *
     * 说明：
     *  1) 负责计算包头checksum，累加包头中每两个字节的16位值（减去最后一对）
     *  2) 计算数据部分校验码并累加
     *  3) 组装最终字节数组发送
     */
    static int __send2stm32(PacketHeader &header, const void *data)
    {
        // 取包头指针，方便计算checksum
        char *header_p = (char *)&header;

        // 初始化校验和
        header.checksum = 0;

        // 依次累加包头中每两个字节的16bit值（除了最后一对，避免重复计算checksum自身）
        for (int i = 0; i < sizeof(PacketHeader) / 2 - 1; i++)
        {
            header.checksum += *(uint16_t *)(header_p + 2 * i);
        }

        // 对包体数据计算校验码并累加到包头checksum
        header.checksum += calChecksum((char *)data, header.data_len);

        // 先用包头构造字节数组
        MyUtils::DataType::ByteArray arr(&header, sizeof(PacketHeader));

        // 追加包体数据
        arr.append(data, header.data_len);

        // 通过fd_manager发送最终数据流到指定设备
        int ret = fd_manager.send(STM32_NAME, (char *)arr.get(), arr.size());
        return ret;
    }

    // STM32串口读取回调，声明在外部实现
    static void stm32_read_cb(ByteArray arr);

    // 雷达串口读取回调，功能暂不实现，仅占位
    static void radar_read_cb(ByteArray arr)
    {
        static MyUtils::DataType::ByteArray radar_recv_buffer;
        return;
    }

    /**
     * @brief 导航数据发布函数，将接收到的导航结构体转为ROS消息发布
     * @param msg 指向导航数据结构体
     *
     * 说明：
     *  1) 使用ROS发布器将数据发布至主题`/NavRequest`
     *  2) 通过getStatus获得全局可访问的ROS消息实例，以便多线程安全访问
     */

    static void nav_publish(const NavRes *msg)
    {
        // 获取全局消息引用，这里示范自己创建变量
        static robots_msgs::msg::Nav nav_data;

        // 复制数据字段
        nav_data.target_x = msg->x;
        nav_data.target_y = msg->y;

        // 设置导航模式枚举，示例为单点模式
        nav_data.nav_mode = robots_msgs::msg::Nav::MODE_SINGLE_POINT;

        // 设置时间戳为当前ROS2时间
        nav_data.header.stamp = rclcpp::Clock().now();

        // 静态节点和发布者，只创建一次
        static auto nh_ = rclcpp::Node::make_shared("nav_publisher_node");
        static auto nav_pub = nh_->create_publisher<robots_msgs::msg::Nav>("/NavRequest", 10);

        // 发布消息
        nav_pub->publish(nav_data);
    }
};

#endif