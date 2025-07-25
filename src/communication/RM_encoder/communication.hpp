#pragma once
#ifndef _COMMUNICATION_
#define _COMMUNICATION_
#include "../my-utils/MyUtils/DataType/ByteArray.hpp"
#include "../my-utils/MyUtils/Net/FdManager.hpp"
#include "../my-utils/MyUtils/Net/SerialPort.hpp"
#include "../my-utils/MyUtils/Thread/ThreadManager.hpp"
#include "../my-utils/MyUtils/Thread/ThreadSafeContainer.hpp"
#include "../my-utils/MyUtils/Timer/Timer.hpp"
#include <iostream>
#include <map>
#include <thread>
#include <stdio.h>

#include "custom_protocol.hpp"
#include "protocol.hpp"
#include <ros/ros.h>

#include "robots_msgs/Nav.h"
#include "robots_msgs/MotorSpeed.h"

using namespace std;

template <typename T>
const T &getStatus()
{
    static T fdb_data;
    return fdb_data;
}

class Communication
{
    typedef MyUtils::DataType::ByteArray ByteArray;

private:
    static MyUtils::Net::FdManager fd_manager;
    static std::atomic<uint16_t> arm_seq_num;
    static uint16_t puncture_seq_num;
    static MyUtils::MyTimer::TimerManager timer_manager;

#define STM32_NAME "stm32"
#define RADAR_NAME "radar"

    static std::string STM32_PORT;
    static std::string RADAR_PORT;

public:
    /**
     * @brief 该函数不需要放在单独的线程中执行
     */
    static void init()
    {
        // 添加STM读取回调
        timer_manager.addTimer(1000, true, []()
                               { Communication::__open(STM32_NAME, STM32_PORT, stm32_read_cb,
                                                       115200); });

        // 启动 fd_manager
        std::thread fd_manager_thread([&]()
                                      { fd_manager.run(); });

        MyUtils::Thread::ThreadManager::getInstance().add(
            fd_manager_thread.native_handle());
        fd_manager_thread.detach();

        // 启动 timer_manager
        std::thread timer_manager_thread([&]()
                                         { timer_manager.run(); });
        MyUtils::Thread::ThreadManager::getInstance().add(
            timer_manager_thread.native_handle());
        timer_manager_thread.detach();

        // 初始化ros设置

        // static ros::NodeHandle nh_;
        // motor_speed_pub = nh_.advertise<robots_msgs::MotorSpeed>("",10);
        // nav_pub = nh_.advertise<robots_msgs::Nav>("/NavRequest", 10);

        return;
    }

    // template and number sign to be improved
    inline static int send2stm32(const _ChassisTarget &data_packet)
    {
        PacketHeader header(ENUM_PACKET_ARMOR_DATA,
                            sizeof(PacketHeader) + sizeof(_ChassisTarget));
        header.packet_type =
            static_cast<uint8_t>(ENUM_PACKET_NAV_DATA);
        header.from = static_cast<uint8_t>(ArmEnum::ENUM_ARM_SENTRY);
        header.setTo(ArmEnum::ENUM_ARM_SLAVE_COMPUTER);
        header.setDataLen(sizeof(data_packet));
        return __send2stm32(header, &data_packet);
    }

private:
    static void __open(const string &name, const string &port,
                       MyUtils::Net::fd_read_cb read_cb, int _baud_rate = 9600,
                       int _n_bits = 8, int _stop_length = 1,
                       char _check_type = 'N')
    {
        if (fd_manager.exist(name))
            return;

        int fd = MyUtils::Net::SerialPort::open(port, _baud_rate, _n_bits,
                                                _stop_length, _check_type);

        if (fd != -1)
        {
            try
            {
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

    static int __send2stm32(PacketHeader &header, const void *data)
    {

        char *header_p = (char *)&header;
        for (int i = 0; i < sizeof(PacketHeader) / 2 - 1; i++)
        {
            header.checksum += *(uint16_t *)(header_p + 2 * i);
        }

        header.checksum += calChecksum((char *)data, header.data_len);

        MyUtils::DataType::ByteArray arr(&header, sizeof(PacketHeader));
        arr.append(data, header.data_len);

        int ret = fd_manager.send(STM32_NAME, (char *)arr.get(), arr.size());

        return ret;
    }

    static void stm32_read_cb(ByteArray arr);

    static void radar_read_cb(ByteArray arr)
    {
        static MyUtils::DataType::ByteArray radar_recv_buffer;
        return;
    }

    static void nav_publish(const NavRes *msg)
    {
        const robots_msgs::Nav &data = getStatus<robots_msgs::Nav>();
        robots_msgs::Nav &nav_data = const_cast<robots_msgs::Nav &>(data);
        nav_data.target_x = msg->x;
        nav_data.target_y = msg->y;
        nav_data.nav_mode = robots_msgs::Nav::MODE_SINGLE_POINT;
        nav_data.header.stamp = ros::Time::now();
        static ros::NodeHandle nh_;
        static ros::Publisher nav_pub = nh_.advertise<robots_msgs::Nav>("/NavRequest", 10);
        nav_pub.publish(nav_data);
    }
};

#endif
