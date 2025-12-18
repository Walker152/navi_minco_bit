#include "com.hpp"
#include "com_interface_ros.hpp"

// 日志输出落地函数（由头文件模板调用）
void com_log::log_info_line(std::string_view text)
{
  std::cout << text;
}

namespace ns_com
{
  // 静态成员变量定义
  MyUtils::Net::FdManager Communication::fd_manager;
  MyUtils::MyTimer::TimerManager Communication::timer_manager;
  std::atomic<uint16_t> Communication::arm_seq_num{0};
  uint16_t Communication::puncture_seq_num = 0;
  std::string Communication::STM32_PORT = "/dev/ttyACM0";
  std::shared_ptr<ComInterfaceRos> Communication::ros_if_{nullptr};

  // 初始化通信模块
  void Communication::init()
  {
    timer_manager.addTimer(1000, true, []() { Communication::__open(STM32_NAME, STM32_PORT, stm32_read_cb, 115200); });

    std::thread([]() { fd_manager.run(); }).detach();
    std::thread([]() { timer_manager.run(); }).detach();
  }

  void Communication::setRosInterface(const std::shared_ptr<ComInterfaceRos>& ptr)
  {
    ros_if_ = ptr;
  }

  // 打开串口
  void Communication::__open(const std::string& name,
                             const std::string& port,
                             MyUtils::Net::fd_read_cb read_cb,
                             int baud_rate,
                             int n_bits,
                             int stop_length,
                             char check_type)
  {
    if(fd_manager.exist(name))
      return;
    int fd = MyUtils::Net::SerialPort::open(port, baud_rate, n_bits, stop_length, check_type);
    if(fd != -1)
    {
      try
      {
        fd_manager.add(name, fd, read_cb);
        LOG_DEBUG_BLOCK(std::string(BLUE) + "[COM] ", NV(name), NV(fd));
      }
      catch(const char* e)
      {
        std::cerr << RED << "[COM] Error adding " << name << ": " << e << RESET << std::endl;
      }
    }
    else
    {
      std::cerr << RED << "[COM] Failed to open serial port for " << name << RESET << std::endl;
      LOG_DEBUG_BLOCK(std::string(YELLOW) + "[COM] ",
                      NV(name),
                      NV(port),
                      NV(baud_rate),
                      NV(n_bits),
                      NV(stop_length),
                      NV(check_type));
    }
  }
  // send2stm32 模板定义已放入头文件（com.hpp），此处仅保留 __send2stm32 的实现
  // 计算
  int Communication::__send2stm32(PacketHeader& header, const void* data)
  {
    header.checksum = 0;
    char* header_p = reinterpret_cast<char*>(&header);
    for(size_t i = 0; i < sizeof(PacketHeader) / 2 - 1; ++i)
    {
      header.checksum += *(uint16_t*)(header_p + 2 * i);
    }
    header.checksum += calChecksum((char*)data, header.data_len);

    MyUtils::DataType::ByteArray arr(&header, sizeof(PacketHeader));
    arr.append(data, header.data_len);

    return fd_manager.send(STM32_NAME, (char*)arr.get(), arr.size());
  }

  // STM32串口数据读取回调
  void Communication::stm32_read_cb(ByteArray arr)
  {
    static ByteArray stm32_recv_buffer;
    stm32_recv_buffer.append(arr.get(), arr.size());

    // 节流：仅每秒输出一次详细调试信息
    using Clock = std::chrono::steady_clock;
    static auto last_debug_tp = Clock::now();
    bool allow_debug = false;
    auto now_tp = Clock::now();
    if(now_tp - last_debug_tp >= std::chrono::seconds(1))
    {
      allow_debug = true;
      last_debug_tp = now_tp;
    }

    while(true)
    {
      // 错误处理
      if(stm32_recv_buffer.size() < 2)
        return;

      char* buf = (char*)stm32_recv_buffer.get();

      int start_idx = -1;
      for(size_t i = 0; i + 1 < stm32_recv_buffer.size(); ++i)
      {
        if((uint8_t)buf[i] == 0xA5 && (uint8_t)buf[i + 1] == 0x5A)
        {
          start_idx = i;
          break;
        }
      }
      if(start_idx == -1)
      {
        std::cout << RED << "[COM] Warning: No valid start flag found in STM32 buffer, clearing buffer." << RESET
                  << std::endl;
        stm32_recv_buffer.reset();
        return;
      }
      if(start_idx > 0)
      {
        stm32_recv_buffer = stm32_recv_buffer.sub(start_idx, stm32_recv_buffer.size() - start_idx);
        buf = (char*)stm32_recv_buffer.get();
      }
      if(stm32_recv_buffer.size() < sizeof(PacketHeader))
      {
        return;
      }
      PacketHeader* header = (PacketHeader*)buf;
      size_t full_len = sizeof(PacketHeader) + header->data_len;

      if(stm32_recv_buffer.size() < full_len)
      {
        return;
      }
      if(check(buf, full_len))
      {
        if(allow_debug)
        {
          LOG_DEBUG_BLOCK(std::string(BLUE) + "[COM] ", NV(header->packet_type), NV(header->data_len));
        }
        switch(header->packet_type)
        {
          // 校验通过
        case ENUM_PACKET_NAV_DATA:
        {
          const NavRes* nav_data = (const NavRes*)(buf + sizeof(PacketHeader));
          auto ros_ptr = ros_if_;
          if(ros_ptr)
            ros_ptr->publishNav(*nav_data);
          if(allow_debug)
          {
            LOG_DEBUG_BLOCK(std::string(GREEN) + "[COM][Nav] ",
                            NV(nav_data->x),
                            NV(nav_data->y),
                            NV(nav_data->yaw),
                            NV(nav_data->is_reach));
          }
          break;
        }

        case ENUM_PACKET_GAMESTATUS_DATA:
        {
          const EventStatus* event_status = (const EventStatus*)(buf + sizeof(PacketHeader));
          if(allow_debug)
          {
            LOG_DEBUG_BLOCK(std::string(REDPURPLE) + "[COM][Evt] ",
                            NV(event_status->self_health),
                            NV(event_status->num_shoot),
                            NV(event_status->own_outpost_destroyed),
                            NV(event_status->buff_active),
                            NV(event_status->is_get),
                            NV(event_status->x),
                            NV(event_status->y),
                            NV(event_status->z),
                            NV(event_status->armor_id));
                            NV(event_status->position);
          }
          auto ros_ptr = ros_if_;
          if(ros_ptr)
            ros_ptr->publishEventStatus(*event_status);
          break;
        }
        default:
        {
          if(allow_debug)
          {
            LOG_DEBUG_BLOCK(std::string(YELLOW) + "[COM][Warn] ", NV(header->packet_type));
          }
        }
        }
      }
      else
      {
        std::cout << RED << "[COM] Warning: Checksum error (Packet type: " << (int)header->packet_type << ")" << RESET
                  << std::endl;
      }
      stm32_recv_buffer = stm32_recv_buffer.sub(full_len);
    }
  }

}  // namespace ns_com