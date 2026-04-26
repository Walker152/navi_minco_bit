#include "com.hpp"
#include "com_interface_ros.hpp"
#include <chrono>
#include <cstdint>

// 日志输出落地函数（由头文件模板调用）
void com_log::log_info_line(std::string_view text)
{
  std::cout << text;
}

namespace ns_com {
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
  timer_manager.addTimer(1000, true, []() {
    Communication::__open(STM32_NAME, STM32_PORT, stm32_read_cb, 115200);
  });

  std::thread([]() {
    fd_manager.run();
  }).detach();
  std::thread([]() {
    timer_manager.run();
  }).detach();
}

void Communication::setRosInterface(const std::shared_ptr<ComInterfaceRos> & ptr)
{
  ros_if_ = ptr;
}

// 打开串口
void Communication::__open(const std::string & name,
  const std::string & port,
  MyUtils::Net::fd_read_cb read_cb,
  int baud_rate,
  int n_bits,
  int stop_length,
  char check_type)
{
  if (fd_manager.exist(name))
    return;
  int fd = MyUtils::Net::SerialPort::open(port, baud_rate, n_bits, stop_length, check_type);
  if (fd != -1) {
    try {
      fd_manager.add(name, fd, read_cb);
      LOG_DEBUG_BLOCK(std::string(BLUE) + "[COM] ", NV(name), NV(fd));
    } catch (const char * e) {
      std::cerr << RED << "[COM] Error adding " << name << ": " << e << RESET << std::endl;
    }
  } else {
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
int Communication::__send2stm32(PacketHeader & header, const void * data)
{
  header.checksum = 0;
  char * header_p = reinterpret_cast<char *>(&header);
  for (size_t i = 0; i < sizeof(PacketHeader) / 2 - 1; ++i) {
    header.checksum += *(uint16_t *)(header_p + 2 * i);
  }
  header.checksum += calChecksum((char *)data, header.data_len);

  MyUtils::DataType::ByteArray arr(&header, sizeof(PacketHeader));
  arr.append(data, header.data_len);

  return fd_manager.send(STM32_NAME, (char *)arr.get(), arr.size());
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
#ifdef COM_DEBUG
  auto now_tp = Clock::now();
  if (now_tp - last_debug_tp >= std::chrono::milliseconds(300)) {
    allow_debug = true;
    last_debug_tp = now_tp;
  }
#endif

  while (true) {
    // 错误处理
    if (stm32_recv_buffer.size() < 2)
      return;

    char * buf = (char *)stm32_recv_buffer.get();

    int start_idx = -1;
    for (size_t i = 0; i + 1 < stm32_recv_buffer.size(); ++i) {
      if ((uint8_t)buf[i] == 0xA5 && (uint8_t)buf[i + 1] == 0x5A) {
        start_idx = i;
        break;
      }
    }
    if (start_idx == -1) {
      std::cout << RED << "[COM] Warning: No valid start flag found in STM32 buffer, clearing buffer."
                << RESET << std::endl;
      stm32_recv_buffer.reset();
      return;
    }
    if (start_idx > 0) {
      stm32_recv_buffer = stm32_recv_buffer.sub(start_idx, stm32_recv_buffer.size() - start_idx);
      buf = (char *)stm32_recv_buffer.get();
    }
    if (stm32_recv_buffer.size() < sizeof(PacketHeader)) {
      return;
    }
    PacketHeader * header = (PacketHeader *)buf;
    size_t full_len = sizeof(PacketHeader) + header->data_len;

    if (stm32_recv_buffer.size() < full_len) {
      return;
    }
    if (check(buf, full_len)) {
      if (allow_debug) {
        LOG_DEBUG_BLOCK(std::string(BLUE) + "[COM] ", NV(header->packet_type), NV(header->data_len));
      }
      // std::cout << static_cast<int>(header->packet_type) << std::endl;
      switch (header->packet_type) {
        // 校验通过，根据报文类型解析数据并发布ROS消息
      case ENUM_PACKET_ALLY_STATUS: {
        const TeamInfo * team_data = (const TeamInfo *)(buf + sizeof(PacketHeader));
        if (allow_debug) {
          LOG_DEBUG_BLOCK(std::string(REDPURPLE) + "[COM][Team] ",
            NV(static_cast<int>(team_data->ally_status[0].robot_id)),
            NV(team_data->ally_status[0].robot_hp),
            NV(team_data->ally_status[0].robot_pos_x),
            NV(team_data->ally_status[0].robot_pos_y),
            NV(static_cast<int>(team_data->ally_status[1].robot_id)),
            NV(team_data->ally_status[1].robot_hp),
            NV(team_data->ally_status[1].robot_pos_x),
            NV(team_data->ally_status[1].robot_pos_y),
            NV(static_cast<int>(team_data->ally_status[2].robot_id)),
            NV(team_data->ally_status[2].robot_hp),
            NV(team_data->ally_status[2].robot_pos_x),
            NV(team_data->ally_status[2].robot_pos_y),
            NV(static_cast<int>(team_data->ally_status[3].robot_id)),
            NV(team_data->ally_status[3].robot_hp),
            NV(team_data->ally_status[3].robot_pos_x),
            NV(team_data->ally_status[3].robot_pos_y),
            NV(team_data->outpost_hp),
            NV(team_data->base_hp)

          );
        }
        auto ros_ptr = ros_if_;
        if (ros_ptr)
          ros_ptr->publishTeamInfo(*team_data);
        break;
      }

      case ENUM_PACKET_GAMESTATUS_DATA: {
        // std::cout << YELLOW << "[COM] Received GameInfo packet." << RESET << std::endl;
        const GameInfo * game_data = (const GameInfo *)(buf + sizeof(PacketHeader));

        if (allow_debug) {
          LOG_DEBUG_BLOCK(std::string(REDPURPLE) + "[COM][Game] ",
            NV(static_cast<int>(game_data->coin_remaining)),
            NV(static_cast<int>(game_data->event_code)),
            NV(static_cast<int>(game_data->game_status)),
            NV(static_cast<int>(game_data->game_time_remaining)));
        }
        auto ros_ptr = ros_if_;
        if (ros_ptr)
          ros_ptr->publishGameInfo(*game_data);
        break;
      }

      case ENUM_PACKET_SENTRY_SERVER_DATA: {
        const SentryInfoOnline * sentry_server_data =
          (const SentryInfoOnline *)(buf + sizeof(PacketHeader));
        if (allow_debug) {
          LOG_DEBUG_BLOCK(std::string(REDPURPLE) + "[COM][Sol] ",
            NV(sentry_server_data->bullets_remaining),
            NV(sentry_server_data->cooling_value),
            NV(sentry_server_data->current_heat),
            NV(sentry_server_data->heat_limit),
            NV(sentry_server_data->self_health),
            NV(sentry_server_data->sentry_info_1),
            NV(sentry_server_data->sentry_info_2),
            NV(sentry_server_data->sentry_pos_x),
            NV(sentry_server_data->sentry_pos_y),
            NV(sentry_server_data->speed_monitor_angle));
        }
        auto ros_ptr = ros_if_;
        if (ros_ptr)
          ros_ptr->publishSentryInfoOnline(*sentry_server_data);
        break;
      }

      case ENUM_PACKET_SENTRY_SELF_DATA: {
        const SentryInfoOffline * sentry_self_data =
          (const SentryInfoOffline *)(buf + sizeof(PacketHeader));
        if (allow_debug) {
          LOG_DEBUG_BLOCK(std::string(REDPURPLE) + "[COM][Sof] ",
            NV(sentry_self_data->armor_num),
            NV(static_cast<int>(sentry_self_data->armor_pos[0])),
            NV(static_cast<int>(sentry_self_data->armor_pos[1])),
            NV(static_cast<int>(sentry_self_data->armor_pos[2])),
            NV(sentry_self_data->is_get),
            NV(sentry_self_data->is_transformable),
            NV(sentry_self_data->lifter_current_pos),
            NV(sentry_self_data->transform_state),
            NV(sentry_self_data->yaw_imu),
            NV(sentry_self_data->capacitor_capacity));
        }
        auto ros_ptr = ros_if_;
        if (ros_ptr)
          ros_ptr->publishSentryInfoOffline(*sentry_self_data);
        break;
      }

      case ENUM_PACKET_RADAR: {
        const RadarInfo * radar_data = (const RadarInfo *)(buf + sizeof(PacketHeader));
        // TODO:还没写完调试日志输出，待写
        if (allow_debug) {
          LOG_DEBUG_BLOCK(std::string(REDPURPLE) + "[COM][Rdr] ",
            NV(static_cast<int>(radar_data->enemy_status[0].robot_id)),
            NV(radar_data->enemy_status[0].robot_hp),
            NV(radar_data->enemy_status[0].allowed_projectile),
            NV(radar_data->enemy_status[0].robot_pos_x),
            NV(radar_data->enemy_status[0].robot_pos_y),
            NV(static_cast<int>(radar_data->enemy_status[1].robot_id)),
            NV(radar_data->enemy_status[1].robot_hp),
            NV(radar_data->enemy_status[1].allowed_projectile),
            NV(radar_data->enemy_status[1].robot_pos_x),
            NV(radar_data->enemy_status[1].robot_pos_y),
            NV(static_cast<int>(radar_data->enemy_status[2].robot_id)),
            NV(radar_data->enemy_status[2].robot_hp),
            NV(radar_data->enemy_status[2].allowed_projectile),
            NV(radar_data->enemy_status[2].robot_pos_x),
            NV(radar_data->enemy_status[2].robot_pos_y),
            NV(static_cast<int>(radar_data->enemy_status[3].robot_id)),
            NV(radar_data->enemy_status[3].robot_hp),
            NV(radar_data->enemy_status[3].allowed_projectile),
            NV(radar_data->enemy_status[3].robot_pos_x),
            NV(radar_data->enemy_status[3].robot_pos_y),
            NV(static_cast<int>(radar_data->enemy_status[4].robot_id)),
            NV(radar_data->enemy_status[4].robot_hp),
            NV(radar_data->enemy_status[4].allowed_projectile),
            NV(radar_data->enemy_status[4].robot_pos_x),
            NV(radar_data->enemy_status[4].robot_pos_y),
            NV(static_cast<int>(radar_data->enemy_status[5].robot_id)),
            NV(radar_data->enemy_status[5].robot_hp),
            NV(radar_data->enemy_status[5].allowed_projectile),
            NV(radar_data->enemy_status[5].robot_pos_x),
            NV(radar_data->enemy_status[5].robot_pos_y),
            NV(radar_data->enemy_coin_accumulated),
            NV(radar_data->enemy_coin_left),
            NV(radar_data->is_enemy_outpost_sensed));
        }
        auto ros_ptr = ros_if_;
        if (ros_ptr)
          ros_ptr->publishRadarInfo(*radar_data);
        break;
      }

      default: {
        if (allow_debug) {
          LOG_DEBUG_BLOCK(std::string(YELLOW) + "[COM][Warn] ", NV(header->packet_type));
        }
      }
      }
    } else {
      std::cout << RED << "[COM] Warning: Checksum error (Packet type: " << (int)header->packet_type << ")"
                << RESET << std::endl;
    }
    stm32_recv_buffer = stm32_recv_buffer.sub(full_len);
  }
}

}  // namespace ns_com