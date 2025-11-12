#include "com.hpp"

namespace ns_com
{

  // 静态成员变量定义
  MyUtils::Net::FdManager Communication::fd_manager;
  MyUtils::MyTimer::TimerManager Communication::timer_manager;
  std::atomic<uint16_t> Communication::arm_seq_num{0};
  uint16_t Communication::puncture_seq_num = 0;
  std::string Communication::STM32_PORT = "/dev/ttyACM0";

  // 初始化通信模块
  void Communication::init()
  {
    // 定时尝试打开串口
    timer_manager.addTimer(1000, true, []() { Communication::__open(STM32_NAME, STM32_PORT, stm32_read_cb, 115200); });

    // 启动fd_manager线程
    std::thread([]() { fd_manager.run(); }).detach();

    // 启动timer_manager线程
    std::thread([]() { timer_manager.run(); }).detach();
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
        std::cout << "[COM] Added " << name << " fd=" << fd << std::endl;
      }
      catch(const char* e)
      {
        std::cerr << "[COM] Error adding " << name << ": " << e << std::endl;
      }
    }
    else
    {
      std::cerr << "[COM] Failed to open serial port for " << name << std::endl;
    }
  }
  // 发送底盘目标数据包到STM32,依靠两个函数
  // send2stm32 与 __send2stm32
  // 构造数据包包头
  int Communication::send2stm32(const ChassisTarget& data_packet)
  {
    // 设置数据包头
    PacketHeader header(ENUM_PACKET_ARMOR_DATA, sizeof(PacketHeader) + sizeof(ChassisTarget));
    header.packet_type = static_cast<uint8_t>(ENUM_PACKET_NAV_DATA);
    //
    header.start1 = 0xa5;
    header.start2 = 0x5a;
    header.from = static_cast<uint8_t>(ArmEnum::ENUM_ARM_SENTRY);
    header.setTo(ArmEnum::ENUM_ARM_SLAVE_COMPUTER);
    header.setDataLen(sizeof(data_packet));
    return __send2stm32(header, &data_packet);
  }
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

    while(true)
    {
      // 错误处理
      if(stm32_recv_buffer.size() < 2)
        return;

      char* buf = (char*)stm32_recv_buffer.get();
      // fmt::print("stm32_recv_buffer size: {}\n", stm32_recv_buffer.size());
      // // print buffer in hex
      // fmt::print("Buffer: ");
      // for(size_t i = 0; i < stm32_recv_buffer.size(); ++i)
      // {
      //   fmt::print("{:02X} ", (uint8_t)buf[i]);
      // }
      // fmt::print("\n");
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
        fmt::print(stderr, "[COM] Warning: Start flag(0xA5 0x5A) not found\n");
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
        // std::cout << "[COM] Received packet type: " << std::dec << (int)header->packet_type
        //           << ", length: " << std::dec << (int)header->data_len << std::endl;
        switch(header->packet_type)
        {
        // 校验通过
        case ENUM_PACKET_NAV_DATA:
        {
          const NavRes* nav_data = (const NavRes*)(buf + sizeof(PacketHeader));
          nav_publish(nav_data);
          // fmt::print("[SUCCESS] Reveive Nav Goal\n");
          break;
        }

        case ENUM_PACKET_GAMESTATUS_DATA:
        {
          const EventStatus* event_status = (const EventStatus*)(buf + sizeof(PacketHeader));
          // fmt::print("Event Status: self_health={}, own_outpost_destroyed={}, buff_active={}, "
          //            "enemy_detected.is_get={}, enemy_detected.position=({}, {}, {}), enemy_detected.armor_id={}\n",
          //            event_status->self_health,
          //            event_status->own_outpost_destroyed,
          //            event_status->buff_active,
          //            event_status->is_get,
          //            event_status->x,
          //            event_status->y,
          //            event_status->z,
          //            event_status->armor_id);
          // fmt::print("Team Positions:\n");
          // for(int i = 0; i < 5; ++i)
          // {
          //   fmt::print("  Robot {}: (x={}, y={})\n", i + 1, event_status->team_position[i][0],
          //   event_status->team_position[i][1]);
          // }
          game_status_publish(event_status);
          // fmt::print("[SUCCESS] Reveive GameStatus Data\n");
          break;
        }
        default:
          fmt::print(stderr, "[COM] Warning: Undefined packet type {}\n", header->packet_type);
        }
      }
      else
      {
        //  fmt::print(stderr, "[COM] 警告: 校验和错误 (包类型: {})\n", header->packet_type);
      }
      stm32_recv_buffer = stm32_recv_buffer.sub(full_len);
    }
  }

  // 接收消息回调
  // 导航数据发布
  void Communication::nav_publish(const NavRes* msg)
  {
    static std::shared_ptr<rclcpp::Node> node;
    static std::shared_ptr<rclcpp::Publisher<robot_msgs::msg::Nav>> pub;
    static std::once_flag flag;
    std::call_once(flag,
                   []()
                   {
                     node = rclcpp::Node::make_shared("nav_publisher_node");
                     pub = node->create_publisher<robot_msgs::msg::Nav>("/NavRequest", 10);
                   });

    robot_msgs::msg::Nav nav_data;
    nav_data.target_x = msg->x;
    nav_data.target_y = msg->y;
    nav_data.nav_mode = robot_msgs::msg::Nav::MODE_SINGLE_POINT;
    nav_data.header.stamp = rclcpp::Clock().now();
    pub->publish(nav_data);
  }

  void Communication::referee_publish(const RefereeInfo* msg)
  {
    static std::shared_ptr<rclcpp::Node> node;
    static std::shared_ptr<rclcpp::Publisher<robot_msgs::msg::Referee>> pub;
    static std::once_flag flag;
    std::call_once(flag,
                   []()
                   {
                     node = rclcpp::Node::make_shared("referee_publisher_node");
                     pub = node->create_publisher<robot_msgs::msg::Referee>("/RefereeInfo", 10);
                   });

    robot_msgs::msg::Referee referee_data;
    referee_data.header.stamp = rclcpp::Clock().now();
    // TO DO
    (void)msg;
    pub->publish(referee_data);
  }

  void Communication::game_status_publish(const EventStatus* msg)
  {
    static std::shared_ptr<rclcpp::Node> node;
    static std::shared_ptr<rclcpp::Publisher<robot_msgs::msg::EventStatus>> pub;
    static std::shared_ptr<rclcpp::Publisher<robot_msgs::msg::Referee>> pub_team_position;
    static std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> gimbal_yaw_pub;

    static std::once_flag flag;
    std::call_once(flag,
                   []()
                   {
                     node = rclcpp::Node::make_shared("event_status_publisher_node");
                     pub = node->create_publisher<robot_msgs::msg::EventStatus>("/sentry/event_status", 10);
                     pub_team_position = node->create_publisher<robot_msgs::msg::Referee>("/sentry/team_position", 10);
                     gimbal_yaw_pub = node->create_publisher<std_msgs::msg::Float32>("/sentry/gimbal_yaw", 10);
                   });
    // std::cout << "gimbal:" << msg->gimbal_yaw << "+" << msg->game_status << std::endl;
    robot_msgs::msg::EventStatus event_status;
    event_status.own_outpost_destroyed = msg->own_outpost_destroyed;
    event_status.enemy_outpost_health = 1000;
    // event_status.enemy_outpost_health = msg->enemy_outpost_health;
    event_status.buff_active = msg->buff_active;
    event_status.enemy_detected.is_get = msg->is_get;
    event_status.enemy_detected.position.x = msg->x;
    event_status.enemy_detected.position.y = msg->y;
    event_status.enemy_detected.position.z = msg->z;
    event_status.enemy_detected.armor_id = msg->armor_id;
    event_status.game_status = msg->game_status;
    event_status.self_health = msg->self_health;
    // std::cout << "self_health:" << event_status.self_health << std::endl;
    // if(msg->num_shoot < 50)
    // {
    //   event_status.self_health = 50;
    // }
    // ADD Delay ？？
    event_status.header.stamp = rclcpp::Clock().now();
    pub->publish(event_status);
    // publish team position
    robot_msgs::msg::Referee team_position_msg;
    for(int i = 0; i < 5; ++i)
    {
      robot_msgs::msg::TeamPos team_pos;
      // team_position 是 float[5][2] 数组，[i][0] 是 x坐标，[i][1] 是 y坐标
      team_pos.x = msg->team_position[i][0];
      team_pos.y = msg->team_position[i][1];
      team_position_msg.team.push_back(team_pos);
    }
    team_position_msg.header.stamp = rclcpp::Clock().now();
    pub_team_position->publish(team_position_msg);

    std_msgs::msg::Float32 gimbal_yaw;
    gimbal_yaw.data = msg->gimbal_yaw;
    gimbal_yaw_pub->publish(gimbal_yaw);
  }

}  // namespace ns_com