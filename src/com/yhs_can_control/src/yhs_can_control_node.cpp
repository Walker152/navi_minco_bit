#include "yhs_can_control/yhs_can_control_node.hpp"

namespace yhs
{

  CanControl::CanControl(rclcpp::Node::SharedPtr node)
    : node_(node)
    , if_name_("can0")
    , can_socket_(-1)
  {
    READ_PARAM(std::string, "can_name", (if_name_), "can0");

    node_->declare_parameter<std::vector<int64_t>>("ultrasonic_number", std::vector<int64_t>{});
    node_->get_parameter("ultrasonic_number", ultrasonic_number_);

    io_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::IoCmd>(
        "io_cmd", 1, std::bind(&CanControl::io_cmd_callback, this, std::placeholders::_1));

    motor_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::MotorCmd>(
        "motor_cmd", 1, std::bind(&CanControl::motor_cmd_callback, this, std::placeholders::_1));

    ctrl_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::CtrlCmd>(
        "ctrl_cmd", 1, std::bind(&CanControl::ctrl_cmd_callback, this, std::placeholders::_1));

    steering_ctrl_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::SteeringCtrlCmd>(
        "steering_ctrl_cmd", 1, std::bind(&CanControl::steering_ctrl_cmd_callback, this, std::placeholders::_1));

    chassis_info_fb_publisher_ = node_->create_publisher<yhs_can_interfaces::msg::ChassisInfoFb>("chassis_info_fb", 1);

    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
  }

  void CanControl::io_cmd_callback(const yhs_can_interfaces::msg::IoCmd::SharedPtr io_cmd_msg)
  {
    yhs_can_interfaces::msg::IoCmd msg = *io_cmd_msg;
    static unsigned char count = 0;

    unsigned char sendDataTemp[8] = {0};

    if(msg.io_cmd_lamp_ctrl)
      sendDataTemp[0] |= 0x01;
    if(msg.io_cmd_unlock)
      sendDataTemp[0] |= 0x02;

    if(msg.io_cmd_low_power_enable)
      sendDataTemp[0] |= 0x04;

    if(msg.io_cmd_lower_beam_headlamp)
      sendDataTemp[1] |= 0x01;
    if(msg.io_cmd_upper_beam_headlamp)
      sendDataTemp[1] |= 0x02;

    if(msg.io_cmd_turn_lamp == 0)
      sendDataTemp[1] |= 0x00;
    if(msg.io_cmd_turn_lamp == 1)
      sendDataTemp[1] |= 0x04;
    if(msg.io_cmd_turn_lamp == 2)
      sendDataTemp[1] |= 0x08;
    if(msg.io_cmd_turn_lamp == 3)
      sendDataTemp[1] |= 0x0C;

    if(msg.io_cmd_braking_lamp)
      sendDataTemp[1] |= 0x10;
    if(msg.io_cmd_clearance_lamp)
      sendDataTemp[1] |= 0x20;
    if(msg.io_cmd_fog_lamp)
      sendDataTemp[1] |= 0x40;
    if(msg.io_cmd_speaker)
      sendDataTemp[1] |= 0x80;

    sendDataTemp[2] = msg.io_cmd_low_power_ratio;

    count++;
    if(count == 16)
      count = 0;

    sendDataTemp[6] = count << 4;

    sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^
                      sendDataTemp[5] ^ sendDataTemp[6];

    can_frame send_frame;

    send_frame.can_id = 0x98C4D7D0;
    send_frame.can_dlc = 8;

    memcpy(send_frame.data, sendDataTemp, 8);

    int ret = write(can_socket_, &send_frame, sizeof(send_frame));
    if(ret <= 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to send message: " << strerror(errno));
    }
  }

  void CanControl::motor_cmd_callback(const yhs_can_interfaces::msg::MotorCmd::SharedPtr motor_cmd_msg)
  {
    yhs_can_interfaces::msg::MotorCmd msg = *motor_cmd_msg;
    static unsigned char count = 0;

    unsigned char sendDataTemp[8] = {0};

    if(msg.motor_cmd_drive_enable_lf)
      sendDataTemp[0] |= 0x01;
    if(msg.motor_cmd_drive_enable_lr)
      sendDataTemp[0] |= 0x02;
    if(msg.motor_cmd_drive_enable_rf)
      sendDataTemp[0] |= 0x04;
    if(msg.motor_cmd_drive_enable_rr)
      sendDataTemp[0] |= 0x08;

    if(msg.motor_cmd_steering_enable_lf)
      sendDataTemp[0] |= 0x10;
    if(msg.motor_cmd_steering_enable_lr)
      sendDataTemp[0] |= 0x20;
    if(msg.motor_cmd_steering_enable_rf)
      sendDataTemp[0] |= 0x40;
    if(msg.motor_cmd_steering_enable_rr)
      sendDataTemp[0] |= 0x80;

    if(msg.motor_cmd_power_restart)
      sendDataTemp[1] |= 0x01;

    count++;
    if(count == 16)
      count = 0;

    sendDataTemp[6] = count << 4;

    sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^
                      sendDataTemp[5] ^ sendDataTemp[6];

    can_frame send_frame;

    send_frame.can_id = 0x98C4D8D0;
    send_frame.can_dlc = 8;

    memcpy(send_frame.data, sendDataTemp, 8);

    int ret = write(can_socket_, &send_frame, sizeof(send_frame));
    if(ret <= 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to send message: " << strerror(errno));
    }
  }

  void CanControl::ctrl_cmd_callback(const yhs_can_interfaces::msg::CtrlCmd::SharedPtr ctrl_cmd_msg)
  {
    yhs_can_interfaces::msg::CtrlCmd msg = *ctrl_cmd_msg;
    const short ctrl_cmd_x_linear = msg.ctrl_cmd_x_linear * 1000;
    const short ctrl_cmd_z_angular = msg.ctrl_cmd_z_angular * 1000;
    const short ctrl_cmd_y_linear = msg.ctrl_cmd_y_linear * 1000;
    const unsigned char gear = msg.ctrl_cmd_gear;

    static unsigned char count = 0;
    unsigned char sendDataTemp[8] = {0};

    sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);

    sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((ctrl_cmd_x_linear & 0x0f) << 4));

    sendDataTemp[1] = (ctrl_cmd_x_linear >> 4) & 0xff;

    sendDataTemp[2] = sendDataTemp[2] | (0x0f & (ctrl_cmd_x_linear >> 12));

    sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((ctrl_cmd_z_angular & 0x0f) << 4));

    sendDataTemp[3] = (ctrl_cmd_z_angular >> 4) & 0xff;

    sendDataTemp[4] = sendDataTemp[4] | (0x0f & (ctrl_cmd_z_angular >> 12));

    sendDataTemp[4] = sendDataTemp[4] | (0xf0 & ((ctrl_cmd_y_linear & 0x0f) << 4));

    sendDataTemp[5] = (ctrl_cmd_y_linear >> 4) & 0xff;

    sendDataTemp[6] = sendDataTemp[6] | (0x0f & (ctrl_cmd_y_linear >> 12));

    count++;

    if(count == 16)
      count = 0;

    sendDataTemp[6] = sendDataTemp[6] | (count << 4);

    sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^
                      sendDataTemp[5] ^ sendDataTemp[6];

    can_frame send_frame;

    send_frame.can_id = 0x98C4D1D0;
    send_frame.can_dlc = 8;

    memcpy(send_frame.data, sendDataTemp, 8);

    int ret = write(can_socket_, &send_frame, sizeof(send_frame));

    if(ret <= 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to send message: " << strerror(errno));
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("yhs_can_control_node"), "send ctrl cmd");
  }

  void CanControl::steering_ctrl_cmd_callback(
      const yhs_can_interfaces::msg::SteeringCtrlCmd::SharedPtr steering_ctrl_cmd_msg)
  {
    yhs_can_interfaces::msg::SteeringCtrlCmd msg = *steering_ctrl_cmd_msg;
    const short steering_ctrl_cmd_velocity = msg.steering_ctrl_cmd_velocity * 1000;
    const short steering_ctrl_cmd_steering = msg.steering_ctrl_cmd_steering * 100;
    const unsigned char gear = msg.ctrl_cmd_gear;

    static unsigned char count = 0;
    unsigned char sendDataTemp[8] = {0};
    std::cout << "get can" << std::endl;
    sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);

    sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((steering_ctrl_cmd_velocity & 0x0f) << 4));

    sendDataTemp[1] = (steering_ctrl_cmd_velocity >> 4) & 0xff;

    sendDataTemp[2] = sendDataTemp[2] | (0x0f & (steering_ctrl_cmd_velocity >> 12));

    sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((steering_ctrl_cmd_steering & 0x0f) << 4));

    sendDataTemp[3] = (steering_ctrl_cmd_steering >> 4) & 0xff;

    sendDataTemp[4] = sendDataTemp[4] | (0x0f & (steering_ctrl_cmd_steering >> 12));

    count++;
    if(count == 16)
      count = 0;

    sendDataTemp[6] = count << 4;

    sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^
                      sendDataTemp[5] ^ sendDataTemp[6];

    can_frame send_frame;

    send_frame.can_id = 0x98C4D2D0;
    send_frame.can_dlc = 8;

    memcpy(send_frame.data, sendDataTemp, 8);

    int ret = write(can_socket_, &send_frame, sizeof(send_frame));
    if(ret <= 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to send message: " << strerror(errno));
    }
  }

  bool CanControl::wait_for_can_frame()
  {
    struct timeval tv;
    fd_set rdfs;
    FD_ZERO(&rdfs);
    FD_SET(can_socket_, &rdfs);
    tv.tv_sec = 0;
    tv.tv_usec = 500000;  // 10ms

    int ret = select(can_socket_ + 1, &rdfs, NULL, NULL, &tv);
    if(ret == -1)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"),
                          "Error waiting for CAN frame: " << std::strerror(errno));
      return false;
    }
    else if(ret == 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"),
                          "Timeout waiting for CAN frame! Please check whether the can0 setting is correct,\
whether the can line is connected correctly, and whether the chassis is powered on.");
      return false;
    }
    else
    {
      return true;
    }
    return false;
  }

  void CanControl::can_data_recv_callback()
  {
    can_frame recv_frame;
    yhs_can_interfaces::msg::ChassisInfoFb chassis_info_msg;

    while(rclcpp::ok())
    {
      if(!wait_for_can_frame())
        continue;

      if(read(can_socket_, &recv_frame, sizeof(recv_frame)) >= 0)
      {
        switch(recv_frame.can_id)
        {
        case 0x98C4D1EF:
        {
          yhs_can_interfaces::msg::CtrlFb msg;
          msg.ctrl_fb_gear = 0x0f & recv_frame.data[0];

          msg.ctrl_fb_x_linear =
              static_cast<float>(static_cast<short>((recv_frame.data[2] & 0x0f) << 12 | recv_frame.data[1] << 4 |
                                                    (recv_frame.data[0] & 0xf0) >> 4)) /
              1000;

          msg.ctrl_fb_z_angular =
              static_cast<float>(static_cast<short>((recv_frame.data[4] & 0x0f) << 12 | recv_frame.data[3] << 4 |
                                                    (recv_frame.data[2] & 0xf0) >> 4)) /
              100;

          msg.ctrl_fb_y_linear =
              static_cast<float>(static_cast<short>((recv_frame.data[6] & 0x0f) << 12 | recv_frame.data[5] << 4 |
                                                    (recv_frame.data[4] & 0xf0) >> 4)) /
              100;

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^
                              recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if(crc == recv_frame.data[7])
          {
            chassis_info_msg.header.stamp = node_->get_clock()->now();
            chassis_info_msg.ctrl_fb = msg;
            chassis_info_fb_publisher_->publish(chassis_info_msg);
            publish_odom(msg.ctrl_fb_x_linear, msg.ctrl_fb_z_angular / 180 * 3.14);
          }

          break;
        }

        case 0x98C4D2EF:
        {
          yhs_can_interfaces::msg::SteeringCtrlFb msg;
          msg.steering_ctrl_fb_gear = 0x0f & recv_frame.data[0];

          msg.steering_ctrl_fb_velocity =
              static_cast<float>(static_cast<short>((recv_frame.data[2] & 0x0f) << 12 | recv_frame.data[1] << 4 |
                                                    (recv_frame.data[0] & 0xf0) >> 4)) /
              1000;

          msg.steering_ctrl_fb_steering =
              static_cast<float>(static_cast<short>((recv_frame.data[4] & 0x0f) << 12 | recv_frame.data[3] << 4 |
                                                    (recv_frame.data[2] & 0xf0) >> 4)) /
              100;

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^
                              recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if(crc == recv_frame.data[7])
          {
            chassis_info_msg.steering_ctrl_fb = msg;
          }

          break;
        }

        case 0x98C4D6EF:
        {
          yhs_can_interfaces::msg::LfWheelFb msg;
          msg.lf_wheel_fb_velocity =
              static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000;

          msg.lf_wheel_fb_pulse = static_cast<int>(recv_frame.data[5] << 24 | recv_frame.data[4] << 16 |
                                                   recv_frame.data[3] << 8 | recv_frame.data[2]);

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^
                              recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if(crc == recv_frame.data[7])
          {
            chassis_info_msg.lf_wheel_fb = msg;
          }

          break;
        }

        case 0x98C4D7EF:
        {
          yhs_can_interfaces::msg::LrWheelFb msg;
          msg.lr_wheel_fb_velocity =
              static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000;

          msg.lr_wheel_fb_pulse = static_cast<int>(recv_frame.data[5] << 24 | recv_frame.data[4] << 16 |
                                                   recv_frame.data[3] << 8 | recv_frame.data[2]);

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^
                              recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if(crc == recv_frame.data[7])
          {
            chassis_info_msg.lr_wheel_fb = msg;
          }

          break;
        }

        case 0x98C4D8EF:
        {
          yhs_can_interfaces::msg::RrWheelFb msg;
          msg.rr_wheel_fb_velocity =
              static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000;

          msg.rr_wheel_fb_pulse = static_cast<int>(recv_frame.data[5] << 24 | recv_frame.data[4] << 16 |
                                                   recv_frame.data[3] << 8 | recv_frame.data[2]);

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^
                              recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if(crc == recv_frame.data[7])
          {
            chassis_info_msg.rr_wheel_fb = msg;
          }

          break;
        }

        case 0x98C4D9EF:
        {
          yhs_can_interfaces::msg::RfWheelFb msg;
          msg.rf_wheel_fb_velocity =
              static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000;

          msg.rf_wheel_fb_pulse = static_cast<int>(recv_frame.data[5] << 24 | recv_frame.data[4] << 16 |
                                                   recv_frame.data[3] << 8 | recv_frame.data[2]);

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^
                              recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if(crc == recv_frame.data[7])
          {
            chassis_info_msg.rf_wheel_fb = msg;
          }

          break;
        }

        case 0x98C4DAEF:
        {
          yhs_can_interfaces::msg::IoFb msg;
          msg.io_fb_lamp_ctrl = (recv_frame.data[0] & 0x01) != 0;
          msg.io_fb_unlock = (recv_frame.data[0] & 0x02) != 0;
          msg.io_fb_lower_beam_headlamp = (recv_frame.data[1] & 0x01) != 0;
          msg.io_fb_upper_beam_headlamp = (recv_frame.data[1] & 0x02) != 0;

          msg.io_fb_low_power_enable = (recv_frame.data[1] & 0x04) != 0;
          msg.io_fb_low_power_state = (recv_frame.data[1] & 0x08) != 0;

          msg.io_fb_turn_lamp = (0x0c & recv_frame.data[1]) >> 2;

          msg.io_fb_braking_lamp = (0x10 & recv_frame.data[1]) != 0;
          msg.io_fb_clearance_lamp = (0x20 & recv_frame.data[1]) != 0;
          msg.io_fb_fog_lamp = (0x40 & recv_frame.data[1]) != 0;
          msg.io_fb_speaker = (0x80 & recv_frame.data[1]) != 0;

          msg.io_fb_low_power_ratio = recv_frame.data[2];

          msg.io_fb_fl_impact_sensor = (0x01 & recv_frame.data[3]) != 0;
          msg.io_fb_fm_impact_sensor = (0x02 & recv_frame.data[3]) != 0;
          msg.io_fb_fr_impact_sensor = (0x04 & recv_frame.data[3]) != 0;
          msg.io_fb_rl_impact_sensor = (0x08 & recv_frame.data[3]) != 0;
          msg.io_fb_rm_impact_sensor = (0x10 & recv_frame.data[3]) != 0;
          msg.io_fb_rr_impact_sensor = (0x20 & recv_frame.data[3]) != 0;
          msg.io_fb_fl_drop_sensor = (0x01 & recv_frame.data[4]) != 0;
          msg.io_fb_fm_drop_sensor = (0x02 & recv_frame.data[4]) != 0;
          msg.io_fb_fr_drop_sensor = (0x04 & recv_frame.data[4]) != 0;
          msg.io_fb_rl_drop_sensor = (0x08 & recv_frame.data[4]) != 0;
          msg.io_fb_rm_drop_sensor = (0x10 & recv_frame.data[4]) != 0;
          msg.io_fb_rr_drop_sensor = (0x20 & recv_frame.data[4]) != 0;
          msg.io_fb_estop = (0x01 & recv_frame.data[5]) != 0;
          msg.io_fb_joypad_ctrl = (0x02 & recv_frame.data[5]) != 0;
          msg.io_fb_charge_state = (0x04 & recv_frame.data[5]) != 0;

          msg.io_fb_charger_sign = (0x08 & recv_frame.data[5]) != 0;
          msg.io_fb_joypad_first = (0x10 & recv_frame.data[5]) != 0;
          msg.io_fb_joypad_online = (0x20 & recv_frame.data[5]) != 0;

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^
                              recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if(crc == recv_frame.data[7])
          {
            chassis_info_msg.io_fb = msg;
          }

          break;
        }

        case 0x98C4DBEF:
        {
          yhs_can_interfaces::msg::MotorFb msg;
          msg.motor_cmd_drive_enable_lf = (recv_frame.data[0] & 0x01) != 0;
          msg.motor_cmd_drive_enable_lr = (recv_frame.data[0] & 0x02) != 0;

          msg.motor_cmd_drive_enable_rf = (recv_frame.data[0] & 0x04) != 0;
          msg.motor_cmd_drive_enable_rr = (recv_frame.data[0] & 0x08) != 0;

          msg.motor_cmd_steering_enable_lf = (0x10 & recv_frame.data[0]) != 0;
          msg.motor_cmd_steering_enable_lr = (0x20 & recv_frame.data[0]) != 0;
          msg.motor_cmd_steering_enable_rf = (0x40 & recv_frame.data[0]) != 0;
          msg.motor_cmd_steering_enable_rr = (0x80 & recv_frame.data[0]) != 0;

          msg.motor_cmd_power_restart = (recv_frame.data[1] & 0x01) != 0;

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^
                              recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if(crc == recv_frame.data[7])
          {
            chassis_info_msg.motor_fb = msg;
            chassis_info_fb_publisher_->publish(chassis_info_msg);
          }

          break;
        }

        case 0x98C4DCEF:
        {
          yhs_can_interfaces::msg::FrontAngleFb msg;
          msg.front_angle_fb_l =
              static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 100;

          msg.front_angle_fb_r =
              static_cast<float>(static_cast<short>(recv_frame.data[3] << 8 | recv_frame.data[2])) / 100;

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^
                              recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if(crc == recv_frame.data[7])
          {
            chassis_info_msg.front_angle_fb = msg;
          }

          break;
        }

        case 0x98C4DDEF:
        {
          yhs_can_interfaces::msg::RearAngleFb msg;
          msg.rear_angle_fb_l =
              static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 100;

          msg.rear_angle_fb_r =
              static_cast<float>(static_cast<short>(recv_frame.data[3] << 8 | recv_frame.data[2])) / 100;

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^
                              recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if(crc == recv_frame.data[7])
          {
            chassis_info_msg.rear_angle_fb = msg;
          }

          break;
        }

        case 0x98C4E1EF:
        {
          yhs_can_interfaces::msg::BmsFb msg;
          msg.bms_fb_voltage =
              static_cast<float>(static_cast<unsigned short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 100;

          msg.bms_fb_current =
              static_cast<float>(static_cast<short>(recv_frame.data[3] << 8 | recv_frame.data[2])) / 100;

          msg.bms_fb_remaining_capacity =
              static_cast<float>(static_cast<unsigned short>(recv_frame.data[5] << 8 | recv_frame.data[4])) / 100;

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^
                              recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if(crc == recv_frame.data[7])
          {
            chassis_info_msg.bms_fb = msg;
          }

          break;
        }

        case 0x98C4E2EF:
        {
          yhs_can_interfaces::msg::BmsFlagFb msg;
          msg.bms_flag_fb_soc = recv_frame.data[0];

          msg.bms_flag_fb_single_ov = (recv_frame.data[1] & 0x01) != 0;
          msg.bms_flag_fb_single_uv = (recv_frame.data[1] & 0x02) != 0;
          msg.bms_flag_fb_ov = (recv_frame.data[1] & 0x04) != 0;
          msg.bms_flag_fb_uv = (recv_frame.data[1] & 0x08) != 0;
          msg.bms_flag_fb_charge_ot = (recv_frame.data[1] & 0x10) != 0;
          msg.bms_flag_fb_charge_ut = (recv_frame.data[1] & 0x20) != 0;
          msg.bms_flag_fb_discharge_ot = (recv_frame.data[1] & 0x40) != 0;
          msg.bms_flag_fb_discharge_ut = (recv_frame.data[1] & 0x80) != 0;

          msg.bms_flag_fb_charge_oc = (recv_frame.data[2] & 0x01) != 0;
          msg.bms_flag_fb_discharge_oc = (recv_frame.data[2] & 0x02) != 0;
          msg.bms_flag_fb_short = (recv_frame.data[2] & 0x04) != 0;
          msg.bms_flag_fb_ic_error = (recv_frame.data[2] & 0x08) != 0;
          msg.bms_flag_fb_lock_mos = (recv_frame.data[2] & 0x10) != 0;
          msg.bms_flag_fb_charge_flag = (recv_frame.data[2] & 0x20) != 0;
          msg.bms_flag_fb_heating_flag = (recv_frame.data[2] & 0x40) != 0;

          msg.bms_flag_fb_hight_temperature =
              static_cast<float>(static_cast<short>(recv_frame.data[4] << 4 | recv_frame.data[3] >> 4)) / 10;

          msg.bms_flag_fb_low_temperature =
              static_cast<float>(static_cast<short>((recv_frame.data[6] & 0x0f) << 8 | recv_frame.data[5])) / 10;

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^
                              recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if(crc == recv_frame.data[7])
          {
            chassis_info_msg.bms_flag_fb = msg;
          }

          break;
        }

        case 0x98C4E3EF:
        {
          yhs_can_interfaces::msg::DriveMotorCurrentFb msg;
          msg.drive_motor_current_fb_lf =
              (float)((short)(((recv_frame.data[1] & 0x0f) << 8 | recv_frame.data[0]) << 4) >> 4) / 10;
          msg.drive_motor_current_fb_lr =
              (float)((short)((recv_frame.data[2] << 4 | (recv_frame.data[1] >> 4)) << 4) >> 4) / 10;

          msg.drive_motor_current_fb_rf =
              (float)((short)(((recv_frame.data[4] & 0x0f) << 8 | recv_frame.data[3]) << 4) >> 4) / 10;
          msg.drive_motor_current_fb_rr =
              (float)((short)((recv_frame.data[5] << 4 | (recv_frame.data[4] >> 4)) << 4) >> 4) / 10;

          if(0x01 & recv_frame.data[6])
            msg.drive_motor_oc_flag_fb_lf = true;
          else
            msg.drive_motor_oc_flag_fb_lf = false;

          if(0x02 & recv_frame.data[6])
            msg.drive_motor_oc_flag_fb_lr = true;
          else
            msg.drive_motor_oc_flag_fb_lr = false;

          if(0x04 & recv_frame.data[6])
            msg.drive_motor_oc_flag_fb_rf = true;
          else
            msg.drive_motor_oc_flag_fb_rf = false;

          if(0x08 & recv_frame.data[6])
            msg.drive_motor_oc_flag_fb_rr = true;
          else
            msg.drive_motor_oc_flag_fb_rr = false;

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^
                              recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if(crc == recv_frame.data[7])
          {
            chassis_info_msg.drive_motor_current_fb = msg;
          }

          break;
        }

        case 0x98C4E4EF:
        {
          yhs_can_interfaces::msg::SteeringMotorCurrentFb msg;
          msg.steering_motor_current_fb_lf =
              (float)((short)(((recv_frame.data[1] & 0x0f) << 8 | recv_frame.data[0]) << 4) >> 4) / 10;
          msg.steering_motor_current_fb_lr =
              (float)((short)((recv_frame.data[2] << 4 | (recv_frame.data[1] >> 4)) << 4) >> 4) / 10;

          msg.steering_motor_current_fb_rf =
              (float)((short)(((recv_frame.data[4] & 0x0f) << 8 | recv_frame.data[3]) << 4) >> 4) / 10;
          msg.steering_motor_current_fb_rr =
              (float)((short)((recv_frame.data[5] << 4 | (recv_frame.data[4] >> 4)) << 4) >> 4) / 10;

          if(0x01 & recv_frame.data[6])
            msg.steering_motor_oc_flag_fb_lf = true;
          else
            msg.steering_motor_oc_flag_fb_lf = false;

          if(0x02 & recv_frame.data[6])
            msg.steering_motor_oc_flag_fb_lr = true;
          else
            msg.steering_motor_oc_flag_fb_lr = false;

          if(0x04 & recv_frame.data[6])
            msg.steering_motor_oc_flag_fb_rf = true;
          else
            msg.steering_motor_oc_flag_fb_rf = false;

          if(0x08 & recv_frame.data[6])
            msg.steering_motor_oc_flag_fb_rr = true;
          else
            msg.steering_motor_oc_flag_fb_rr = false;
          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^
                              recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if(crc == recv_frame.data[7])
          {
            chassis_info_msg.steering_motor_current_fb = msg;
          }

          break;
        }

          // ultrasonic
          static unsigned short ultra_data[8] = {0};
        case 0x98C4E8EF:
        {
          ultra_data[0] = (unsigned short)((recv_frame.data[1] & 0x0f) << 8 | recv_frame.data[0]);
          ultra_data[1] = (unsigned short)(recv_frame.data[2] << 4 | ((recv_frame.data[1] & 0xf0) >> 4));

          ultra_data[2] = (unsigned short)((recv_frame.data[4] & 0x0f) << 8 | recv_frame.data[3]);
          ultra_data[3] = (unsigned short)(recv_frame.data[5] << 4 | ((recv_frame.data[4] & 0xf0) >> 4));
          break;
        }

        case 0x98C4E9EF:
        {
          ultra_data[4] = (unsigned short)((recv_frame.data[1] & 0x0f) << 8 | recv_frame.data[0]);
          ultra_data[5] = (unsigned short)(recv_frame.data[2] << 4 | ((recv_frame.data[1] & 0xf0) >> 4));

          ultra_data[6] = (unsigned short)((recv_frame.data[4] & 0x0f) << 8 | recv_frame.data[3]);
          ultra_data[7] = (unsigned short)(recv_frame.data[5] << 4 | ((recv_frame.data[4] & 0xf0) >> 4));

          yhs_can_interfaces::msg::Ultrasonic ultra_msg;

          ultra_msg.front_right = ultra_data[ultrasonic_number_[0]];
          ultra_msg.front_left = ultra_data[ultrasonic_number_[1]];
          ultra_msg.left_front = ultra_data[ultrasonic_number_[2]];
          ultra_msg.left_rear = ultra_data[ultrasonic_number_[3]];

          ultra_msg.rear_left = ultra_data[ultrasonic_number_[4]];
          ultra_msg.rear_right = ultra_data[ultrasonic_number_[5]];
          ultra_msg.right_rear = ultra_data[ultrasonic_number_[6]];
          ultra_msg.right_front = ultra_data[ultrasonic_number_[7]];

          chassis_info_msg.ultrasonic = ultra_msg;

          break;
        }

        case 0x98C4EAEF:
        {
          yhs_can_interfaces::msg::ErrorFb msg;
          msg.error_fb_level = recv_frame.data[0];

          msg.error_fb_device_type = recv_frame.data[1];

          msg.error_fb_devive_id = recv_frame.data[2];

          msg.error_fb_emergency_code = recv_frame.data[3];

          msg.error_fb_register_code = (recv_frame.data[5] << 8) | recv_frame.data[4];

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^
                              recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if(crc == recv_frame.data[7])
          {
            chassis_info_msg.error_fb = msg;
          }

          break;
        }

        default:
          break;
        }
      }
    }
  }

  void CanControl::publish_odom(const double linear_vel, const double angular_vel)
  {
    static double x_ = 0.0;
    static double y_ = 0.0;
    static double theta_ = 0.0;
    static rclcpp::Time last_time_ = node_->now();

    rclcpp::Time current_time = node_->now();

    double dt = (current_time - last_time_).seconds();

    x_ += linear_vel * cos(theta_) * dt;
    y_ += linear_vel * sin(theta_) * dt;
    theta_ += angular_vel * dt;

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    geometry_msgs::msg::PoseWithCovariance pose_cov;
    pose_cov.pose.position.x = x_;
    pose_cov.pose.position.y = y_;
    pose_cov.pose.position.z = 0.0;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, theta_);
    pose_cov.pose.orientation.x = quat.x();
    pose_cov.pose.orientation.y = quat.y();
    pose_cov.pose.orientation.z = quat.z();
    pose_cov.pose.orientation.w = quat.w();
    odom_msg.pose = pose_cov;

    geometry_msgs::msg::TwistWithCovariance twist_cov;
    twist_cov.twist.linear.x = linear_vel;
    twist_cov.twist.linear.y = 0.0;
    twist_cov.twist.linear.z = 0.0;
    twist_cov.twist.angular.x = 0.0;
    twist_cov.twist.angular.y = 0.0;
    twist_cov.twist.angular.z = angular_vel;
    odom_msg.twist = twist_cov;

    odom_pub_->publish(odom_msg);

    last_time_ = current_time;
  }

  CanControl::~CanControl() {}

  bool CanControl::run()
  {
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(can_socket_ < 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to open socket: " << strerror(errno));
      return false;
    }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, if_name_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if(ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"),
                          "Failed to get interface index: " << strerror(errno) << " ==> " << if_name_.c_str());
      return false;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if(bind(can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to bind socket: " << strerror(errno));
      return false;
    }

    thread_ = std::thread(&CanControl::can_data_recv_callback, this);

    return true;
  }

  void CanControl::stop()
  {
    if(can_socket_ >= 0)
    {
      close(can_socket_);
      can_socket_ = -1;
    }

    if(thread_.joinable())
    {
      thread_.join();
    }
  }
}  // namespace yhs

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("yhs_can_control_node");

  yhs::CanControl cancontrol(node);
  if(!cancontrol.run())
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize yhs_can_control_node");
    return 0;
  }

  RCLCPP_INFO(node->get_logger(), "yhs_can_control_node initialized successfully");

  rclcpp::spin(node);

  cancontrol.stop();
  RCLCPP_INFO(node->get_logger(), "yhs_can_control_node stopped");

  rclcpp::shutdown();

  return 0;
}
