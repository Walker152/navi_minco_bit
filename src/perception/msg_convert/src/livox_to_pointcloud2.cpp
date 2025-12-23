#include <cmath>    // for cos, sin
#include <cstring>  // for memcpy
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// 定义点云数据结构（16字节对齐，提高缓存性能）
struct PointXYZI
{
  float x;
  float y;
  float z;
  float intensity;
} __attribute__((packed));

// 3x3旋转矩阵
struct RotationMatrix
{
  float data[3][3];

  RotationMatrix()
  {
    // 初始化为单位矩阵
    for(int i = 0; i < 3; ++i)
      for(int j = 0; j < 3; ++j)
        data[i][j] = (i == j) ? 1.0f : 0.0f;
  }

  // 从欧拉角构造旋转矩阵 (Roll-Pitch-Yaw, XYZ顺序)
  static RotationMatrix fromRPY(float roll, float pitch, float yaw)
  {
    RotationMatrix R;

    float cr = std::cos(roll), sr = std::sin(roll);
    float cp = std::cos(pitch), sp = std::sin(pitch);
    float cy = std::cos(yaw), sy = std::sin(yaw);

    // ZYX欧拉角旋转矩阵
    R.data[0][0] = cy * cp;
    R.data[0][1] = cy * sp * sr - sy * cr;
    R.data[0][2] = cy * sp * cr + sy * sr;

    R.data[1][0] = sy * cp;
    R.data[1][1] = sy * sp * sr + cy * cr;
    R.data[1][2] = sy * sp * cr - cy * sr;

    R.data[2][0] = -sp;
    R.data[2][1] = cp * sr;
    R.data[2][2] = cp * cr;

    return R;
  }

  // 旋转点
  void rotate(float& x, float& y, float& z) const
  {
    float tx = data[0][0] * x + data[0][1] * y + data[0][2] * z;
    float ty = data[1][0] * x + data[1][1] * y + data[1][2] * z;
    float tz = data[2][0] * x + data[2][1] * y + data[2][2] * z;
    x = tx;
    y = ty;
    z = tz;
  }
};

class LivoxToPointCloud2 : public rclcpp::Node
{
public:
  LivoxToPointCloud2()
    : Node("livox_to_pointcloud2")
  {
    // 声明参数
    this->declare_parameter<std::string>("input_topic", "/livox/lidar");
    this->declare_parameter<std::string>("output_topic", "/livox/stdpc");
    this->declare_parameter<std::string>("frame_id", "livox_frame");
    this->declare_parameter<int>("queue_size", 10);

    // 平移参数 (米)
    this->declare_parameter<double>("translation_x", 0.0);
    this->declare_parameter<double>("translation_y", 0.0);
    this->declare_parameter<double>("translation_z", 0.0);

    // 旋转参数 (弧度)
    this->declare_parameter<double>("rotation_roll", 0.0);
    this->declare_parameter<double>("rotation_pitch", 0.0);
    this->declare_parameter<double>("rotation_yaw", 0.0);

    // 裁剪参数 (米)
    this->declare_parameter<bool>("enable_crop", false);
    this->declare_parameter<double>("crop_x_min", -100.0);
    this->declare_parameter<double>("crop_x_max", 100.0);
    this->declare_parameter<double>("crop_y_min", -100.0);
    this->declare_parameter<double>("crop_y_max", 100.0);
    this->declare_parameter<double>("crop_z_min", -100.0);
    this->declare_parameter<double>("crop_z_max", 100.0);

    // 获取参数
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    std::string frame_id = this->get_parameter("frame_id").as_string();
    int queue_size = this->get_parameter("queue_size").as_int();

    translation_x_ = this->get_parameter("translation_x").as_double();
    translation_y_ = this->get_parameter("translation_y").as_double();
    translation_z_ = this->get_parameter("translation_z").as_double();

    double roll = this->get_parameter("rotation_roll").as_double();
    double pitch = this->get_parameter("rotation_pitch").as_double();
    double yaw = this->get_parameter("rotation_yaw").as_double();

    enable_crop_ = this->get_parameter("enable_crop").as_bool();
    crop_x_min_ = this->get_parameter("crop_x_min").as_double();
    crop_x_max_ = this->get_parameter("crop_x_max").as_double();
    crop_y_min_ = this->get_parameter("crop_y_min").as_double();
    crop_y_max_ = this->get_parameter("crop_y_max").as_double();
    crop_z_min_ = this->get_parameter("crop_z_min").as_double();
    crop_z_max_ = this->get_parameter("crop_z_max").as_double();

    // 构建旋转矩阵
    rotation_matrix_ = RotationMatrix::fromRPY(roll, pitch, yaw);

    // 预分配PointCloud2消息（避免重复构造）
    cloud_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    setupPointCloud2Fields();

    // 创建订阅者和发布者
    sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        input_topic,
        queue_size,
        [this](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) { livoxCallback(msg); });

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, queue_size);
    // 发布点云ID
    frame_id_ = frame_id;
    // 打印
    RCLCPP_INFO(this->get_logger(), "  Input topic: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic.c_str());
    RCLCPP_INFO(
        this->get_logger(), "  Translation: [%.3f, %.3f, %.3f] m", translation_x_, translation_y_, translation_z_);
    RCLCPP_INFO(this->get_logger(), "  Rotation (RPY): [%.3f, %.3f, %.3f] rad", roll, pitch, yaw);
    RCLCPP_INFO(this->get_logger(), "  Crop enabled: %s", enable_crop_ ? "YES" : "NO");
    if(enable_crop_)
    {
      RCLCPP_INFO(this->get_logger(), "    X: [%.2f, %.2f] m", crop_x_min_, crop_x_max_);
      RCLCPP_INFO(this->get_logger(), "    Y: [%.2f, %.2f] m", crop_y_min_, crop_y_max_);
      RCLCPP_INFO(this->get_logger(), "    Z: [%.2f, %.2f] m", crop_z_min_, crop_z_max_);
    }
  }

private:
  void setupPointCloud2Fields()
  {
    // 预设置点云字段信息（只需要设置一次）
    cloud_msg_->fields.resize(4);

    cloud_msg_->fields[0].name = "x";
    cloud_msg_->fields[0].offset = 0;
    cloud_msg_->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg_->fields[0].count = 1;

    cloud_msg_->fields[1].name = "y";
    cloud_msg_->fields[1].offset = 4;
    cloud_msg_->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg_->fields[1].count = 1;

    cloud_msg_->fields[2].name = "z";
    cloud_msg_->fields[2].offset = 8;
    cloud_msg_->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg_->fields[2].count = 1;

    cloud_msg_->fields[3].name = "intensity";
    cloud_msg_->fields[3].offset = 12;
    cloud_msg_->fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg_->fields[3].count = 1;

    cloud_msg_->point_step = sizeof(PointXYZI);  // 16 bytes
    cloud_msg_->height = 1;
    cloud_msg_->is_bigendian = false;
    cloud_msg_->is_dense = true;
    cloud_msg_->header.frame_id = "camera_init";
    // cloud_msg_->header.frame_id = frame_id_;
  }

  void livoxCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    const size_t point_num = msg->points.size();
    auto receive_time = this->now();
    // 快速返回空点云
    if(point_num == 0) [[unlikely]]
    {
      return;
    }

    // 预分配数据缓冲区（使用临时buffer来处理裁剪）
    std::vector<PointXYZI> temp_buffer;
    temp_buffer.reserve(point_num);

    // 批量转换、变换和裁剪数据
    for(size_t i = 0; i < point_num; ++i)
    {
      const auto& src = msg->points[i];

      float x = src.x;
      float y = src.y;
      float z = src.z;

      // 应用旋转和平移变换
      rotation_matrix_.rotate(x, y, z);
      x += translation_x_;
      y += translation_y_;
      z += translation_z_;

      // 裁剪检查
      if(enable_crop_)
      {
        if(x < crop_x_min_ || x > crop_x_max_ || y < crop_y_min_ || y > crop_y_max_ || z < crop_z_min_ ||
           z > crop_z_max_)
        {
          continue;  // 跳过范围外的点
        }
      }

      // 添加到临时buffer
      PointXYZI point;
      point.x = x;
      point.y = y;
      point.z = z;
      point.intensity = static_cast<float>(src.reflectivity);
      temp_buffer.push_back(point);
    }

    const size_t output_point_num = temp_buffer.size();

    // 设置消息头和大小
    cloud_msg_->width = output_point_num;
    cloud_msg_->row_step = cloud_msg_->point_step * output_point_num;
    cloud_msg_->data.resize(output_point_num * sizeof(PointXYZI));

    // 拷贝到输出消息
    if(output_point_num > 0)
    {
      std::memcpy(cloud_msg_->data.data(), temp_buffer.data(), output_point_num * sizeof(PointXYZI));
    }

    // 发布转换后的点云
    pub_->publish(*cloud_msg_);
    auto publish_time = this->now();
    // std::cout << "Transform time: " << (publish_time.seconds() - receive_time.seconds()) * 1000 << " ms" << std::endl
    //           << "Input points: " << point_num << ", Output points: " << output_point_num << std::endl;
  }

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_;

  // 变换参数
  RotationMatrix rotation_matrix_;
  double translation_x_, translation_y_, translation_z_;

  // 裁剪参数
  bool enable_crop_;
  double crop_x_min_, crop_x_max_;
  double crop_y_min_, crop_y_max_;
  double crop_z_min_, crop_z_max_;

  // TF 点云后的 frame_id
  std::string frame_id_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LivoxToPointCloud2>();

  std::cout << "Convert Start!\n";

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
