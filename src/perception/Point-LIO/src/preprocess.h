/**
 * @file preprocess.h
 * @brief Point-LIO雷达数据预处理模块头文件
 * @details 该模块负责处理不同类型雷达（Livox、Velodyne、Ouster、Hesai）的点云数据，
 *          包括点云过滤、特征提取、时间戳处理等功能
 * @author Point-LIO开发团队
 * @date 2023
 */

#include <pcl_conversions/pcl_conversions.h>  // PCL与ROS消息转换

#include <deque>                                        // 双端队列容器
#include <livox_ros_driver2/msg/custom_msg.hpp>       // Livox雷达自定义消息类型
#include <rclcpp/rclcpp.hpp>                           // ROS2 C++客户端库
#include <sensor_msgs/msg/point_cloud2.hpp>           // ROS2标准点云消息类型

using namespace std;

// 判断数值是否有效的宏定义（用于检测异常大的数值）
#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

// 点云数据类型定义
typedef pcl::PointXYZINormal PointType;        // 包含坐标、强度和法向量的点类型
typedef pcl::PointCloud<PointType> PointCloudXYZI;  // 点云容器类型

/**
 * @brief 雷达类型枚举
 * @details 支持的雷达类型：
 *          AVIA - Livox Avia雷达 (1)
 *          VELO16 - Velodyne VLP-16雷达 (2) 
 *          OUST64 - Ouster OS1-64雷达 (3)
 *          HESAIxt32 - 禾赛XT32雷达 (4)
 */
enum LID_TYPE { AVIA = 1, VELO16, OUST64, HESAIxt32 };

/**
 * @brief 时间单位枚举
 * @details 用于处理不同雷达的时间戳格式：
 *          SEC - 秒 (0)
 *          MS - 毫秒 (1) 
 *          US - 微秒 (2)
 *          NS - 纳秒 (3)
 */
enum TIME_UNIT { SEC = 0, MS = 1, US = 2, NS = 3 };

/**
 * @brief 点特征类型枚举
 * @details 点云特征分类：
 *          Nor - 普通点
 *          Poss_Plane - 可能的平面点
 *          Real_Plane - 确定的平面点
 *          Edge_Jump - 边缘跳跃点
 *          Edge_Plane - 边缘平面点
 *          Wire - 线状特征点
 *          ZeroPoint - 零点
 */
enum Feature { Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint };

/**
 * @brief 邻域方向枚举
 * @details 用于特征提取时的邻域搜索：
 *          Prev - 前一个点
 *          Next - 后一个点
 */
enum Surround { Prev, Next };

/**
 * @brief 边缘跳跃类型枚举
 * @details 描述点之间的跳跃特性：
 *          Nr_nor - 正常
 *          Nr_zero - 零跳跃
 *          Nr_180 - 180度跳跃
 *          Nr_inf - 无限跳跃
 *          Nr_blind - 盲区跳跃
 */
enum E_jump { Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind };

/**
 * @brief 点云时间排序比较函数
 * @param x 第一个点
 * @param y 第二个点
 * @return 按时间戳（curvature字段）排序的结果
 * @details 用于对点云按时间顺序进行排序，时间信息存储在curvature字段中
 */
const bool time_list_cut_frame(PointType & x, PointType & y);

/**
 * @brief 点组织结构体
 * @details 存储每个点的特征信息，用于特征提取和分类
 */
struct orgtype
{
  double range;      ///< 点到雷达中心的距离
  double dista;      ///< 点之间的距离
  double angle[2];   ///< 与相邻点的夹角[Prev, Next]
  double intersect;  ///< 相邻向量的交角
  E_jump edj[2];     ///< 边缘跳跃类型[Prev, Next]
  Feature ftype;     ///< 点的特征类型
  
  /**
   * @brief 默认构造函数
   * @details 初始化所有字段为默认值
   */
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

/**
 * @brief Velodyne雷达点云数据结构
 * @details 定义Velodyne雷达特有的点云数据格式
 */
namespace velodyne_ros
{
/**
 * @brief Velodyne点数据结构
 * @details 包含3D坐标、强度、时间戳和扫描线编号
 */
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;      ///< PCL标凅4D点坐标(x,y,z,padding)
  float intensity;      ///< 点云强度值
  float time;          ///< 点的时间戳
  uint16_t ring;       ///< 扫描线编号（用于多线束雷达）
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  ///< Eigen内存对齐宏
};
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(
  velodyne_ros::Point, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                         float, time, time)(std::uint16_t, ring, ring))

/**
 * @brief Hesai雷达点云数据结构
 * @details 定义Hesai雷达特有的点云数据格式
 */
namespace hesai_ros
{
/**
 * @brief Hesai点数据结构
 * @details 包含3D坐标、强度、双精度时间戳和扫描线编号
 */
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;      ///< PCL标凅4D点坐标(x,y,z,padding)
  float intensity;      ///< 点云强度值
  double timestamp;     ///< 点的双精度时间戳
  uint16_t ring;        ///< 扫描线编号（用于多线束雷达）
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  ///< Eigen内存对齐宏
};
}  // namespace hesai_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(
  hesai_ros::Point, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                      double, timestamp, timestamp)(std::uint16_t, ring, ring))

/**
 * @brief Ouster雷达点云数据结构
 * @details 定义Ouster雷达特有的点云数据格式，包含更丰富的传感器信息
 */
namespace ouster_ros
{
/**
 * @brief Ouster点数据结构
 * @details 包含3D坐标、强度、时间戳、反射率、扫描线、环境光和距离信息
 */
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;         ///< PCL标凅4D点坐标(x,y,z,padding)
  float intensity;         ///< 点云强度值
  uint32_t t;             ///< 点的时间戳（纳秒级）
  uint16_t reflectivity;  ///< 反射率
  uint8_t ring;           ///< 扫描线编号
  uint16_t ambient;       ///< 环境光强度
  uint32_t range;         ///< 距离测量值
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  ///< Eigen内存对齐宏
};
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

/**
 * @brief 雷达数据预处理类
 * @details 负责处理不同类型雷达的点云数据，包括：
 *          1. 点云数据格式转换
 *          2. 点云过滤和降采样
 *          3. 特征提取（平面、边缘等）
 *          4. 时间戳处理和运动补偿
 *          5. 分帧处理和数据组织
 */
class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief 构造函数
   * @details 初始化预处理参数和特征提取阈值
   */
  Preprocess();
  
  /**
   * @brief 析构函数
   */
  ~Preprocess();
  
  /**
   * @brief Livox雷达切帧处理函数
   * @param msg Livox雷达原始数据消息
   * @param pcl_out 输出的点云帧队列
   * @param time_lidar 输出的帧时间戳队列
   * @param required_frame_num 需要切割的帧数
   * @param scan_count 当前扫描计数
   * @details 将Livox雷达的一帧数据按时间切分为多个子帧，用于高频率处理
   */
  void process_cut_frame_livox(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg, deque<PointCloudXYZI::Ptr> &pcl_out, deque<double> &time_lidar, const int required_frame_num, int scan_count);
  
  /**
   * @brief 通用雷达切帧处理函数
   * @param msg ROS2标准点云消息
   * @param pcl_out 输出的点云帧队列
   * @param time_lidar 输出的帧时间戳队列
   * @param required_frame_num 需要切割的帧数
   * @param scan_count 当前扫描计数
   * @details 支持Velodyne、Ouster、Hesai等雷达的切帧处理
   */
  void process_cut_frame_pcl2(const sensor_msgs::msg::PointCloud2::SharedPtr &msg, deque<PointCloudXYZI::Ptr> &pcl_out, deque<double> &time_lidar, const int required_frame_num, int scan_count);
 
  /**
   * @brief Livox雷达数据处理函数
   * @param msg Livox雷达原始数据消息
   * @param pcl_out 输出的处理后点云
   * @details 处理Livox雷达数据，包括过滤、特征提取等
   */
  void process(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  
  /**
   * @brief 通用雷达数据处理函数
   * @param msg ROS2标准点云消息
   * @param pcl_out 输出的处理后点云
   * @details 根据雷达类型自动选择对应的处理方法
   */
  void process(const sensor_msgs::msg::PointCloud2::SharedPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  
  /**
   * @brief 设置预处理参数
   * @param feat_en 是否启用特征提取
   * @param lid_type 雷达类型
   * @param bld 盲区距离阈值
   * @param pfilt_num 点过滤间隔
   */
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // ================ 公共数据成员 ================
  
  // sensor_msgs::msg::PointCloud2::SharedPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf;  ///< 全部点、角点、面点云
  PointCloudXYZI pl_buff[128];               ///< 每线的点云缓存（最多128条扫描线）
  vector<orgtype> typess[128];               ///< 每线点的特征信息（最多128条扫描线）
  
  float time_unit_scale;                     ///< 时间单位缩放因子
  int lidar_type;                           ///< 雷达类型
  int point_filter_num;                     ///< 点过滤数量（递减采样）
  int N_SCANS;                              ///< 扫描线数量
  int SCAN_RATE;                            ///< 扫描频率（Hz）
  int time_unit;                            ///< 时间单位类型
  double blind;                             ///< 盲区距离（米）
  double det_range;                         ///< 有效探测距离（米）
  bool given_offset_time;                   ///< 是否已给出时间偏移

  private:
  // ================ 私有成员函数 ================
  
  /**
   * @brief Livox Avia雷达数据处理器
   * @param msg Livox雷达数据消息
   * @details 处理Livox Avia特定格式的点云数据
   */
  void avia_handler(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg);
  
  /**
   * @brief Ouster 64线雷达数据处理器
   * @param msg ROS2点云消息
   * @details 处理Ouster OS1-64雷达数据
   */
  void oust64_handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);
  
  /**
   * @brief Velodyne雷达数据处理器
   * @param msg ROS2点云消息
   * @details 处理Velodyne VLP-16等型号雷达数据
   */
  void velodyne_handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);
  
  /**
   * @brief Hesai雷达数据处理器
   * @param msg ROS2点云消息
   * @details 处理禾赛XT32等型号雷达数据
   */
  void hesai_handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);
  
  /**
   * @brief 特征提取函数
   * @param pl 输入点云
   * @param types 点组织信息数组
   * @details 对点云进行特征分类，提取平面、边缘等特征
   */
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  
  /**
   * @brief 平面判断函数
   * @param pl 输入点云
   * @param types 点组织信息数组
   * @param i 当前点索引
   * @param i_nex 下一个点索引
   * @param curr_direct 当前平面法向量
   * @return 平面类型（1-平面，0-非平面）
   * @details 判断一组点是否形成平面特征
   */
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  
  /**
   * @brief 小平面判断函数
   * @param pl 输入点云
   * @param types 点组织信息数组
   * @param i_cur 当前点索引
   * @param i_nex 下一个点索引
   * @param curr_direct 当前平面法向量
   * @return 是否为小平面
   * @details 判断小尺寸平面特征
   */
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  
  /**
   * @brief 边缘跳跃判断函数
   * @param pl 输入点云
   * @param types 点组织信息数组
   * @param i 当前点索引
   * @param nor_dir 邻域方向
   * @return 是否为边缘跳跃
   * @details 检测点云中的边缘跳跃特征
   */
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);
  
  // ================ 私有成员变量 ================
  
  int group_size;                           ///< 平面检测的点组大小
  
  // 距离相关参数
  double disA, disB;                        ///< 距离计算系数A和B
  double inf_bound;                         ///< 无穷远边界
  
  // 平面检测阈值参数
  double limit_maxmid;                      ///< 最大值与中间值比值阈值
  double limit_midmin;                      ///< 中间值与最小值比值阈值
  double limit_maxmin;                      ///< 最大值与最小值比值阈值
  double p2l_ratio;                         ///< 点到线距离比值
  
  // 跳跃检测参数
  double jump_up_limit;                     ///< 上跳跃角度阈值
  double jump_down_limit;                   ///< 下跳跃角度阈值
  double cos160;                            ///< 160度角的余弦值
  
  // 边缘检测参数
  double edgea, edgeb;                      ///< 边缘检测系数A和B
  
  // 小平面检测参数
  double smallp_intersect;                  ///< 小平面交角阈值
  double smallp_ratio;                      ///< 小平面距离比值
  
  // 临时计算变量
  double vx, vy, vz;                        ///< 临时向量分量
};
