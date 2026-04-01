#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class VoxelIntegrator : public rclcpp::Node
{
public:
    VoxelIntegrator() : Node("voxel_integrator")
    {
        // 参数声明
        this->declare_parameter<double>("resolution", 0.1);
        this->declare_parameter<bool>("latch", false);
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<bool>("use_ground", true);
        this->declare_parameter<bool>("use_obstacles", true);
        this->declare_parameter<double>("voxel_filter_leaf_size", 0.05);
        this->declare_parameter<double>("publish_frequency", 1.0); // Hz

        resolution_ = this->get_parameter("resolution").as_double();
        latch_ = this->get_parameter("latch").as_bool();
        frame_id_ = this->get_parameter("frame_id").as_string();
        use_ground_ = this->get_parameter("use_ground").as_bool();
        use_obstacles_ = this->get_parameter("use_obstacles").as_bool();
        leaf_size_ = this->get_parameter("voxel_filter_leaf_size").as_double();
        publish_freq_ = this->get_parameter("publish_frequency").as_double();

        // 创建 octree
        octree_ = std::make_shared<octomap::OcTree>(resolution_);

        // 订阅点云
        if (use_ground_) {
            ground_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/ground_points_map", 10,
                std::bind(&VoxelIntegrator::groundCallback, this, std::placeholders::_1));
        }
        if (use_obstacles_) {
            obstacle_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/obstacle_clusters_map", 10,
                std::bind(&VoxelIntegrator::obstacleCallback, this, std::placeholders::_1));
        }

        // 发布 octomap 消息
        map_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("octomap", 10);

        // 定时发布地图
        if (publish_freq_ > 0) {
            timer_ = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / publish_freq_),
                std::bind(&VoxelIntegrator::publishMap, this));
        }
    }

private:
    double resolution_;
    bool latch_;
    std::string frame_id_;
    bool use_ground_;
    bool use_obstacles_;
    double leaf_size_;
    double publish_freq_;

    std::shared_ptr<octomap::OcTree> octree_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ground_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 处理地面点云（标记为自由空间）
    void groundCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) return;

        // 降采样以提升性能
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        vg.filter(*cloud);

        // 将每个点标记为自由空间（占据概率降低）
        for (const auto& pt : cloud->points) {
            octree_->updateNode(octomap::point3d(pt.x, pt.y, pt.z), false); // false = free
        }
    }

    // 处理障碍物点云（标记为占据）
    void obstacleCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) return;

        // 降采样
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        vg.filter(*cloud);

        // 将每个点标记为占据空间
        for (const auto& pt : cloud->points) {
            octree_->updateNode(octomap::point3d(pt.x, pt.y, pt.z), true); // true = occupied
        }
    }

    void publishMap()
    {
        if (!octree_) return;
        octomap_msgs::msg::Octomap msg;
        if (octomap_msgs::fullMapToMsg(*octree_, msg)) {
            msg.header.frame_id = frame_id_;
            msg.header.stamp = this->now();
            msg.id = "OcTree";
            msg.resolution = resolution_;
            map_pub_->publish(msg);
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxelIntegrator>());
    rclcpp::shutdown();
    return 0;
}