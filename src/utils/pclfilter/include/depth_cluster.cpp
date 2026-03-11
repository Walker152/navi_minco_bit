#include "depth_cluster.hpp"
using namespace std;
using namespace pcl;
using namespace Eigen;
namespace pclfilter
{
    DepthCluster::DepthCluster(float vertcal_resolution, float horizontal_resolution, int lidar_lines, int cluster_size)
        : vertcal_resolution_(vertcal_resolution), horizontal_resolution_(horizontal_resolution), lidar_lines_(lidar_lines), cluster_size_(cluster_size)
    {
        initParams();
    }

    void DepthCluster::initParams()
    {
        // 初始值清零
        sorted_Pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        // 列数 = 360/分辨率
        image_cols_ = int(360.0 / horizontal_resolution_);
        // 行数 = 雷达线数
        image_rows_ = lidar_lines_;
        // 垂直最大、小角
        vertical_angle_min_ = -27;
        vertical_angle_max_ = 32;
        sensor_height_ = 0.20;
        // 默认地面距离阈值（米）
        ground_distance_threshold_ = 0.3f;
        // 默认地面初始高度阈值（米）
        ground_height_threshold_ = 0.5f;
        // 默认地面最大斜率（度）
        ground_max_slope_angle_ = 30.0f;
    }

    void DepthCluster::setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &msg)
    {
        cout << "---------------- Algorithm Start ----------------" << endl;
        vector<vector<int>> label_image(image_rows_, vector<int>(image_cols_, -1));
        exactGroundPoints(const_cast<pcl::PointCloud<pcl::PointXYZ>::Ptr&>(msg), label_image);
        // TODO:OpenMP加速
        auto depth_image = generateDepthImage(const_cast<pcl::PointCloud<pcl::PointXYZ>::Ptr&>(msg));
        labelComponents(depth_image, label_image, const_cast<pcl::PointCloud<pcl::PointXYZ>::Ptr&>(msg));
    }

    vector<int> DepthCluster::exactGroundPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg, vector<vector<int>> &label_image)
    {
        clock_t time_start = clock();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>); // shared ptr, dont need to be deleted by self
        pcl::copyPointCloud(*msg, *cloud_copy);                                               //
        vector<int> ground_indices;
        pcl::PointCloud<pcl::PointXYZ>::Ptr initial_ground_points(new pcl::PointCloud<pcl::PointXYZ>); //
        extractInitialGroundPoint(cloud_copy, initial_ground_points);
        Eigen::Vector4f ground_params = estimatePlaneParams(initial_ground_points);
        ground_indices = exactGroundCloudIndices(msg, label_image, ground_params);
        clock_t time_end = clock();
        auto time_used = 1000.0 * double(time_end - time_start) / (double)CLOCKS_PER_SEC;
        cout << "Label Ground used time: " << time_used << " ms" << endl;
        return ground_indices;
    }

    vector<int> DepthCluster::exactGroundCloudIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg, vector<vector<int>> &label_image, Eigen::Vector4f &ground_params)
    {
        int clouds_size = msg->points.size();
        vector<int> ground_points_indeices;
        ground_points_indeices.reserve(clouds_size / 4);  // 预分配内存，地面点通常占1/4
        
        Eigen::Vector3f normal = ground_params.block<3, 1>(0, 0);
        float d = ground_params(3, 0);
        // 使用点到平面的绝对距离作为地面判定（单位: m），阈值可通过 setGroundDistanceThreshold 设置
        const float distance_threshold = ground_distance_threshold_;

        for (int i = 0; i < clouds_size; ++i)
        {
            const auto& point = msg->points[i];
            // 计算点到平面的有向距离: normal.dot(point) + d
            float signed_dist = point.x * normal(0) + point.y * normal(1) + point.z * normal(2) + d;
            float abs_dist = fabs(signed_dist);

            if (abs_dist < distance_threshold)
            {
                int row_index, col_index;
                if (!calculateCoordinate(point, row_index, col_index))
                {
                    continue;
                }
                label_image[row_index][col_index] = 0;  // 0 means ground
                ground_points_indeices.push_back(i);
            }
        }
        
        ground_points_indices_ = ground_points_indeices;
        return ground_points_indeices;
    }

    void DepthCluster::setGroundDistanceThreshold(float thresh)
    {
        ground_distance_threshold_ = thresh;
    }

    void DepthCluster::setGroundHeightThreshold(float thresh)
    {
        ground_height_threshold_ = thresh;
    }

    void DepthCluster::setGroundMaxSlopeAngle(float angle_deg)
    {
        ground_max_slope_angle_ = angle_deg;
    }

    void DepthCluster::extractInitialGroundPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &initial_ground_points)
    {
        sort(cloud->points.begin(), cloud->points.end(), [](const pcl::PointXYZ &a, const pcl::PointXYZ &b)
             { return a.z < b.z; });
        // NOTE: 原实现会删除所有 z < 1.5 * sensor_height_ 的点，
        // 这将误删地面点（z 接近 0），导致 "Too few points"。
        // 不再删除低高度点，直接使用排序后的前 N 个最小 z 值作为初始地面估计样本。
        int initial_ground_points_counts = 50;
        if (cloud->points.size() < static_cast<size_t>(initial_ground_points_counts))
        {
            cout << "Too few points" << endl;
            return;
        }

        float ground_height = 0.0f;
        for (int i = 0; i < initial_ground_points_counts; ++i)
        {
            ground_height += cloud->points[i].z;
        }
        ground_height /= static_cast<float>(initial_ground_points_counts);

        initial_ground_points->clear();
        for (const auto &point : cloud->points)
        {
            if (point.z < ground_height + 0.2f)
            {
                initial_ground_points->points.push_back(point);
            }
        }
    }

    Eigen::Vector4f DepthCluster::estimatePlaneParams(const pcl::PointCloud<pcl::PointXYZ>::Ptr &initial_ground_points)
    {
        Eigen::Matrix3f cov;
        Eigen::Vector4f pc_mean;
        pcl::computeMeanAndCovarianceMatrix(*initial_ground_points, cov, pc_mean);
        JacobiSVD<MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
        MatrixXf normal = (svd.matrixU().col(2));           // 第三列为地面平面的法向量。 3*1
        Eigen::Vector3f seeds_mean = pc_mean.head<3>();     // 前三个数 也就是xyz的均值 取地面点的均值
        float d = -(normal.transpose() * seeds_mean)(0, 0); // d!=0
        Eigen::Vector4f plane_parameters;
        plane_parameters(3, 0) = d;
        plane_parameters.block<3, 1>(0, 0) = normal.block<3, 1>(0, 0); //
        return plane_parameters;
    }
    // 根据点云坐标x,y,z生成深度图像
    vector<vector<PointInfo>> DepthCluster::generateDepthImage(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fused_ptr)
    {
        vector<vector<PointInfo>> depth_image(image_rows_, vector<PointInfo>(image_cols_, PointInfo(-1, -1)));
        clock_t time_start = clock();
        auto cloud_size = cloud_fused_ptr->points.size();
        
        const float MIN_DEPTH = 0.15f;
        const float MAX_DEPTH = 110.0f;

        for (int i = 0; i < cloud_size; ++i)
        {
            const pcl::PointXYZ &point = cloud_fused_ptr->points[i];
            
            // NaN检查
            if (isnan(point.x) || isnan(point.y) || isnan(point.z))
                continue;
            
            // 获取深度图坐标
            int row_index, col_index;
            if (!calculateCoordinate(point, row_index, col_index))
                continue;
            
            // 计算深度（优化：避免额外的中间变量）
            float dx = point.x, dy = point.y, dz = point.z;
            float depth = sqrt(dx * dx + dy * dy + dz * dz);
            
            // 范围检查
            if (depth < MIN_DEPTH || depth > MAX_DEPTH)
                continue;
            
            // 更新深度图
            depth_image[row_index][col_index].depth_ = depth;
            depth_image[row_index][col_index].index_ = i;
        }

        clock_t time_end = clock();
        auto time_used = 1000.0 * double(time_end - time_start) / (double)CLOCKS_PER_SEC;
        cout << "Depth Image used time: " << time_used << " ms" << endl;
        return depth_image;
    }

    void DepthCluster::labelComponents(const vector<vector<PointInfo>> &depth_image, vector<vector<int>> &label_image, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_msg)
    {
        // 八邻域
        static const vector<pair<int8_t, int8_t>> neighbor = {
            {1, 0}, {1, 1}, {1, -1}, {-1, 0}, {-1, -1}, {-1, 1}, {0, 1}, {0, -1}
        };

        int label_val = 1;
        int rows = image_rows_;
        int cols = image_cols_;
        const float DEPTH_THRESHOLD = 0.4f;

        vector<vector<int>> clusters_indices_vec;
        clock_t time_start = clock();
        
        for (int row = 0; row < rows; ++row)
        {
            for (int col = 0; col < cols; ++col)
            {
                // 未标记且深度有效
                if (label_image[row][col] == -1 && depth_image[row][col].depth_ > 0)
                {
                    queue<pair<int, int>> q;
                    q.push({row, col});
                    label_image[row][col] = label_val;

                    vector<int> cluster_indices_vec;
                    cluster_indices_vec.push_back(depth_image[row][col].index_);

                    while (!q.empty())
                    {
                        auto [cur_row, cur_col] = q.front();
                        q.pop();
                        
                        float cur_depth = depth_image[cur_row][cur_col].depth_;
                        
                        for (const auto& [dr, dc] : neighbor)
                        {
                            int nr = cur_row + dr;
                            int nc = cur_col + dc;
                            
                            pair<int, int> neigh_point = {nr, nc};
                            
                            // 边界检查和环绕处理
                            if (!warpPoint(neigh_point))
                                continue;
                            
                            nr = neigh_point.first;
                            nc = neigh_point.second;
                            
                            // 检查是否已标记或无效
                            if (label_image[nr][nc] != -1 || depth_image[nr][nc].depth_ <= 0)
                                continue;
                            
                            // 深度相似性判断
                            float depth_diff = fabs(cur_depth - depth_image[nr][nc].depth_);
                            if (depth_diff >= DEPTH_THRESHOLD)
                                continue;
                            
                            // 添加到队列
                            q.push({nr, nc});
                            label_image[nr][nc] = label_val;
                            cluster_indices_vec.push_back(depth_image[nr][nc].index_);
                        }
                    }
                    
                    // 只保存满足大小要求的聚类
                    if (cluster_indices_vec.size() > static_cast<size_t>(cluster_size_))
                    {
                        clusters_indices_vec.push_back(move(cluster_indices_vec));
                    }
                    
                    label_val++;
                }
            }
        }

        clock_t time_end = clock();
        auto time_used = 1000.0 * double(time_end - time_start) / (double)CLOCKS_PER_SEC;
        cout << "Cluster Algorithm used time: " << time_used << " ms" << endl;

        clusters_indices_vec_ = move(clusters_indices_vec);
    }

    bool DepthCluster::judgmentCondition(const vector<vector<PointInfo>> &depth_image, const pair<int, int> &target_point, const pair<int, int> &neigh_point)
    {
        float distance_sum = fabs(depth_image[target_point.first][target_point.second].depth_ - depth_image[neigh_point.first][neigh_point.second].depth_);
        return distance_sum < 0.4;
    }

    vector<vector<int>> DepthCluster::getClustersIndex()
    {
        return clusters_indices_vec_;
    }

    vector<int> DepthCluster::getGroundCloudIndices()
    {
        return ground_points_indices_;
    }
    vector<int> DepthCluster::getMergedClustersIndex()
    {
        vector<int> res_resturn;
        // 计算总大小以避免重复分配
        size_t total_size = 0;
        for (const auto& cluster : clusters_indices_vec_)
        {
            total_size += cluster.size();
        }
        res_resturn.reserve(total_size);
        
        for (const auto& cluster : clusters_indices_vec_)
        {
            res_resturn.insert(res_resturn.end(), cluster.begin(), cluster.end());
        }
        return res_resturn;
    }
    bool DepthCluster::warpPoint(pair<int, int> &pt)
    {
        if (pt.first < 0 || pt.first >= image_rows_)
            return false;
        if (pt.second < 0)
            pt.second += image_cols_;
        if (pt.second >= image_cols_)
            pt.second -= image_cols_;
        return true;
    }

    void DepthCluster::paramsReset()
    {
        sorted_Pointcloud_->clear();
    }

    bool DepthCluster::calculateCoordinate(const pcl::PointXYZ &point, int &row, int &col)
    {
        // 计算垂直角度
        float xy_dist = sqrt(point.x * point.x + point.y * point.y);
        float vertical_angle = atan2(point.z, xy_dist) * 180.0f / M_PI;
        
        // 计算水平角度
        float horizon_angle = atan2(point.x, point.y) * 180.0f / M_PI;
        
        // 计算行索引
        int row_index = int((vertical_angle - vertical_angle_min_) / vertcal_resolution_);
        
        // 边界检查
        if (row_index < 0)
            row_index = 0;
        else if (row_index >= image_rows_)
            row_index = image_rows_ - 1;

        // 计算列索引
        int col_index = -round(horizon_angle / horizontal_resolution_) + image_cols_ / 2;
        
        // 列环绕处理
        if (col_index >= image_cols_)
            col_index -= image_cols_;
        else if (col_index < 0)
            col_index += image_cols_;
        
        // 最终验证
        if (col_index < 0 || col_index >= image_cols_)
            return false;
        
        row = row_index;
        col = col_index;
        return true;
    }
}
