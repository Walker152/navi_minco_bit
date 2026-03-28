#include "depth_cluster.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <random>
#include <algorithm>

namespace pclfilter
{
    DepthCluster::DepthCluster(float vertcal_resolution, float horizontal_resolution, int lidar_lines, int min_cluster_size,
                               float ground_max_slope_angle, int normal_k, float depth_threshold,
                               bool use_euclidean, float euclidean_tolerance,
                               int euclidean_min_size, int euclidean_max_size,
                               float normal_curvature_threshold,
                               bool auto_estimate_reference_normal,
                               float sensor_tilt_angle_x, float sensor_tilt_angle_y,
                               bool use_adaptive_radius, bool use_temporal_filter, int temporal_window_size)
        : vertcal_resolution_(vertcal_resolution), horizontal_resolution_(horizontal_resolution),
          lidar_lines_(lidar_lines), min_cluster_size_(min_cluster_size),
          normal_estimation_k_(normal_k), depth_threshold_(depth_threshold),
          use_euclidean_(use_euclidean),
          euclidean_tolerance_(euclidean_tolerance),
          euclidean_min_cluster_size_(euclidean_min_size),
          euclidean_max_cluster_size_(euclidean_max_size),
          normal_curvature_threshold_(normal_curvature_threshold),
          auto_estimate_reference_normal_(auto_estimate_reference_normal),
          use_adaptive_radius_(use_adaptive_radius),
          use_temporal_filter_(use_temporal_filter),
          temporal_window_size_(temporal_window_size)
    {
        max_slope_angle_rad_ = ground_max_slope_angle * M_PI / 180.0f;
        if (!auto_estimate_reference_normal_)
            setSensorTilt(sensor_tilt_angle_x, sensor_tilt_angle_y);
        else
            reference_normal_ = Eigen::Vector3f(0, 0, 1);
        initParams();
    }

    void DepthCluster::initParams()
    {
        sorted_Pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        image_cols_ = int(360.0 / horizontal_resolution_);
        image_rows_ = lidar_lines_;
        vertical_angle_min_ = -27;
        vertical_angle_max_ = 32;
        sensor_height_ = 0.20;
    }

    void DepthCluster::setSensorTilt(float angle_x, float angle_y)
    {
        Eigen::Vector3f up(0, 0, 1);
        float rad_x = angle_x * M_PI / 180.0f;
        float rad_y = angle_y * M_PI / 180.0f;
        Eigen::AngleAxisf rot_x(rad_x, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf rot_y(rad_y, Eigen::Vector3f::UnitY());
        reference_normal_ = rot_y * rot_x * up;
        reference_normal_.normalize();
        cout << "User-defined reference normal: " << reference_normal_.transpose() << endl;
    }

    void DepthCluster::estimateReferenceNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, centroid);
        Eigen::Matrix3f cov;
        pcl::computeCovarianceMatrix(*cloud, centroid, cov);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
        Eigen::Vector3f normal = solver.eigenvectors().col(0);
        normal.normalize();
        if (normal.dot(Eigen::Vector3f::UnitZ()) < 0)
            normal = -normal;
        reference_normal_ = normal;
        cout << "Auto-estimated reference normal: " << reference_normal_.transpose() << endl;
    }

    bool DepthCluster::computeLocalNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                          const std::vector<int> &indices,
                                          Eigen::Vector3f &normal,
                                          float &curvature)
    {
        if (indices.size() < 3) return false;

        Eigen::Vector3f centroid(0, 0, 0);
        for (int idx : indices) {
            const auto &pt = cloud->points[idx];
            centroid += Eigen::Vector3f(pt.x, pt.y, pt.z);
        }
        centroid /= indices.size();

        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for (int idx : indices) {
            const auto &pt = cloud->points[idx];
            Eigen::Vector3f p(pt.x, pt.y, pt.z);
            Eigen::Vector3f diff = p - centroid;
            cov += diff * diff.transpose();
        }
        cov /= indices.size();

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
        if (solver.info() != Eigen::Success) return false;

        Eigen::Vector3f evals = solver.eigenvalues();
        normal = solver.eigenvectors().col(0);
        normal.normalize();
        curvature = evals(0) / (evals(0) + evals(1) + evals(2) + 1e-6);
        return true;
    }

    float DepthCluster::computeAdaptiveTolerance(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        if (cloud->empty()) return euclidean_tolerance_;
        int sample_size = std::min(1000, (int)cloud->size());
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);
        float sum_dist = 0;
        int valid = 0;
        for (int i = 0; i < sample_size; ++i) {
            int idx = rand() % cloud->size();
            std::vector<int> nn_indices;
            std::vector<float> nn_dist;
            if (kdtree.nearestKSearch(cloud->points[idx], 2, nn_indices, nn_dist) >= 2) {
                sum_dist += sqrt(nn_dist[1]);
                valid++;
            }
        }
        if (valid == 0) return euclidean_tolerance_;
        float avg_dist = sum_dist / valid;
        float radius = avg_dist * 3.0f;
        radius = std::max(0.2f, std::min(2.0f, radius));
        cout << "Adaptive clustering tolerance: " << radius << " m (avg dist = " << avg_dist << " m)" << endl;
        return radius;
    }

    vector<int> DepthCluster::temporalFilterGround(const vector<int> &current_ground,
                                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr &current_cloud,
                                                    double current_time)
    {
        FrameData current_frame;
        current_frame.ground_indices = current_ground;
        current_frame.cloud = current_cloud;
        current_frame.timestamp = current_time;
        frame_history_.push_back(current_frame);
        if (frame_history_.size() > (size_t)temporal_window_size_)
            frame_history_.pop_front();

        unordered_map<int, int> ground_votes;
        for (const auto &frame : frame_history_) {
            for (int idx : frame.ground_indices) {
                ground_votes[idx]++;
            }
        }

        int threshold = frame_history_.size() / 2 + 1;
        vector<int> filtered_ground;
        for (const auto &pair : ground_votes) {
            if (pair.second >= threshold)
                filtered_ground.push_back(pair.first);
        }
        cout << "Temporal filter: raw ground=" << current_ground.size()
             << ", filtered=" << filtered_ground.size() << endl;
        return filtered_ground;
    }

    void DepthCluster::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg)
    {
        cout << "============================================" << endl;
        cout << "Algorithm Start" << endl;
        cout << "Input points: " << msg->points.size() << endl;
        cout << "Max slope angle: " << (max_slope_angle_rad_ * 180.0f / M_PI) << " degrees" << endl;
        cout << "Normal estimation K: " << normal_estimation_k_ << endl;
        cout << "Curvature threshold: " << normal_curvature_threshold_ << endl;
        cout << "Auto estimate reference normal: " << (auto_estimate_reference_normal_ ? "yes" : "no") << endl;
        cout << "Use Euclidean clustering: " << (use_euclidean_ ? "yes" : "no") << endl;
        cout << "Adaptive radius: " << (use_adaptive_radius_ ? "yes" : "no") << endl;
        cout << "Temporal filter: " << (use_temporal_filter_ ? "yes" : "no") << endl;
        cout << "============================================" << endl;

        if (auto_estimate_reference_normal_)
            estimateReferenceNormal(msg);

        auto depth_image = generateDepthImage(msg);
        vector<vector<int>> label_image(image_rows_, vector<int>(image_cols_, -1));

        clock_t time_start = clock();
        vector<int> raw_ground = exactGroundCloudIndicesByLocalNormal(msg, label_image, depth_image);
        clock_t time_end = clock();
        auto time_used = 1000.0 * double(time_end - time_start) / (double)CLOCKS_PER_SEC;
        cout << "Ground detection time: " << time_used << " ms" << endl;

        if (use_temporal_filter_) {
            double now = double(clock()) / CLOCKS_PER_SEC; // 简单用程序时间，实际应用中可用ros时间
            ground_points_indices_ = temporalFilterGround(raw_ground, msg, now);
        } else {
            ground_points_indices_ = raw_ground;
        }
        cout << "Ground points: " << ground_points_indices_.size() << endl;

        if (use_euclidean_)
        {
            vector<bool> is_ground(msg->size(), false);
            for (int idx : ground_points_indices_) is_ground[idx] = true;
            vector<int> non_ground_indices;
            non_ground_indices.reserve(msg->size() - ground_points_indices_.size());
            for (size_t i = 0; i < msg->size(); ++i)
                if (!is_ground[i]) non_ground_indices.push_back(i);
            cout << "Non-ground points: " << non_ground_indices.size() << endl;

            if (!non_ground_indices.empty())
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::copyPointCloud(*msg, non_ground_indices, *non_ground_cloud);
                vector<pcl::PointIndices> cluster_indices = euclideanClustering(non_ground_cloud, non_ground_indices);
                clusters_indices_vec_.clear();
                for (const auto &indices : cluster_indices)
                {
                    vector<int> cluster;
                    cluster.reserve(indices.indices.size());
                    for (int idx : indices.indices)
                        cluster.push_back(non_ground_indices[idx]);
                    clusters_indices_vec_.push_back(cluster);
                }
            }
            else
            {
                clusters_indices_vec_.clear();
            }
            cout << "Euclidean clusters found: " << clusters_indices_vec_.size() << endl;
        }
        else
        {
            labelComponents(depth_image, label_image, msg);
            cout << "Depth clusters found: " << clusters_indices_vec_.size() << endl;
        }
    }

    vector<int> DepthCluster::exactGroundCloudIndicesByLocalNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg,
                                                                    vector<vector<int>> &label_image,
                                                                    const vector<vector<PointInfo>> &depth_image)
    {
        vector<int> ground_indices;
        int clouds_size = msg->points.size();
        if (clouds_size == 0) return ground_indices;

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(msg);

        vector<Eigen::Vector3f> normals(clouds_size);
        vector<float> curvatures(clouds_size, 1.0f);
        vector<bool> is_ground(clouds_size, false);
        int min_neighbors = 5;

        for (int i = 0; i < clouds_size; ++i)
        {
            const auto &point = msg->points[i];
            std::vector<int> neighbor_indices;
            std::vector<float> distances;
            int found = kdtree.nearestKSearch(point, normal_estimation_k_, neighbor_indices, distances);
            if (found < min_neighbors) continue;

            Eigen::Vector3f normal;
            float curvature;
            if (!computeLocalNormal(msg, neighbor_indices, normal, curvature))
                continue;

            if (normal.dot(reference_normal_) < 0)
                normal = -normal;
            normals[i] = normal;
            curvatures[i] = curvature;

            float cos_angle = normal.dot(reference_normal_);
            cos_angle = std::max(-1.0f, std::min(1.0f, cos_angle));
            float angle_deg = acos(cos_angle) * 180.0f / M_PI;

            if (angle_deg <= (max_slope_angle_rad_ * 180.0f / M_PI) && curvature < normal_curvature_threshold_)
            {
                is_ground[i] = true;
            }
        }

        int reliable_count = 0;
        for (int i = 0; i < clouds_size; ++i)
            if (is_ground[i]) reliable_count++;
        cout << "Reliable ground seeds: " << reliable_count << endl;

        if (reliable_count == 0)
        {
            cout << "No reliable ground seeds found!" << endl;
            return ground_indices;
        }

        vector<vector<bool>> pixel_seed(image_rows_, vector<bool>(image_cols_, false));
        vector<vector<bool>> pixel_ground(image_rows_, vector<bool>(image_cols_, false));

        for (int i = 0; i < clouds_size; ++i)
        {
            if (is_ground[i])
            {
                int row, col;
                if (calculateCoordinate(msg->points[i], row, col))
                    pixel_seed[row][col] = true;
            }
        }

        queue<pair<int, int>> q;
        for (int r = 0; r < image_rows_; ++r)
            for (int c = 0; c < image_cols_; ++c)
                if (pixel_seed[r][c])
                {
                    pixel_ground[r][c] = true;
                    q.push({r, c});
                }

        const int dr[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
        const int dc[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

        while (!q.empty())
        {
            auto [r, c] = q.front(); q.pop();
            float center_depth = depth_image[r][c].depth_;
            for (int k = 0; k < 8; ++k)
            {
                int nr = r + dr[k];
                int nc = c + dc[k];
                if (nr < 0 || nr >= image_rows_ || nc < 0 || nc >= image_cols_) continue;
                if (pixel_ground[nr][nc]) continue;
                if (depth_image[nr][nc].indices.empty()) continue;

                float neigh_depth = depth_image[nr][nc].depth_;
                if (fabs(center_depth - neigh_depth) > depth_threshold_) continue;

                const auto &indices = depth_image[nr][nc].indices;
                int count_ok = 0;
                for (int idx : indices)
                {
                    float cos_a = normals[idx].dot(reference_normal_);
                    cos_a = std::max(-1.0f, std::min(1.0f, cos_a));
                    float ang = acos(cos_a) * 180.0f / M_PI;
                    if (ang <= (max_slope_angle_rad_ * 180.0f / M_PI) + 2.0f)
                        count_ok++;
                }
                if (count_ok >= indices.size() * 0.5)
                {
                    pixel_ground[nr][nc] = true;
                    q.push({nr, nc});
                    for (int idx : indices)
                        is_ground[idx] = true;
                }
            }
        }

        for (size_t i = 0; i < clouds_size; ++i)
            if (is_ground[i])
                ground_indices.push_back(i);

        for (int idx : ground_indices)
        {
            int row, col;
            if (calculateCoordinate(msg->points[idx], row, col))
                label_image[row][col] = 0;
        }

        cout << "Final ground points: " << ground_indices.size() << endl;
        return ground_indices;
    }

    vector<pcl::PointIndices> DepthCluster::euclideanClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                                 const vector<int> &indices_to_cluster)
    {
        vector<pcl::PointIndices> cluster_indices;
        if (cloud->empty()) return cluster_indices;

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        float tolerance = euclidean_tolerance_;
        if (use_adaptive_radius_) {
            tolerance = computeAdaptiveTolerance(cloud);
        }
        ec.setClusterTolerance(tolerance);
        ec.setMinClusterSize(euclidean_min_cluster_size_);
        ec.setMaxClusterSize(euclidean_max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);
        return cluster_indices;
    }

    vector<vector<PointInfo>> DepthCluster::generateDepthImage(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fused_ptr)
    {
        vector<vector<PointInfo>> depth_image(image_rows_, vector<PointInfo>(image_cols_));
        clock_t time_start = clock();
        auto cloud_size = cloud_fused_ptr->points.size();

        const float MIN_DEPTH = 0.15f;
        const float MAX_DEPTH = 110.0f;

        for (int i = 0; i < cloud_size; ++i)
        {
            const pcl::PointXYZ &point = cloud_fused_ptr->points[i];
            if (isnan(point.x) || isnan(point.y) || isnan(point.z))
                continue;

            int row, col;
            if (!calculateCoordinate(point, row, col))
                continue;

            float depth = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
            if (depth < MIN_DEPTH || depth > MAX_DEPTH)
                continue;

            if (depth_image[row][col].depth_ < 0)
            {
                depth_image[row][col].depth_ = depth;
                depth_image[row][col].indices.push_back(i);
            }
            else
            {
                depth_image[row][col].indices.push_back(i);
            }
        }

        clock_t time_end = clock();
        auto time_used = 1000.0 * double(time_end - time_start) / (double)CLOCKS_PER_SEC;
        cout << "Depth Image used time: " << time_used << " ms" << endl;
        return depth_image;
    }

    void DepthCluster::labelComponents(const vector<vector<PointInfo>> &depth_image,
                                       vector<vector<int>> &label_image,
                                       const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_msg)
    {
        static const vector<pair<int8_t, int8_t>> neighbor = {
            {1,0},{1,1},{1,-1},{-1,0},{-1,-1},{-1,1},{0,1},{0,-1}
        };
        int label_val = 1;
        int rows = image_rows_, cols = image_cols_;
        vector<vector<int>> clusters;
        clock_t time_start = clock();

        for (int r = 0; r < rows; ++r)
        {
            for (int c = 0; c < cols; ++c)
            {
                if (label_image[r][c] == -1 && !depth_image[r][c].indices.empty())
                {
                    queue<pair<int,int>> q;
                    q.push({r,c});
                    label_image[r][c] = label_val;
                    vector<int> cluster_indices;
                    cluster_indices.insert(cluster_indices.end(), depth_image[r][c].indices.begin(), depth_image[r][c].indices.end());

                    while (!q.empty())
                    {
                        auto [cr, cc] = q.front(); q.pop();
                        float cur_depth = depth_image[cr][cc].depth_;
                        for (auto [dr, dc] : neighbor)
                        {
                            int nr = cr + dr, nc = cc + dc;
                            pair<int,int> np = {nr, nc};
                            if (!warpPoint(np)) continue;
                            nr = np.first; nc = np.second;
                            if (label_image[nr][nc] != -1 || depth_image[nr][nc].indices.empty()) continue;
                            float diff = fabs(cur_depth - depth_image[nr][nc].depth_);
                            if (diff >= depth_threshold_) continue;
                            q.push({nr, nc});
                            label_image[nr][nc] = label_val;
                            cluster_indices.insert(cluster_indices.end(), depth_image[nr][nc].indices.begin(), depth_image[nr][nc].indices.end());
                        }
                    }
                    label_val++;
                    if (cluster_indices.size() > static_cast<size_t>(min_cluster_size_))
                        clusters.push_back(cluster_indices);
                }
            }
        }

        clock_t time_end = clock();
        auto time_used = 1000.0 * double(time_end - time_start) / (double)CLOCKS_PER_SEC;
        cout << "Clustering time: " << time_used << " ms" << endl;
        clusters_indices_vec_ = clusters;
    }

    bool DepthCluster::judgmentCondition(const vector<vector<PointInfo>> &depth_image,
                                         const pair<int,int> &target, const pair<int,int> &neigh)
    {
        float diff = fabs(depth_image[target.first][target.second].depth_ -
                          depth_image[neigh.first][neigh.second].depth_);
        return diff < depth_threshold_;
    }

    bool DepthCluster::calculateCoordinate(const pcl::PointXYZ &point, int &row, int &col)
    {
        float xy_dist = sqrt(point.x*point.x + point.y*point.y);
        float v_angle = atan2(point.z, xy_dist) * 180 / M_PI;
        float h_angle = atan2(point.x, point.y) * 180 / M_PI;
        int r = int((v_angle - vertical_angle_min_) / vertcal_resolution_);
        r = max(0, min(r, image_rows_-1));
        int c = -round(h_angle / horizontal_resolution_) + image_cols_/2;
        if (c >= image_cols_) c -= image_cols_;
        if (c < 0 || c >= image_cols_) return false;
        row = r; col = c;
        return true;
    }

    bool DepthCluster::warpPoint(pair<int,int> &pt)
    {
        if (pt.first < 0 || pt.first >= image_rows_) return false;
        if (pt.second < 0) pt.second += image_cols_;
        if (pt.second >= image_cols_) pt.second -= image_cols_;
        return true;
    }

    vector<vector<int>> DepthCluster::getClustersIndex() { return clusters_indices_vec_; }
    vector<int> DepthCluster::getGroundCloudIndices() { return ground_points_indices_; }
    vector<int> DepthCluster::getMergedClustersIndex()
    {
        vector<int> res;
        for (auto &c : clusters_indices_vec_)
            res.insert(res.end(), c.begin(), c.end());
        return res;
    }
    void DepthCluster::paramsReset() { sorted_Pointcloud_->clear(); }
}