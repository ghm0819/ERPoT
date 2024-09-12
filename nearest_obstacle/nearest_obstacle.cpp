#include "nearest_obstacle.h"

NearestObstacle::NearestObstacle(ros::NodeHandle *nh, const GroundSegmentationParams &params) : node_handle_(*nh), param_(params)
{
    ROS_INFO("Inititalizing PatchWork++...");

    ROS_INFO("Sensor Height: %f", param_.sensor_height_);
    ROS_INFO("Num of Iteration: %d", param_.num_iter_);
    ROS_INFO("Num of LPR: %d", param_.num_lpr_);
    ROS_INFO("Num of min. points: %d", param_.num_min_pts_);
    ROS_INFO("Seeds Threshold: %f", param_.th_seeds_);
    ROS_INFO("Distance Threshold: %f", param_.th_dist_);
    ROS_INFO("Max. range:: %f", param_.max_range_);
    ROS_INFO("Min. range:: %f", param_.min_range_);
    ROS_INFO("Normal vector threshold: %f", param_.uprightness_thr_);
    ROS_INFO("adaptive_seed_selection_margin: %f", param_.adaptive_seed_selection_margin_);
    ROS_INFO("Num. zones: %d", param_.num_zones_);

    std::cout << (boost::format("Num. sectors: %d, %d, %d, %d") % param_.num_sectors_each_zone_[0] % param_.num_sectors_each_zone_[1] % param_.num_sectors_each_zone_[2] % param_.num_sectors_each_zone_[3]).str() << std::endl;
    std::cout << (boost::format("Num. rings: %01d, %01d, %01d, %01d") % param_.num_rings_each_zone_[0] % param_.num_rings_each_zone_[1] % param_.num_rings_each_zone_[2] % param_.num_rings_each_zone_[3]).str() << std::endl;
    std::cout << (boost::format("elevation_thr_: %0.4f, %0.4f, %0.4f, %0.4f ") % param_.elevation_thr_[0] % param_.elevation_thr_[1] % param_.elevation_thr_[2] % param_.elevation_thr_[3]).str() << std::endl;
    std::cout << (boost::format("flatness_thr_: %0.4f, %0.4f, %0.4f, %0.4f ") % param_.flatness_thr_[0] % param_.flatness_thr_[1] % param_.flatness_thr_[2] % param_.flatness_thr_[3]).str() << std::endl;
    num_rings_of_interest_ = param_.elevation_thr_.size();

    ground_pc_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
    regionwise_ground_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
    regionwise_nonground_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);

    min_range_z2_ = (7 * param_.min_range_ + param_.max_range_) / 8.0;
    min_range_z3_ = (3 * param_.min_range_ + param_.max_range_) / 4.0;
    min_range_z4_ = (param_.min_range_ + param_.max_range_) / 2.0;

    min_ranges_ = {param_.min_range_, min_range_z2_, min_range_z3_, min_range_z4_};
    ring_sizes_ = {(min_range_z2_ - param_.min_range_) / param_.num_rings_each_zone_.at(0),
                   (min_range_z3_ - min_range_z2_) / param_.num_rings_each_zone_.at(1),
                   (min_range_z4_ - min_range_z3_) / param_.num_rings_each_zone_.at(2),
                   (param_.max_range_ - min_range_z4_) / param_.num_rings_each_zone_.at(3)};
    sector_sizes_ = {2 * M_PI / param_.num_sectors_each_zone_.at(0), 2 * M_PI / param_.num_sectors_each_zone_.at(1),
                     2 * M_PI / param_.num_sectors_each_zone_.at(2), 2 * M_PI / param_.num_sectors_each_zone_.at(3)};

    ROS_INFO("INITIALIZATION COMPLETE");

    for (int i = 0; i < param_.num_zones_; i++)
    {
        Zone z;
        initialize_zone(z, param_.num_sectors_each_zone_[i], param_.num_rings_each_zone_[i]);
        ConcentricZoneModel_.push_back(z);
    }
}

void NearestObstacle::initialize_zone(Zone &z, int num_sectors, int num_rings)
{
    z.clear();
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.reserve(1000);
    Ring ring;
    for (int i = 0; i < num_sectors; i++)
    {
        ring.emplace_back(cloud);
    }
    for (int j = 0; j < num_rings; j++)
    {
        z.emplace_back(ring);
    }
}

void NearestObstacle::flush_patches_in_zone(Zone &patches, int num_sectors, int num_rings)
{
    for (int i = 0; i < num_sectors; i++)
    {
        for (int j = 0; j < num_rings; j++)
        {
            if (!patches[j][i].points.empty())
                patches[j][i].points.clear();
        }
    }
}

void NearestObstacle::flush_patches(std::vector<Zone> &czm)
{
    for (int k = 0; k < param_.num_zones_; ++k)
    {
        for (int i = 0; i < param_.num_rings_each_zone_[k]; ++i)
        {
            for (int j = 0; j < param_.num_sectors_each_zone_[k]; ++j)
            {
                if (!czm[k][i][j].points.empty())
                    czm[k][i][j].points.clear();
            }
        }
    }
}

void NearestObstacle::estimate_plane(const pcl::PointCloud<pcl::PointXYZI> &ground)
{
    pcl::computeMeanAndCovarianceMatrix(ground, cov_, pc_mean_);
    // Singular Value Decomposition: SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov_, Eigen::DecompositionOptions::ComputeFullU);
    singular_values_ = svd.singularValues();

    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));

    // up
    if (normal_(2) < 0)
    {
        for (int i = 0; i < 3; i++)
            normal_(i) *= -1;
    }

    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean_.head<3>();

    // according to normal.T*[x,y,z] = -d, ground clearance, which > 0
    d_ = -(normal_.transpose() * seeds_mean)(0, 0);
}

void NearestObstacle::extract_initial_seeds(
    const int zone_idx, const pcl::PointCloud<pcl::PointXYZI> &p_sorted,
    pcl::PointCloud<pcl::PointXYZI> &init_seeds, double th_seed)
{
    init_seeds.points.clear();

    // LPR is the mean of low point representative
    double sum = 0;
    int cnt = 0;

    int init_idx = 0;
    if (zone_idx == 0)
    {
        for (int i = 0; i < p_sorted.points.size(); ++i)
        {
            if (p_sorted.points[i].z < param_.adaptive_seed_selection_margin_ * param_.sensor_height_)
            {
                ++init_idx;
            }
            else
            {
                break;
            }
        }
    }

    // Calculate the mean height value.
    for (int i = init_idx; i < p_sorted.points.size() && cnt < param_.num_lpr_; ++i)
    {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0; // in case divide by 0

    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (int i = 0; i < p_sorted.points.size(); i++)
    {
        if (p_sorted.points[i].z < lpr_height + th_seed)
        {
            init_seeds.points.push_back(p_sorted.points[i]);
        }
    }
}

void NearestObstacle::extract_initial_seeds(const int zone_idx, const pcl::PointCloud<pcl::PointXYZI> &p_sorted,
                                            pcl::PointCloud<pcl::PointXYZI> &init_seeds)
{
    init_seeds.points.clear();

    // LPR is the mean of low point representative
    double sum = 0;
    int cnt = 0;

    int init_idx = 0;
    if (zone_idx == 0)
    {
        for (int i = 0; i < p_sorted.points.size(); i++)
        {
            if (p_sorted.points[i].z < param_.adaptive_seed_selection_margin_ * param_.sensor_height_)
            {
                ++init_idx;
            }
            else
            {
                break;
            }
        }
    }

    // Calculate the mean height value.
    for (int i = init_idx; i < p_sorted.points.size() && cnt < param_.num_lpr_; i++)
    {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0; // in case divide by 0

    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (int i = 0; i < p_sorted.points.size(); i++)
    {
        if (p_sorted.points[i].z < lpr_height + param_.th_seeds_)
        {
            init_seeds.points.push_back(p_sorted.points[i]);
        }
    }
}

void NearestObstacle::estimate_ground(const pcl::PointCloud<pcl::PointXYZI> &cloud_in, pcl::PointCloud<pcl::PointXYZI> &cloud_obstacle,
                                      sensor_msgs::LaserScan &obstacle_scan)
{

    std::unique_lock<std::recursive_mutex> lock(mutex_);
    std::ofstream outfile_time(param_.outfile_time_, std::ios::app);
    TicToc time;
    ScanAll obstacle_info;
    obstacle_info.resize(static_cast<size_t>(360.0 / param_.horizontal_resolution_));
    cloud_obstacle.clear();

    flush_patches(ConcentricZoneModel_);
    pc2czm(cloud_in, ConcentricZoneModel_);

    int concentric_idx = 0;

    double t_sort = 0;

    std::vector<double> ringwise_flatness;

    for (int zone_idx = 0; zone_idx < param_.num_zones_; ++zone_idx)
    {
        auto zone = ConcentricZoneModel_[zone_idx];
        for (int ring_idx = 0; ring_idx < param_.num_rings_each_zone_[zone_idx]; ++ring_idx)
        {
            for (int sector_idx = 0; sector_idx < param_.num_sectors_each_zone_[zone_idx]; ++sector_idx)
            {

                if (zone[ring_idx][sector_idx].points.size() < param_.num_min_pts_)
                {
                    continue;
                }

                std::sort(zone[ring_idx][sector_idx].points.begin(), zone[ring_idx][sector_idx].points.end(), point_z_cmp);

                extract_piecewiseground(zone_idx, zone[ring_idx][sector_idx], regionwise_ground_, regionwise_nonground_);

                // Status of each patch
                // used in checking uprightness, elevation, and flatness, respectively
                const double ground_uprightness = normal_(2);
                const double ground_elevation = pc_mean_(2, 0);
                const double ground_flatness = singular_values_.minCoeff();
                const double line_variable = singular_values_(1) != 0 ? singular_values_(0) / singular_values_(1) : std::numeric_limits<double>::max();

                double heading = 0.0;
                for (int i = 0; i < 3; i++)
                {
                    heading += pc_mean_(i, 0) * normal_(i);
                }

                bool is_upright = ground_uprightness > param_.uprightness_thr_;
                bool is_near_zone = concentric_idx < num_rings_of_interest_;
                bool is_heading_outside = heading < 0.0;
                bool is_not_elevated = false;
                bool is_flat = false;

                if (is_near_zone)
                {
                    is_not_elevated = ground_elevation < param_.elevation_thr_[concentric_idx];
                    is_flat = ground_flatness < param_.flatness_thr_[concentric_idx];
                }

                if (is_upright && is_not_elevated && is_near_zone)
                {
                    update_elevation_[concentric_idx].push_back(ground_elevation);
                    update_flatness_[concentric_idx].push_back(ground_flatness);
                    ringwise_flatness.push_back(ground_flatness);
                }

                // Ground estimation based on conditions
                if (!is_upright)
                {
                    // do nothing
                }
                else if (!is_near_zone)
                {
                    NearestObstacleScanPointSelection(regionwise_nonground_, obstacle_info);
                }
                else if (!is_heading_outside)
                {
                    // do nothing
                }
                else if (is_not_elevated || is_flat)
                {
                    NearestObstacleScanPointSelection(regionwise_nonground_, obstacle_info);
                }
                else
                {
                    // do nothing
                }
                // Every regionwise_nonground is considered nonground.
            }
            concentric_idx++;
        }
    }

    update_elevation_thr();
    update_flatness_thr();

    GenerateObstacleCloud(cloud_in, obstacle_info, cloud_obstacle, obstacle_scan);
    outfile_time << cloud_in.header.stamp << " " << time.toc() << std::endl;
}

/****
 *
 *
 */
void NearestObstacle::NearestObstacleScanPointSelection(const pcl::PointCloud<pcl::PointXYZI> &non_ground_dst, ScanAll &obstacle_info)
{
    Eigen::MatrixXf points(non_ground_dst.points.size(), 3);
    int j = 0;
    for (auto &p : non_ground_dst.points)
    {
        points.row(j++) << p.x, p.y, p.z;
    }

    Eigen::VectorXf result = points * normal_;
    // threshold filter
    for (size_t r = 0; r < result.rows(); ++r)
    {
        if (result[r] < param_.sensor_height_v_)
        {
            //// 0-value corresponding the installing height of the LiDAR; sensor_height_v_ > 0
            const auto &point = non_ground_dst[r];
            auto point_index = ObtainIndexOfPoint(non_ground_dst[r]);
            obstacle_info[point_index].emplace_back(point.x, point.y, point.z, true, result[r] + d_);
        }
    }
}

size_t NearestObstacle::ObtainIndexOfPoint(const pcl::PointXYZI &point)
{
    auto angle = std::atan2(point.y, point.x) + M_PI;
    if (std::abs(angle - 2 * M_PI) < 1e-6)
    {
        angle = 0.0;
    }
    return static_cast<size_t>(angle / M_PI * 180 / param_.horizontal_resolution_);
}

void NearestObstacle::GenerateObstacleCloud(const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
                                            ScanAll &obstacle_info, pcl::PointCloud<pcl::PointXYZI> &cloud_obstacle, sensor_msgs::LaserScan &obstacle_scan)
{

    for (auto &scan_ray : obstacle_info)
    {
        std::sort(scan_ray.begin(), scan_ray.end(), [](const ScanPoint &point_a, const ScanPoint &point_b)
                  { return point_a.GetDistance() < point_b.GetDistance(); });
    }
    size_t index = 0U;
    for (const auto &obstacle : obstacle_info)
    {
        pcl::PointXYZI point{};
        if (obstacle.size() > 0)
        {
            point.x = static_cast<float>(obstacle.front().x());
            point.y = static_cast<float>(obstacle.front().y());
            point.z = static_cast<float>(obstacle.front().z());
            point.intensity = 0.0;
            obstacle_scan.ranges[index] = std::sqrt(point.x * point.x + point.y * point.y);
            obstacle_scan.intensities[index] = obstacle.front().GetHeightDistance();
        }
        else
        {
            point.x = 0.0f;
            point.y = 0.0f;
            point.z = 0.0f;
            point.intensity = 2.0;
            obstacle_scan.ranges[index] = 1500.0;
            obstacle_scan.intensities[index] = -1.0;
        }
        cloud_obstacle.points.push_back(point);
        ++index;
    }
}

void NearestObstacle::update_elevation_thr(void)
{
    for (int i = 0; i < num_rings_of_interest_; ++i)
    {
        if (update_elevation_[i].empty())
        {
            continue;
        }
        double update_mean = 0.0, update_stdev = 0.0;
        calc_mean_stdev(update_elevation_[i], update_mean, update_stdev);
        if (i == 0)
        {
            param_.elevation_thr_[i] = update_mean + 3 * update_stdev;
            param_.sensor_height_ = -update_mean;
        }
        else
        {
            param_.elevation_thr_[i] = update_mean + 2 * update_stdev;
        }

        int exceed_num = update_elevation_[i].size() - param_.max_elevation_storage_;
        if (exceed_num > 0)
        {
            update_elevation_[i].erase(update_elevation_[i].begin(), update_elevation_[i].begin() + exceed_num);
        }
    }

    return;
}

void NearestObstacle::update_flatness_thr(void)
{
    for (int i = 0; i < num_rings_of_interest_; ++i)
    {
        if (update_flatness_[i].empty())
            break;
        if (update_flatness_[i].size() <= 1)
            break;

        double update_mean = 0.0, update_stdev = 0.0;
        calc_mean_stdev(update_flatness_[i], update_mean, update_stdev);
        param_.flatness_thr_[i] = update_mean + update_stdev;

        int exceed_num = update_flatness_[i].size() - param_.max_flatness_storage_;
        if (exceed_num > 0)
        {
            update_flatness_[i].erase(update_flatness_[i].begin(), update_flatness_[i].begin() + exceed_num);
        }
    }

    return;
}

void NearestObstacle::extract_piecewiseground(
    const int zone_idx, const pcl::PointCloud<pcl::PointXYZI> &src,
    pcl::PointCloud<pcl::PointXYZI> &dst,
    pcl::PointCloud<pcl::PointXYZI> &non_ground_dst)
{

    // 0. Initialization
    if (!ground_pc_.empty())
    {
        ground_pc_.clear();
    }
    if (!dst.empty())
    {
        dst.clear();
    }
    if (!non_ground_dst.empty())
    {
        non_ground_dst.clear();
    }

    // 1. Region-wise Vertical Plane Fitting (R-VPF)
    // : removes potential vertical plane under the ground plane
    pcl::PointCloud<pcl::PointXYZI> src_wo_verticals;
    src_wo_verticals = src;

    if (param_.enable_RVPF_)
    {
        for (int i = 0; i < param_.num_iter_; ++i)
        {
            extract_initial_seeds(zone_idx, src_wo_verticals, ground_pc_, param_.th_seeds_v_);
            estimate_plane(ground_pc_);

            if (zone_idx == 0 && normal_(2) < param_.uprightness_thr_)
            {
                pcl::PointCloud<pcl::PointXYZI> src_tmp;
                src_tmp = src_wo_verticals;
                src_wo_verticals.clear();

                Eigen::MatrixXf points(src_tmp.points.size(), 3);
                int j = 0;
                for (auto &p : src_tmp.points)
                {
                    points.row(j++) << p.x, p.y, p.z;
                }
                // ground plane model
                Eigen::VectorXf result = points * normal_;

                for (int r = 0; r < result.rows(); ++r)
                {
                    if (result[r] < param_.th_dist_v_ - d_ && result[r] > -param_.th_dist_v_ - d_)
                    {
                        non_ground_dst.points.push_back(src_tmp[r]);
                    }
                    else
                    {
                        src_wo_verticals.points.push_back(src_tmp[r]);
                    }
                }
            }
            else
            {
                break;
            }
        }
    }

    extract_initial_seeds(zone_idx, src_wo_verticals, ground_pc_);
    estimate_plane(ground_pc_);

    // 2. Region-wise Ground Plane Fitting (R-GPF)
    // : fits the ground plane

    // pointcloud to matrix
    Eigen::MatrixXf points(src_wo_verticals.points.size(), 3);
    int j = 0;
    for (auto &p : src_wo_verticals.points)
    {
        points.row(j++) << p.x, p.y, p.z;
    }

    for (int i = 0; i < param_.num_iter_; i++)
    {
        ground_pc_.clear();

        // ground plane model
        Eigen::VectorXf result = points * normal_;
        // threshold filter
        for (int r = 0; r < result.rows(); r++)
        {
            if (i < param_.num_iter_ - 1)
            {
                if (result[r] < param_.th_dist_ - d_)
                {
                    ground_pc_.points.push_back(src_wo_verticals[r]);
                }
            }
            else
            { // Final stage
                if (result[r] < param_.th_dist_ - d_)
                {
                    dst.points.push_back(src_wo_verticals[r]);
                }
                else
                {
                    non_ground_dst.points.push_back(src_wo_verticals[r]);
                }
            }
        }

        if (i < param_.num_iter_ - 1)
        {
            estimate_plane(ground_pc_);
        }
        else
        {
            estimate_plane(dst);
        }
    }
}

void NearestObstacle::calc_mean_stdev(std::vector<double> vec, double &mean, double &stdev)
{
    if (vec.size() <= 1)
    {
        return;
    }

    mean = std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();

    for (int i = 0; i < vec.size(); ++i)
    {
        stdev += (vec.at(i) - mean) * (vec.at(i) - mean);
    }
    stdev /= vec.size() - 1;
    stdev = sqrt(stdev);
}

double NearestObstacle::xy2theta(const double &x, const double &y)
{ // 0 ~ 2 * PI
    double angle = atan2(y, x);
    return angle > 0 ? angle : 2 * M_PI + angle;
}

double NearestObstacle::xy2radius(const double &x, const double &y)
{
    return sqrt(pow(x, 2) + pow(y, 2));
}

void NearestObstacle::pc2czm(const pcl::PointCloud<pcl::PointXYZI> &src, std::vector<Zone> &czm)
{
    for (size_t i = 0; i < src.size(); ++i)
    {
        if ((!noise_idxs_.empty()) && (i == noise_idxs_.front()))
        {
            noise_idxs_.pop();
            continue;
        }

        pcl::PointXYZI pt = src.points[i];

        double r = xy2radius(pt.x, pt.y);
        if ((r <= param_.max_range_) && (r > param_.min_range_))
        {
            double theta = xy2theta(pt.x, pt.y);

            int zone_idx = 0;
            if (r < min_ranges_[1])
            {
                zone_idx = 0;
            }
            else if (r < min_ranges_[2])
            {
                zone_idx = 1;
            }
            else if (r < min_ranges_[3])
            {
                zone_idx = 2;
            }
            else
            {
                zone_idx = 3;
            }

            int ring_idx = std::min(static_cast<int>(((r - min_ranges_[zone_idx]) /
                                                      ring_sizes_[zone_idx])),
                                    param_.num_rings_each_zone_[zone_idx] - 1);
            int sector_idx = std::min(static_cast<int>((theta / sector_sizes_[zone_idx])),
                                      param_.num_sectors_each_zone_[zone_idx] - 1);

            czm[zone_idx][ring_idx][sector_idx].points.emplace_back(pt);
        }
    }
}