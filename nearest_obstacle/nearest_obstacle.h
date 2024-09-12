#ifndef NEAREST_OBSTACLE_H
#define NEAREST_OBSTACLE_H

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <Eigen/Dense>
#include <boost/format.hpp>
#include <numeric>
#include <queue>
#include <mutex>
#include "math.h"
#include <sstream>
#include <vector>
#include <map>
#include <iostream>
#include <tf/tf.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include "scan_point.h"

#define MARKER_Z_VALUE -2.2
#define UPRIGHT_ENOUGH 0.55
#define FLAT_ENOUGH 0.2
#define TOO_HIGH_ELEVATION 0.0
#define TOO_TILTED 1.0

#define NUM_HEURISTIC_MAX_PTS_IN_PATCH 3000

using Eigen::JacobiSVD;
using Eigen::MatrixXf;
using Eigen::VectorXf;

struct PointXYZIRT
{
    PCL_ADD_POINT4D; // quad-word XYZ
    float intensity; ///< laser intensity reading
    uint16_t ring;   ///< laser ring number
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))

class TicToc
{
public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

struct GroundSegmentationParams
{
    GroundSegmentationParams() : sensor_height_(1.723),
                                 horizontal_resolution_(0.2),
                                 sensor_height_v_(0.25),
                                 num_iter_(3),
                                 num_lpr_(20),
                                 num_min_pts_(10),
                                 th_seeds_(0.4),
                                 th_dist_(0.3),
                                 th_seeds_v_(0.4),
                                 th_dist_v_(0.3),
                                 max_range_(80.0),
                                 min_range_(2.7),
                                 uprightness_thr_(0.5),
                                 adaptive_seed_selection_margin_(-1.1),
                                 max_flatness_storage_(1000),
                                 max_elevation_storage_(1000),
                                 enable_RVPF_(true),
                                 num_zones_(4) {}

    double sensor_height_;
    double horizontal_resolution_;
    double sensor_height_v_;
    int num_iter_;
    int num_lpr_;
    int num_min_pts_;
    double th_seeds_;
    double th_dist_;
    double th_seeds_v_;
    double th_dist_v_;
    double max_range_;
    double min_range_;
    double uprightness_thr_;
    double adaptive_seed_selection_margin_;
    int max_flatness_storage_;
    int max_elevation_storage_;
    bool enable_RVPF_;
    int num_zones_;
    std::vector<int> num_sectors_each_zone_;
    std::vector<int> num_rings_each_zone_;
    std::vector<double> elevation_thr_;
    std::vector<double> flatness_thr_;

    std::string outfile_time_;
};

class NearestObstacle
{
public:
    typedef std::vector<pcl::PointCloud<pcl::PointXYZI>> Ring;
    typedef std::vector<Ring> Zone;

    NearestObstacle() {};

    NearestObstacle(ros::NodeHandle *nh, const GroundSegmentationParams &params = GroundSegmentationParams());

    void estimate_ground(const pcl::PointCloud<pcl::PointXYZI> &cloud_in, pcl::PointCloud<pcl::PointXYZI> &cloud_obstacle,
                         sensor_msgs::LaserScan &obstacle_scan);

private:
    ros::NodeHandle node_handle_;

    std::recursive_mutex mutex_;

    GroundSegmentationParams param_;

    int num_rings_of_interest_;

    double uprightness_thr_;
    double min_range_z2_; // 12.3625
    double min_range_z3_; // 22.025
    double min_range_z4_; // 41.35

    std::vector<double> update_flatness_[4];
    std::vector<double> update_elevation_[4];

    float d_;

    VectorXf normal_;
    MatrixXf pnormal_;
    VectorXf singular_values_;
    Eigen::Matrix3f cov_;
    Eigen::Vector4f pc_mean_;

    // For visualization
    bool visualize_;

    std::vector<double> sector_sizes_;
    std::vector<double> ring_sizes_;
    std::vector<double> min_ranges_;

    std::queue<size_t> noise_idxs_;

    std::vector<Zone> ConcentricZoneModel_;

    pcl::PointCloud<pcl::PointXYZI> ground_pc_;

    inline static bool point_z_cmp(pcl::PointXYZI a, pcl::PointXYZI b) { return a.z < b.z; }

    pcl::PointCloud<pcl::PointXYZI> regionwise_ground_, regionwise_nonground_;

    void initialize_zone(Zone &z, int num_sectors, int num_rings);

    void flush_patches_in_zone(Zone &patches, int num_sectors, int num_rings);
    void flush_patches(std::vector<Zone> &czm);

    void pc2czm(const pcl::PointCloud<pcl::PointXYZI> &src, std::vector<Zone> &czm);

    void calc_mean_stdev(std::vector<double> vec, double &mean, double &stdev);

    void update_elevation_thr();
    void update_flatness_thr();
    ///////////
    void NearestObstacleScanPointSelection(const pcl::PointCloud<pcl::PointXYZI> &non_ground_dst, ScanAll &obstacle_info);

    size_t ObtainIndexOfPoint(const pcl::PointXYZI &point);

    void GenerateObstacleCloud(const pcl::PointCloud<pcl::PointXYZI> &cloud_in, ScanAll &obstacle_info,
                               pcl::PointCloud<pcl::PointXYZI> &cloud_obstacle, sensor_msgs::LaserScan &obstacle_scan);

    ///////////

    double xy2theta(const double &x, const double &y);

    double xy2radius(const double &x, const double &y);

    void estimate_plane(const pcl::PointCloud<pcl::PointXYZI> &ground);

    void extract_piecewiseground(
        const int zone_idx, const pcl::PointCloud<pcl::PointXYZI> &src,
        pcl::PointCloud<pcl::PointXYZI> &dst,
        pcl::PointCloud<pcl::PointXYZI> &non_ground_dst);

    void extract_initial_seeds(
        const int zone_idx, const pcl::PointCloud<pcl::PointXYZI> &p_sorted,
        pcl::PointCloud<pcl::PointXYZI> &init_seeds);

    void extract_initial_seeds(
        const int zone_idx, const pcl::PointCloud<pcl::PointXYZI> &p_sorted,
        pcl::PointCloud<pcl::PointXYZI> &init_seeds, double th_seed);
};
#endif