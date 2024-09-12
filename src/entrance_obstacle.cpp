//
// Created by ghm on 2024/03/03.
//

#include "entrance_obstacle.h"

bool EntranceObstacle::Start()
{
    // for segmentation
    if (!LoadParameterForGroundSegmentation())
    {
        ROS_ERROR("params load error!");
        return false;
    }

    nearest_obstacle_ground_seg_ = std::make_shared<NearestObstacle>(&nh_,
                                                                     ground_segmentation_params_);

    std::cout << "start planar obstacle generating..." << std::endl;
    pub_obstacle_ = nh_.advertise<sensor_msgs::PointCloud2>("obstacle", 100, true);
    pub_obstacle_scan_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 100, true);
    return true;
}

bool EntranceObstacle::LoadParameterForGroundSegmentation()
{
    nh_.param("sensor_height", ground_segmentation_params_.sensor_height_, 1.723);
    nh_.param("horizontal_resolution", ground_segmentation_params_.horizontal_resolution_, 0.2);
    nh_.param("sensor_height_v", ground_segmentation_params_.sensor_height_v_, 0.25);
    nh_.param("num_iter", ground_segmentation_params_.num_iter_, 3);
    nh_.param("num_lpr", ground_segmentation_params_.num_lpr_, 20);
    nh_.param("num_min_pts", ground_segmentation_params_.num_min_pts_, 10);
    nh_.param("th_seeds", ground_segmentation_params_.th_seeds_, 0.4);
    nh_.param("th_dist", ground_segmentation_params_.th_dist_, 0.3);
    nh_.param("th_seeds_v", ground_segmentation_params_.th_seeds_v_, 0.4);
    nh_.param("th_dist_v", ground_segmentation_params_.th_dist_v_, 0.3);
    nh_.param("max_r", ground_segmentation_params_.max_range_, 80.0);
    nh_.param("min_r", ground_segmentation_params_.min_range_, 2.7);
    nh_.param("uprightness_thr", ground_segmentation_params_.uprightness_thr_, 0.5);
    nh_.param("adaptive_seed_selection_margin",
              ground_segmentation_params_.adaptive_seed_selection_margin_, -1.1);
    nh_.param("max_flatness_storage", ground_segmentation_params_.max_flatness_storage_, 1000);
    nh_.param("max_elevation_storage", ground_segmentation_params_.max_elevation_storage_, 1000);
    nh_.param("enable_RVPF", ground_segmentation_params_.enable_RVPF_, true);

    nh_.getParam("czm/num_zones", ground_segmentation_params_.num_zones_);
    nh_.getParam("czm/num_sectors_each_zone", ground_segmentation_params_.num_sectors_each_zone_);
    nh_.getParam("czm/mum_rings_each_zone", ground_segmentation_params_.num_rings_each_zone_);
    nh_.getParam("czm/elevation_thresholds", ground_segmentation_params_.elevation_thr_);
    nh_.getParam("czm/flatness_thresholds", ground_segmentation_params_.flatness_thr_);

    nh_.param<std::string>("outfile_time", ground_segmentation_params_.outfile_time_, "~/Download/scan_time.txt");

    if (ground_segmentation_params_.num_zones_ != 4 ||
        ground_segmentation_params_.num_sectors_each_zone_.size() !=
            ground_segmentation_params_.num_rings_each_zone_.size())
    {
        ROS_ERROR("Some parameters are wrong! Check the num_zones and num_rings/sectors_each_zone");
        return false;
    }
    if (ground_segmentation_params_.elevation_thr_.size() !=
        ground_segmentation_params_.flatness_thr_.size())
    {
        ROS_ERROR("Some parameters are wrong! Check the elevation/flatness_thresholds");
        return false;
    }

    return true;
}

sensor_msgs::PointCloud2 EntranceObstacle::cloud2msg(pcl::PointCloud<PointType> cloud,
                                                     const ros::Time &stamp, std::string frame_id)
{
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.stamp = stamp;
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}

void EntranceObstacle::PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
{
    pcl::PointCloud<PointType> pc_curr;
    pcl::PointCloud<PointType> pc_nearest_obstacle;
    sensor_msgs::LaserScan scan_obstacle;

    /// Initilization scan
    static uint32_t seq = 0;
    scan_obstacle.header.seq = seq++;
    scan_obstacle.header.stamp = cloudMsg->header.stamp;
    scan_obstacle.header.frame_id = "laser";
    scan_obstacle.angle_increment = ground_segmentation_params_.horizontal_resolution_ / 180 * M_PI;
    scan_obstacle.angle_min = -M_PI;
    scan_obstacle.angle_max = M_PI - scan_obstacle.angle_increment;
    scan_obstacle.range_min = 0.1;
    scan_obstacle.range_max = 150.0;
    scan_obstacle.ranges.resize(static_cast<std::size_t>(360.0 / ground_segmentation_params_.horizontal_resolution_));
    scan_obstacle.intensities.resize(static_cast<std::size_t>(360.0 / ground_segmentation_params_.horizontal_resolution_));

    pcl::fromROSMsg(*cloudMsg, pc_curr);
    std::vector<int> mapping_indices;
    pcl::removeNaNFromPointCloud(pc_curr, pc_curr, mapping_indices);

    nearest_obstacle_ground_seg_->estimate_ground(pc_curr, pc_nearest_obstacle, scan_obstacle);
    pub_obstacle_.publish(cloud2msg(pc_nearest_obstacle, cloudMsg->header.stamp, cloudMsg->header.frame_id));
    pub_obstacle_scan_.publish(scan_obstacle);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle");
    ros::NodeHandle nh("~");

    EntranceObstacle enter_node(nh);

    static_cast<void>(enter_node.Start());
    std::string cloud_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/pointcloud");

    ros::Subscriber point_cloud_sub_ = nh.subscribe(cloud_topic, 1,
                                                    &EntranceObstacle::PointCloudCallback, &enter_node);
    ros::spin();
}