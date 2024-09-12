//
// Created by ghm on 2024/03/03.
//

#ifndef ENTRANCE_OBSTACLE_H
#define ENTRANCE_OBSTACLE_H
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Odometry.h>
#include <thread>
#include <deque>
#include <string>

//
#include <iostream>
// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "nearest_obstacle.h"

using PointType = pcl::PointXYZI;

class EntranceObstacle
{
public:
	explicit EntranceObstacle(const ros::NodeHandle &nh) : nh_(nh) {};

	~EntranceObstacle() = default;

	void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);

	bool Start();

	EntranceObstacle(const EntranceObstacle &) = delete;

	EntranceObstacle &operator=(const EntranceObstacle &) = delete;

	EntranceObstacle(EntranceObstacle &&) = delete;

	EntranceObstacle &operator=(EntranceObstacle &&) = delete;

private:
	bool LoadParameterForGroundSegmentation();

	std::shared_ptr<NearestObstacle> nearest_obstacle_ground_seg_;

	sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<PointType> cloud, const ros::Time &stamp,
									   std::string frame_id = "map");

	GroundSegmentationParams ground_segmentation_params_;

	std::mutex pose_lock_;

	ros::NodeHandle nh_;
	ros::Publisher pub_obstacle_;
	ros::Publisher pub_obstacle_scan_;
};

#endif
