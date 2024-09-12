#ifndef ENTRANCE_OBSTACLE_H
#define ENTRANCE_OBSTACLE_H
#include <ros/ros.h>
#include <deque>
#include <string>
#include <thread>
#include <yaml-cpp/yaml.h>
//
#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include "feature_association.h"
//
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

class EntranceTracking
{
public:
	EntranceTracking(const std::string &map_path, ros::NodeHandle &nh) : map_path_(map_path), nh_(nh)
	{
		static_cast<void>(Initializtion());
	};

	~EntranceTracking() = default;

	void Start();

	void Stop();

	void ScanCallback(const sensor_msgs::PointCloud2ConstPtr &scan_msg);

	EntranceTracking(const EntranceTracking &) = delete;

	EntranceTracking &operator=(const EntranceTracking &) = delete;

	EntranceTracking(EntranceTracking &&) = delete;

	EntranceTracking &operator=(EntranceTracking &&) = delete;

private:
	bool LoadParameterForPoseTracking();

	void MainProcess();

	bool Initializtion();

	bool LoadMapFile(const std::string &file_name);

	void PublishMapEdge(const ros::WallTimerEvent &event);

	bool JudgeExistVertex(const std::vector<Vertex *> &vertices, const cv::Point2f &point,
						  size_t &index);

	std::deque<sensor_msgs::PointCloud2> scan_msgs_;

	std::thread main_processer_;

	std::shared_ptr<FeatureAssociation> feature_association_;

	volatile bool start_flag_ = false;

	std::string map_path_;

	ros::NodeHandle nh_;

	PoseTrackingParams params_;

	Map *map_;

	std::mutex scan_lock_;

	nav_msgs::Odometry odom_;
	nav_msgs::Path path_;

	ros::Publisher pub_vertex_;
	ros::Publisher pub_edge_;
	ros::Publisher pub_path_;
	ros::Publisher pub_odom_;
	tf::TransformBroadcaster tf_broadcaster_;

	ros::WallTimer globalmap_pub_timer;
};

#endif
