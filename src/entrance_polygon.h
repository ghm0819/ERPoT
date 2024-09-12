//
// Created by ghm on 2024/03/03.
//

#ifndef ENTRANCE_OBSTACLE_H
#define ENTRANCE_OBSTACLE_H
#include <ros/ros.h>
#include <deque>
#include <string>
#include <yaml-cpp/yaml.h>
//
#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>

struct MapParam
{
	MapParam() : origin_x_(0.0),
				 origin_y_(0.0),
				 origin_z_(0.0),
				 resolution_(0.15) {}

	float origin_x_;
	float origin_y_;
	float origin_z_;
	float resolution_;
};

using ConvexHulli = std::vector<cv::Point>;
using ConvexHullf = std::vector<cv::Point2f>;

class EntrancePolygon
{
public:
	explicit EntrancePolygon(const std::string &file_path) : file_path_(file_path)
	{
		static_cast<void>(Initializtion());
	};

	~EntrancePolygon() = default;

	bool Start();

	EntrancePolygon(const EntrancePolygon &) = delete;

	EntrancePolygon &operator=(const EntrancePolygon &) = delete;

	EntrancePolygon(EntrancePolygon &&) = delete;

	EntrancePolygon &operator=(EntrancePolygon &&) = delete;

private:
	MapParam map_params_;

	std::string file_path_;

	cv::Mat grid_map_;

	bool Initializtion();

	bool LoadConfigFile(const std::string &file_name);

	void ProcessOccupancyMap(std::vector<std::vector<ConvexHullf>> &find_contour);

	void ProcessOverlapContour(std::vector<ConvexHullf> &contours);

	void ProcessHugeContour(const std::vector<ConvexHullf> &contours, std::vector<std::vector<ConvexHullf>> &find_contours);

	bool JudgeSegmentValid(const cv::Point2f &p1, const cv::Point2f &p2, const cv::Point2f &p_source,
						   const cv::Point2f &p_target);

	bool JudgeInteraction(const ConvexHullf &points, const size_t source, const size_t target);

	bool LineInteraction(const cv::Point2f &l1p1, const cv::Point2f &l1p2, const cv::Point2f &l2p1, cv::Point2f l2p2);

	void ProcessContourInfo(ConvexHullf &contours, std::vector<ConvexHullf> &convex_hull);

	void SimplifyContour(ConvexHullf &contour);

	void CalculateLineParameters(const cv::Point2f &p1, const cv::Point2f &p2, double &a, double &b, double &c);

	bool JudgePointLeft(const cv::Point2f &p1, const cv::Point2f &p2, const cv::Point2f &p3);

	void CalculateNeighbourPixels(const cv::Mat &grid_map, const size_t i, const size_t j, int &num_free, int &num_unknown);

	void MapBinFileWrite(const std::vector<std::vector<ConvexHullf>> &contours);
};

#endif
