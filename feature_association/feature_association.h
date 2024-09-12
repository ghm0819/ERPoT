#ifndef FEATURE_ASSOCIATION_H
#define FEATURE_ASSOCIATION_H
#include <ros/ros.h>
#include <ctime>
#include <cstdlib>
#include <chrono>
#include "vertex.h"
#include "edge.h"
#include "polygon.h"
#include "map.h"
#include "scan_factor.hpp"

struct PoseTrackingParams
{
    PoseTrackingParams() : start_x(0.0),
                           start_y(0.0),
                           start_yaw(0.0),
                           tracking_distance_threshold(1.0),
                           tracking_angle_threshold(0.2),
                           distance_association_vertex(2.0),
                           distance_association_polygon(3.0),
                           factor_vertex(1.0),
                           factor_edge(1.0),
                           corner_threshold(1.0),
                           edge_threshold(0.1),
                           corner_feature_num(20),
                           edge_feature_num(100),
                           edge_feature_selection(false),
                           constant_mode(true) {}

    double start_x;
    double start_y;
    double start_yaw;
    double tracking_distance_threshold;
    double tracking_angle_threshold;
    double distance_association_vertex;
    double distance_association_polygon;
    double factor_vertex;
    double factor_edge;
    double corner_threshold;
    double edge_threshold;
    int corner_feature_num;
    int edge_feature_num;
    bool edge_feature_selection;
    bool constant_mode;

    std::string outfile_pose_;
    std::string outfile_time_;
};

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

class FeatureAssociation
{
public:
    FeatureAssociation() {};

    FeatureAssociation(Map *map, const PoseTrackingParams &params);

    PoseInfo PoseTracking(const pcl::PointCloud<pcl::PointXYZI> &obstacle_scan);

    ~FeatureAssociation() = default;

private:
    // Every private member variable is written with the undescore("_") in its end.
    void PredictCurrentPose();

    void UpdateIncrePose();

    void MotionCompensation(pcl::PointXYZI &point_in);

    void DataInitialization(const pcl::PointCloud<pcl::PointXYZI> &obstacle_scan, std::vector<int> &indexs, std::vector<float> &ranges);

    void CalculateSmoothness(const std::vector<float> &ranges, std::vector<float> &curvature, std::vector<int> &neighbor_picked,
                             std::vector<Smooth> &smooth, std::vector<int> &label);

    void MarkOccludedPoints(const std::vector<float> &ranges, const std::vector<int> &indexs,
                            std::vector<int> &neighbor_picked);

    void ExtractFeatures(const std::vector<int> &indexs, std::vector<Smooth> &smooth, std::vector<float> &curvature,
                         std::vector<int> &label, std::vector<int> &neighbor_picked);

    bool IsCurrentPoseUnvalid(const PoseInfo &current_pose, const PoseInfo &predicted_pose);

    ros::NodeHandle nh_;

    Map *map_;

    PoseTrackingParams params_;

    PoseInfo last_pose_;

    PoseInfo predict_pose_;

    PoseInfo current_pose_;

    PoseInfo incre_pose_;

    std::vector<bool> valid_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr corner_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr edge_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr valid_cloud_;
};
#endif