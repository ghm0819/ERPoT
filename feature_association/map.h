#ifndef MAP_H
#define MAP_H
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl/search/impl/search.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>
#include "vertex.h"
#include "edge.h"
#include "polygon.h"

class Vertex;
class Edge;
class Polygon;

struct Smooth
{
    float value;
    size_t index;

    Smooth() : value(0.0), index(0U) {};

    Smooth(float valueIn, size_t indexIn) : value(valueIn), index(indexIn) {};
};

struct PoseInfo
{
    double x;
    double y;
    double yaw;
    double time;
    PoseInfo() : x(0.0), y(0.0), yaw(0.0), time(0.0) {};

    PoseInfo(const double xIn, const double yIn, const double yawIn, const double timeIn) : x(xIn), y(yIn), yaw(yawIn), time(timeIn) {};
};

struct ToVertex
{
    pcl::PointXYZI point;
    Vertex *associated_vertex;

    ToVertex() : point(0.0), associated_vertex(nullptr) {};

    ToVertex(const pcl::PointXYZI &pointIn, Vertex *associated_vertexIn) : point(pointIn), associated_vertex(associated_vertexIn) {};
};

struct ToEdge
{
    pcl::PointXYZI point;
    Edge *associated_edge;

    ToEdge() : point(0.0), associated_edge(nullptr) {};

    ToEdge(const pcl::PointXYZI &pointIn, Edge *associated_edgeIn) : point(pointIn), associated_edge(associated_edgeIn) {};
};

class Map
{
public:
    Map();

    ~Map() = default;

    std::vector<Vertex *> &GetVertices()
    {
        return vertices_;
    }

    std::vector<Polygon *> &GetPolygons()
    {
        return polygons_;
    }

    void SetVertices(std::vector<Vertex *> &vertices)
    {
        vertices_.swap(vertices);
    }

    void SetPolygons(std::vector<Polygon *> &polygons)
    {
        polygons_.swap(polygons);
    }

    // constrction edge and kd-tree
    void MapGeneration();

    void ObtainAssociationFromMap(const pcl::PointCloud<pcl::PointXYZI> &corner_points,
                                  const pcl::PointCloud<pcl::PointXYZI> &edge_points, const PoseInfo &predict_pose,
                                  std::vector<ToVertex> &to_vertices, std::vector<ToEdge> &to_edges);

private:
    void SelectNearestPolygonEdge(const pcl::PointXYZI &point_new, const pcl::PointXYZI &point,
                                  const std::vector<Polygon *> polygons, std::vector<ToVertex> &to_vertices, std::vector<ToEdge> &to_edges);

    bool DistanceToEdge(const cv::Point2f &point_a, const cv::Point2f &point_b, const cv::Point2f &point,
                        bool &is_left_temp, double &distance);

    std::vector<pcl::PointXYZI> &ConvertPointToWorld(const std::vector<pcl::PointXYZI> &points,
                                                     const PoseInfo &predict_pose);

    std::vector<Vertex *> vertices_;

    std::vector<Polygon *> polygons_;

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_corner_from_vertex_;

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_edge_from_center_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr vertex_points_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr center_points_;
};

#endif