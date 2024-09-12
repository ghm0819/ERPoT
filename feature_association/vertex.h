#ifndef VERTEX_H
#define VERTEX_H

#include <opencv2/opencv.hpp>
#include "edge.h"
#include "polygon.h"

class Edge;
class Polygon;

class Vertex
{
public:
    Vertex() {};

    Vertex(const cv::Point2f &point, const size_t index);

    ~Vertex() = default;

    inline const cv::Point2f &GetValue()
    {
        return vertex_;
    }

    inline const size_t GetIndex()
    {
        return index_;
    }

    inline const std::vector<Polygon *> &GetPolygons()
    {
        return polygons_;
    }

    void AddPolygonInfo(Polygon *polygon_p)
    {
        polygons_.emplace_back(polygon_p);
    }

    void AddEdgeInfo(Edge *edge_p)
    {
        edges_.emplace_back(edge_p);
    }

private:
    cv::Point2f vertex_;

    size_t index_;

    std::vector<Edge *> edges_;

    std::vector<Polygon *> polygons_;
};

#endif