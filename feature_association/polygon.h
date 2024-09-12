#ifndef POLYGON_H
#define POLYGON_H

#include <opencv2/opencv.hpp>
#include "vertex.h"
#include "edge.h"

class Edge;
class Vertex;

class Polygon
{
public:
    Polygon() {};

    Polygon(const cv::Point2f &point, const size_t index);

    inline const cv::Point2f &GetCenter()
    {
        return center_point_;
    }

    inline const size_t GetIndex()
    {
        return index_;
    }

    inline const std::vector<Vertex *> &GetVertices()
    {
        return vertices_;
    }

    inline const std::vector<cv::Point2f> &GetContour()
    {
        return contour_;
    }

    inline const std::vector<Edge *> &GetEdges()
    {
        return edges_;
    }

    void AddEdgeInfo(Edge *edge_p)
    {
        edges_.emplace_back(edge_p);
    }

    void AddVertexInfo(Vertex *vertex_p)
    {
        vertices_.emplace_back(vertex_p);
    }

    void AddContourInfo(const cv::Point2f &point)
    {
        contour_.emplace_back(point);
    }

    ~Polygon() = default;

private:
    std::vector<Vertex *> vertices_;

    std::vector<Edge *> edges_;

    std::vector<cv::Point2f> contour_;

    size_t index_;

    cv::Point2f center_point_;
};

#endif