#ifndef EDGE_H
#define EDGE_H

#include <opencv2/opencv.hpp>
#include "vertex.h"

class Vertex;
class Polygon;

class Edge
{
public:
    Edge() {};

    Edge(Vertex *vex_one, Vertex *vex_two, const size_t index, const size_t polygon_index);

    ~Edge() = default;

    inline Vertex *GetLeftVertex() const
    {
        return edge_left_;
    }

    inline Vertex *GetRightVertex() const
    {
        return edge_right_;
    }

    inline size_t GetIndex() const
    {
        return index_;
    }

    void SetPolygonInfo(Polygon *polygon)
    {
        polygon_ = polygon;
    }

private:
    Vertex *edge_left_;
    Vertex *edge_right_;

    // index_ the index in the special polygon
    size_t index_;
    size_t polygon_index_;
    //
    Polygon *polygon_;
};

#endif