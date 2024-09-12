#include "vertex.h"

Vertex::Vertex(const cv::Point2f &point, const size_t index)
{
    vertex_ = point;
    index_ = index;

    polygons_.clear();
    edges_.clear();
}
