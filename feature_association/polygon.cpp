#include "polygon.h"

Polygon::Polygon(const cv::Point2f &point, const size_t index)
{
    center_point_ = point;
    index_ = index;

    contour_.clear();
    vertices_.clear();
    edges_.clear();
}