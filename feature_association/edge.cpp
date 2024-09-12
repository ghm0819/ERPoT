#include "edge.h"

Edge::Edge(Vertex *vex_one, Vertex *vex_two, const size_t index, const size_t polygon_index)
{
    edge_left_ = vex_one;
    edge_right_ = vex_two;
    index_ = index;
    polygon_index_ = polygon_index;
}