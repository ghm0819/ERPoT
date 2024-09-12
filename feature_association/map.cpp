#include "map.h"

Map::Map()
{
    kdtree_corner_from_vertex_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    kdtree_edge_from_center_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    vertex_points_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    center_points_.reset(new pcl::PointCloud<pcl::PointXYZI>);

    vertices_.clear();
    polygons_.clear();
}

void Map::MapGeneration()
{
    std::cout << "Construct the edge information" << std::endl;
    size_t polygon_num = 0U;
    for (auto &polygon : polygons_)
    {
        auto &v_set = polygon->GetVertices();
        size_t size = v_set.size();
        size_t edge_num = 0U;
        for (size_t i = 0U; i < size - 1U; ++i)
        {
            Edge *edge_p = new Edge(v_set[i], v_set[i + 1], edge_num, polygon_num);
            ++edge_num;
            v_set[i]->AddEdgeInfo(edge_p);
            v_set[i + 1]->AddEdgeInfo(edge_p);
            polygon->AddEdgeInfo(edge_p);
            polygon->AddContourInfo(v_set[i]->GetValue());
        }
        Edge *edge_p = new Edge(v_set.back(), v_set.front(), edge_num, polygon_num);
        v_set.back()->AddEdgeInfo(edge_p);
        v_set.front()->AddEdgeInfo(edge_p);
        polygon->AddEdgeInfo(edge_p);
        polygon->AddContourInfo(v_set.back()->GetValue());
        ++polygon_num;
    }

    std::cout << "Construct the KD-tree information" << std::endl;
    for (auto &vertex : vertices_)
    {
        pcl::PointXYZI point;
        point.x = vertex->GetValue().x;
        point.y = vertex->GetValue().y;
        point.z = 0.0;
        point.intensity = vertex->GetIndex();
        vertex_points_->points.emplace_back(point);
    }
    kdtree_corner_from_vertex_->setInputCloud(vertex_points_);

    for (auto &polygon : polygons_)
    {
        pcl::PointXYZI point;
        point.x = polygon->GetCenter().x;
        point.y = polygon->GetCenter().y;
        point.z = 0.0;
        point.intensity = polygon->GetIndex();
        center_points_->points.emplace_back(point);
    }
    kdtree_edge_from_center_->setInputCloud(center_points_);
    // kdtreeSurfFromMap->nearestKSearch
}

void Map::SelectNearestPolygonEdge(const pcl::PointXYZI &point_new, const pcl::PointXYZI &point,
                                   const std::vector<Polygon *> polygons, std::vector<ToVertex> &to_vertices, std::vector<ToEdge> &to_edges)
{
    double min_distance = 1000.0;
    size_t min_index;
    cv::Point2f point_cv(point_new.x, point_new.y);

    for (size_t i = 0U; i < polygons.size(); ++i)
    {
        auto distance = -cv::pointPolygonTest(polygons[i]->GetContour(), point_cv, true);
        if (distance <= 0.0)
        {
            return;
        }
        if (distance < min_distance)
        {
            min_distance = distance;
            min_index = i;
        }
    }

    const auto &edges = polygons[min_index]->GetEdges();
    double min2_distance = 1000.0;
    int min2_index = -1;
    bool is_online = false;
    bool is_left = true;
    for (size_t i = 0U; i < edges.size(); ++i)
    {
        double distance;
        bool is_left_temp = true;
        bool is_online_temp = DistanceToEdge(edges[i]->GetLeftVertex()->GetValue(), edges[i]->GetRightVertex()->GetValue(),
                                             point_cv, is_left_temp, distance);
        if (distance < min2_distance)
        {
            min2_distance = distance;
            min2_index = i;
            is_online = is_online_temp;
            is_left = is_left_temp;
        }
    }

    if (min2_index < 0)
    {
        return;
    }

    if (is_online)
    {
        to_edges.emplace_back(point, edges[min2_index]);
    }
    else if (is_left)
    {
        to_vertices.emplace_back(point, edges[min2_index]->GetLeftVertex());
    }
    else
    {
        to_vertices.emplace_back(point, edges[min2_index]->GetRightVertex());
    }
}

bool Map::DistanceToEdge(const cv::Point2f &point_a, const cv::Point2f &point_b, const cv::Point2f &point,
                         bool &is_left_temp, double &distance)
{
    auto dx = point_b.x - point_a.x;
    auto dy = point_b.y - point_a.y;
    auto t = ((point.x - point_a.x) * dx + (point.y - point_a.y) * dy) / (dx * dx + dy * dy);
    if (t < 0.0f)
    {
        distance = std::sqrt((point.x - point_a.x) * (point.x - point_a.x) +
                             (point.y - point_a.y) * (point.y - point_a.y));
        return false;
    }
    else if (t > 1.0f)
    {
        distance = std::sqrt((point.x - point_b.x) * (point.x - point_b.x) +
                             (point.y - point_b.y) * (point.y - point_b.y));
        is_left_temp = false;
        return false;
    }
    else
    {
        cv::Point2f point_interaction(point_a.x + t * dx, point_a.y + t * dy);
        distance = std::sqrt((point.x - point_interaction.x) * (point.x - point_interaction.x) +
                             (point.y - point_interaction.y) * (point.y - point_interaction.y));
        return true;
    }
}

std::vector<pcl::PointXYZI> &Map::ConvertPointToWorld(const std::vector<pcl::PointXYZI> &points,
                                                      const PoseInfo &predict_pose)
{
    auto sv = std::sin(predict_pose.yaw);
    auto cv = std::cos(predict_pose.yaw);

    size_t size = points.size();
    std::vector<pcl::PointXYZI> points_new(size, pcl::PointXYZI());
    for (size_t i = 0U; i < size; ++i)
    {
        points_new[i].x = cv * points[i].x - sv * points[i].y + predict_pose.x;
        points_new[i].y = sv * points[i].x + cv * points[i].y + predict_pose.y;
        points_new[i].z = 0.0;
        points_new[i].intensity = points[i].intensity;
    }
}

void Map::ObtainAssociationFromMap(const pcl::PointCloud<pcl::PointXYZI> &corner_points,
                                   const pcl::PointCloud<pcl::PointXYZI> &edge_points, const PoseInfo &predict_pose,
                                   std::vector<ToVertex> &to_vertices, std::vector<ToEdge> &to_edges)
{
    to_vertices.clear();
    to_edges.clear();

    auto sv = std::sin(predict_pose.yaw);
    auto cv = std::cos(predict_pose.yaw);

    size_t corner_size = corner_points.points.size();
    size_t edge_size = edge_points.points.size();

    // for the corner point
    for (size_t i = 0U; i < corner_size; ++i)
    {
        const auto &point = corner_points.points[i];
        pcl::PointXYZI point_new;
        point_new.x = cv * point.x - sv * point.y + predict_pose.x;
        point_new.y = sv * point.x + cv * point.y + predict_pose.y;
        point_new.z = 0.0;

        std::vector<int> point_search_ind;
        std::vector<float> point_search_sqdis;

        kdtree_corner_from_vertex_->nearestKSearch(point_new, 1, point_search_ind, point_search_sqdis);
        if (point_search_sqdis[0] > 2.0)
        { // 3.0
            continue;
        }

        const auto &polygons = vertices_[vertex_points_->points[point_search_ind[0]].intensity]->GetPolygons();

        bool is_outside = true;
        for (size_t i = 0U; i < polygons.size(); ++i)
        {
            cv::Point2f point_cv(point_new.x, point_new.y);
            auto distance = cv::pointPolygonTest(polygons[i]->GetContour(), point_cv, false);
            if (distance >= 0)
            {
                is_outside = false;
                break;
            }
        }
        if (is_outside)
        {
            to_vertices.emplace_back(point, vertices_[point_search_ind[0]]);
        }
    }

    // for the edge point
    for (size_t i = 0U; i < edge_size; ++i)
    {
        const auto &point = edge_points.points[i];
        pcl::PointXYZI point_new;
        point_new.x = cv * point.x - sv * point.y + predict_pose.x;
        point_new.y = sv * point.x + cv * point.y + predict_pose.y;
        point_new.z = 0.0;

        std::vector<int> polygon_search_ind;
        std::vector<float> polygon_search_sqdis;

        // search for the nearest 10 polygons
        kdtree_edge_from_center_->nearestKSearch(point_new, 5, polygon_search_ind,
                                                 polygon_search_sqdis);

        std::vector<Polygon *> polygons;
        for (size_t j = 0U; j < polygon_search_ind.size(); ++j)
        {
            if (polygon_search_sqdis[j] > 3.0)
            {
                break;
            }
            polygons.emplace_back(polygons_[center_points_->points[polygon_search_ind[j]].intensity]);
        }
        if (!polygons.empty())
        {
            SelectNearestPolygonEdge(point_new, point, polygons, to_vertices, to_edges);
        }
    }
}