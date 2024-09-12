#include "feature_association.h"

FeatureAssociation::FeatureAssociation(Map *map, const PoseTrackingParams &params)
{
    map_ = map;
    params_ = params;

    last_pose_.x = params_.start_x;
    last_pose_.y = params_.start_y;
    last_pose_.yaw = params_.start_yaw;

    // only for kitti-06
    // incre_pose_.x = 1.3658903;
    // incre_pose_.y = 0.0114542;

    corner_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    edge_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    valid_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
}

void FeatureAssociation::PredictCurrentPose()
{
    // last pose + incre pose
    if (params_.constant_mode)
    {
        auto cv = std::cos(last_pose_.yaw);
        auto sv = std::sin(last_pose_.yaw);
        predict_pose_.x = last_pose_.x + incre_pose_.x * cv - incre_pose_.y * sv;
        predict_pose_.y = last_pose_.y + incre_pose_.x * sv + incre_pose_.y * cv;
        predict_pose_.yaw = last_pose_.yaw + incre_pose_.yaw;
        if (predict_pose_.yaw > M_PI)
        {
            predict_pose_.yaw -= 2 * M_PI;
        }
        else if (predict_pose_.yaw <= -M_PI)
        {
            predict_pose_.yaw += 2 * M_PI;
        }
        else
        {
            // do nothing
        }
    }
    else
    {
        predict_pose_.x = last_pose_.x;
        predict_pose_.y = last_pose_.y;
        predict_pose_.yaw = last_pose_.yaw;
    }
}

void FeatureAssociation::UpdateIncrePose()
{
    auto cv = std::cos(last_pose_.yaw);
    auto sv = std::sin(last_pose_.yaw);
    incre_pose_.x = current_pose_.x * cv + current_pose_.y * sv - last_pose_.x * cv - last_pose_.y * sv;
    incre_pose_.y = -current_pose_.x * sv + current_pose_.y * cv + last_pose_.x * sv - last_pose_.y * cv;

    incre_pose_.yaw = current_pose_.yaw - last_pose_.yaw;
    if (incre_pose_.yaw <= -M_PI)
    {
        incre_pose_.yaw += 2 * M_PI;
    }
    else if (incre_pose_.yaw > M_PI)
    {
        incre_pose_.yaw -= 2 * M_PI;
    }
    else
    {
        // do nothing
    }
}

void FeatureAssociation::MotionCompensation(pcl::PointXYZI &point_in)
{
    auto ratio_x = incre_pose_.x * point_in.intensity;
    auto ratio_y = incre_pose_.y * point_in.intensity;
    auto ratio_yaw = incre_pose_.yaw * point_in.intensity;
    auto cv = std::cos(ratio_yaw);
    auto sv = std::sin(ratio_yaw);
    auto x = ratio_x + point_in.x * cv - point_in.y * sv;
    auto y = ratio_y + point_in.x * sv + point_in.y * cv;
    point_in.x = x;
    point_in.y = y;
}

/**
 * feature selection
 * corner feature and edge feature, based on the process of LOAM
 */
void FeatureAssociation::DataInitialization(const pcl::PointCloud<pcl::PointXYZI> &obstacle_scan, std::vector<int> &indexs,
                                            std::vector<float> &valid_ranges)
{
    size_t size = obstacle_scan.points.size();
    valid_cloud_->clear();
    indexs.clear();
    valid_ranges.clear();

    for (size_t i = 0U; i < size; ++i)
    {
        if (obstacle_scan.points[i].intensity <= 1.0)
        {
            valid_cloud_->points.emplace_back(obstacle_scan.points[i]);
            indexs.emplace_back(i);
            valid_ranges.emplace_back(std::sqrt(obstacle_scan.points[i].x * obstacle_scan.points[i].x +
                                                obstacle_scan.points[i].y * obstacle_scan.points[i].y));
        }
    }
}

void FeatureAssociation::CalculateSmoothness(const std::vector<float> &ranges, std::vector<float> &curvature,
                                             std::vector<int> &neighbor_picked, std::vector<Smooth> &smooth, std::vector<int> &label)
{
    size_t point_size = ranges.size(); // valid point number
    curvature.resize(point_size);
    neighbor_picked.resize(point_size);
    smooth.resize(point_size);
    label.resize(point_size);

    for (size_t i = 5; i < point_size - 5; ++i)
    {
        float diff_range = ranges[i - 5] + ranges[i - 4] + ranges[i - 3] + ranges[i - 2] + ranges[i - 1] - ranges[i] * 10 + ranges[i + 1] + ranges[i + 2] + ranges[i + 3] + ranges[i + 4] + ranges[i + 5];

        curvature[i] = diff_range * diff_range;
        neighbor_picked[i] = 0;
        label[i] = 0;
        smooth[i].value = curvature[i];
        smooth[i].index = i;
    }
}

void FeatureAssociation::MarkOccludedPoints(const std::vector<float> &ranges, const std::vector<int> &indexs,
                                            std::vector<int> &neighbor_picked)
{
    size_t point_size = ranges.size();

    for (size_t i = 5; i < point_size - 6; ++i)
    {

        float depth1 = ranges[i];
        float depth2 = ranges[i + 1];
        int column_diff = std::abs(int(indexs[i + 1] - indexs[i]));

        if (column_diff < 10)
        {

            if (depth1 - depth2 > 0.3)
            {
                neighbor_picked[i - 5] = 1;
                neighbor_picked[i - 4] = 1;
                neighbor_picked[i - 3] = 1;
                neighbor_picked[i - 2] = 1;
                neighbor_picked[i - 1] = 1;
                neighbor_picked[i] = 1;
            }
            else if (depth2 - depth1 > 0.3)
            {
                neighbor_picked[i + 1] = 1;
                neighbor_picked[i + 2] = 1;
                neighbor_picked[i + 3] = 1;
                neighbor_picked[i + 4] = 1;
                neighbor_picked[i + 5] = 1;
                neighbor_picked[i + 6] = 1;
            }
        }

        float diff1 = std::abs(float(ranges[i - 1] - ranges[i]));
        float diff2 = std::abs(float(ranges[i + 1] - ranges[i]));

        if (diff1 > 0.02 * ranges[i] && diff2 > 0.02 * ranges[i])
        {
            neighbor_picked[i] = 1;
        }
    }
}

void FeatureAssociation::ExtractFeatures(const std::vector<int> &indexs, std::vector<Smooth> &smooth,
                                         std::vector<float> &curvature, std::vector<int> &label, std::vector<int> &neighbor_picked)
{
    corner_cloud_->clear();
    edge_cloud_->clear();
    int start_index = 5;
    int end_index = indexs.size() - 6;
    for (int j = 0; j < 6; ++j)
    {
        int sp = (start_index * (6 - j) + end_index * j) / 6;
        int ep = (start_index * (5 - j) + end_index * (j + 1)) / 6 - 1;

        if (sp >= ep)
        {
            continue;
        }

        std::sort(smooth.begin() + sp, smooth.begin() + ep, [](const Smooth &a, const Smooth &b)
                  { return a.value < b.value; });
        // std::cout << j << ": " << sp << " " << ep << " " << valid_cloud_->points.size() << std::endl;
        int largest_picked_num = 0;
        for (int k = ep; k >= sp; --k)
        {
            auto ind = smooth[k].index;
            if (neighbor_picked[ind] == 0 && curvature[ind] > params_.corner_threshold)
            {
                ++largest_picked_num;
                if (largest_picked_num <= 20)
                {
                    label[ind] = 1;
                    corner_cloud_->push_back(valid_cloud_->points[ind]);
                }
                else
                {
                    break;
                }

                neighbor_picked[ind] = 1;
                for (int l = 1; l <= 5; ++l)
                {
                    int column_diff = std::abs(int(indexs[ind + l] - indexs[ind + l - 1]));
                    if (column_diff > 10)
                    {
                        break;
                    }
                    neighbor_picked[ind + l] = 1;
                }
                for (int l = -1; l >= -5; --l)
                {
                    int column_diff = std::abs(int(indexs[ind + l] - indexs[ind + l + 1]));
                    if (column_diff > 10)
                    {
                        break;
                    }
                    neighbor_picked[ind + l] = 1;
                }
            }
        }

        for (int k = sp; k <= ep; ++k)
        {
            int ind = smooth[k].index;
            if (neighbor_picked[ind] == 0 && curvature[ind] < params_.edge_threshold)
            {
                label[ind] = -1;
                neighbor_picked[ind] = 1;

                for (int l = 1; l <= 2; ++l)
                {
                    int column_diff = std::abs(int(indexs[ind + l] - indexs[ind + l - 1]));
                    if (column_diff > 10)
                    {
                        break;
                    }

                    neighbor_picked[ind + l] = 1;
                }
                for (int l = -1; l >= -2; --l)
                {
                    int column_diff = std::abs(int(indexs[ind + l] - indexs[ind + l + 1]));
                    if (column_diff > 10)
                    {
                        break;
                    }

                    neighbor_picked[ind + l] = 1;
                }
            }
        }

        for (int k = sp; k <= ep; ++k)
        {
            if (label[k] <= 0)
            {
                edge_cloud_->push_back(valid_cloud_->points[k]);
            }
        }
    }
}

bool FeatureAssociation::IsCurrentPoseUnvalid(const PoseInfo &current_pose, const PoseInfo &predicted_pose)
{
    auto angle_diff = current_pose.yaw - predicted_pose.yaw;
    if (angle_diff <= -M_PI)
    {
        angle_diff += 2 * M_PI;
    }
    else if (angle_diff > M_PI)
    {
        angle_diff -= 2 * M_PI;
    }
    else
    {
        // do nothingproblem_options
    }

    return ((std::abs(current_pose.x - predicted_pose.x) > 1.0) || // 0.8
            (std::abs(current_pose.y - predicted_pose.y) > 1.0) || // 0.8
            (std::abs(angle_diff) > 0.2));
}

PoseInfo FeatureAssociation::PoseTracking(const pcl::PointCloud<pcl::PointXYZI> &obstacle_scan)
{

    std::ofstream outfile_pose(params_.outfile_pose_, std::ios::app);
    std::ofstream outfile_time(params_.outfile_time_, std::ios::app);

    TicToc time;

    uint64_t curr_time = obstacle_scan.header.stamp;
    std::vector<int> indexs;
    std::vector<float> ranges;
    std::vector<float> curvature;
    std::vector<int> neighbor_picked;
    std::vector<Smooth> smooth;
    std::vector<int> label;
    // step-1 obtain the predicted pose from last pose:
    PredictCurrentPose();

    current_pose_ = predict_pose_; // the initial pose is the predicted pose

    // step-2 complete feature extraction
    DataInitialization(obstacle_scan, indexs, ranges);

    CalculateSmoothness(ranges, curvature, neighbor_picked, smooth, label);

    MarkOccludedPoints(ranges, indexs, neighbor_picked);

    ExtractFeatures(indexs, smooth, curvature, label, neighbor_picked);

    // calculate the current poses
    for (size_t opti_counter = 0; opti_counter < 20; ++opti_counter)
    {
        ceres::LossFunction *loss_function = new ceres::CauchyLoss(0.1);
        ceres::LocalParameterization *angle_local_parameterization = AngleLocalParameterization::Create();
        ceres::Problem::Options problem_options;

        ceres::Problem problem(problem_options);
        problem.AddParameterBlock(&current_pose_.x, 1);
        problem.AddParameterBlock(&current_pose_.y, 1);
        problem.AddParameterBlock(&current_pose_.yaw, 1, angle_local_parameterization);
        // step-3 construct the factor graph through data-association

        std::vector<ToVertex> to_vertices;
        std::vector<ToEdge> to_edges;
        map_->ObtainAssociationFromMap(*corner_cloud_, *edge_cloud_, current_pose_, to_vertices, to_edges);

        // step-4 ceres-solver or self-calculated to obtain the pose information
        for (const auto &to_vertex : to_vertices)
        {
            Eigen::Vector2d curr_point(to_vertex.point.x, to_vertex.point.y);
            Eigen::Vector2d dest_point(to_vertex.associated_vertex->GetValue().x, to_vertex.associated_vertex->GetValue().y);

            ceres::CostFunction *cost_function = LidarPointFactor::Create(curr_point, dest_point, params_.factor_vertex);
            problem.AddResidualBlock(cost_function, loss_function, &current_pose_.x, &current_pose_.y, &current_pose_.yaw);
        }

        for (const auto &to_edge : to_edges)
        {
            Eigen::Vector2d curr_point(to_edge.point.x, to_edge.point.y);
            Eigen::Vector2d dest_point_a(to_edge.associated_edge->GetLeftVertex()->GetValue().x,
                                         to_edge.associated_edge->GetLeftVertex()->GetValue().y);
            Eigen::Vector2d dest_point_b(to_edge.associated_edge->GetRightVertex()->GetValue().x,
                                         to_edge.associated_edge->GetRightVertex()->GetValue().y);
            ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, dest_point_a, dest_point_b, params_.factor_edge);
            problem.AddResidualBlock(cost_function, loss_function, &current_pose_.x, &current_pose_.y, &current_pose_.yaw);
        }
        // step-5 from the results of current and last step, calculate the pose increment
        TicToc t_solver;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 5; // need to refine
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        // printf("solver time %f ms \n", t_solver.toc());
    }

    // if the calculated current pose is very different the predicted pose
    if (IsCurrentPoseUnvalid(current_pose_, predict_pose_))
    {
        current_pose_ = predict_pose_;
    }

    // std::cout << "pose: " << current_pose_.x << " " << current_pose_.y << " " << current_pose_.yaw << std::endl;
    outfile_time << curr_time << " " << time.toc() << std::endl;
    outfile_time.close();
    outfile_pose << curr_time << " " << current_pose_.x << " " << current_pose_.y << " " << current_pose_.yaw << std::endl;
    outfile_pose.close();

    // update the pose information
    UpdateIncrePose();
    last_pose_ = current_pose_;
    return current_pose_;
}
