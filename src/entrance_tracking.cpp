#include "entrance_tracking.h"

bool EntranceTracking::Initializtion()
{

	map_ = new Map();
	// load map information, including vertices and polygons
	if (!LoadMapFile(map_path_))
	{
		return false;
	}

	map_->MapGeneration();

	auto file_name = map_path_.erase(map_path_.length() - 4U);
	std::ofstream outfile_center(file_name + "_center.txt");
	std::ofstream outfile_polygon(file_name + "_poly.txt");
	if (!outfile_center.is_open() || !outfile_polygon.is_open())
	{
		return false;
	}

	const auto &polygons_ = map_->GetPolygons();
	for (const auto &polygon_ : polygons_)
	{
		const auto &vertices = polygon_->GetVertices();
		for (const auto &point : vertices)
		{
			outfile_polygon << point->GetValue().x << " " << point->GetValue().y << std::endl;
		}
		outfile_polygon << std::endl;
		outfile_center << polygon_->GetCenter().x << " " << polygon_->GetCenter().y << std::endl;
	}
	outfile_center.close();
	outfile_polygon.close();

	odom_.header.frame_id = "odom";
	odom_.child_frame_id = "lidar_link";
	path_.header.frame_id = "odom";

	pub_odom_ = nh_.advertise<nav_msgs::Odometry>("/odometry", 1);
	pub_path_ = nh_.advertise<nav_msgs::Path>("/path", 1);
	pub_edge_ = nh_.advertise<visualization_msgs::MarkerArray>("/map_edge", 1);

	globalmap_pub_timer = nh_.createWallTimer(ros::WallDuration(1.0), &EntranceTracking::PublishMapEdge, this, false, true);

	return true;
}

bool EntranceTracking::LoadMapFile(const std::string &file_name)
{
	std::ifstream in_file(file_name, std::ios::binary);
	if (!in_file.is_open())
	{
		std::cout << "Error Opening! ";
		return false;
	}
	size_t polygon_num = 0U;
	size_t vertex_num = 0U;
	std::vector<Polygon *> polygon_ps;
	std::vector<Vertex *> vertice_ps;
	size_t size;
	while (in_file.read(reinterpret_cast<char *>(&size), sizeof(size_t)))
	{
		std::vector<float> array;
		array.resize(size);
		in_file.read(reinterpret_cast<char *>(array.data()), size * sizeof(float));
		std::vector<std::vector<cv::Point2f>> contours;
		std::vector<cv::Point2f> center_points;
		for (size_t i = 0U; i < size;)
		{
			size_t point_size = static_cast<size_t>(array[i]);
			assert((point_size % 2) == 0);
			std::vector<cv::Point2f> contour;
			for (size_t j = i + 1U; j <= i + point_size; j = j + 2U)
			{
				cv::Point2f point(array[j], array[j + 1U]);
				contour.emplace_back(point);
			}
			cv::Moments m = cv::moments(contour);
			cv::Point2f center_point(m.m10 / m.m00, m.m01 / m.m00);
			contours.emplace_back(contour);
			center_points.emplace_back(center_point);
			i += (point_size + 1U);
		}

		std::vector<Vertex *> temp_vertices;
		for (size_t j = 0U; j < center_points.size(); ++j)
		{
			const auto &contour = contours[j];
			const auto &center_point = center_points[j];
			// construct the polygon
			Polygon *poly_p = new Polygon(center_point, polygon_num);
			polygon_ps.emplace_back(poly_p);
			++polygon_num;
			// construct the vertex
			for (size_t k = 0U; k < contour.size(); ++k)
			{
				size_t index = 0U;
				Vertex *vertex_p;
				if (!JudgeExistVertex(temp_vertices, contour[k], index))
				{
					vertex_p = new Vertex(contour[k], vertex_num);
					temp_vertices.emplace_back(vertex_p);
					vertice_ps.emplace_back(vertex_p);
					++vertex_num;
				}
				else
				{
					vertex_p = temp_vertices[index];
				}
				vertex_p->AddPolygonInfo(poly_p);
				poly_p->AddVertexInfo(vertex_p);
			}
		}
	}
	map_->SetVertices(vertice_ps);
	map_->SetPolygons(polygon_ps);

	return true;
}

void EntranceTracking::PublishMapEdge(const ros::WallTimerEvent &event)
{
	visualization_msgs::MarkerArray line_markers;
	int32_t currentId = 0U;
	const auto &contours = map_->GetPolygons();
	for (const auto &contour : contours)
	{
		visualization_msgs::Marker line_marker;
		line_marker.ns = "contour";
		line_marker.id = currentId++;
		line_marker.type = visualization_msgs::Marker::LINE_LIST;
		line_marker.scale.x = 0.1;
		line_marker.scale.y = 0.1;
		line_marker.color.r = 0.8;
		line_marker.color.g = 0.0;
		line_marker.color.b = 0.8;
		line_marker.color.a = 1.0;
		line_marker.lifetime = ros::Duration(1000);
		const auto &vertices = contour->GetVertices();
		for (size_t i = 0U; i < vertices.size() - 1U; ++i)
		{
			geometry_msgs::Point p_start;
			p_start.x = vertices[i]->GetValue().x;
			p_start.y = vertices[i]->GetValue().y;
			p_start.z = 0.0;
			line_marker.points.emplace_back(p_start);
			geometry_msgs::Point p_end;
			p_end.x = vertices[i + 1U]->GetValue().x;
			p_end.y = vertices[i + 1U]->GetValue().y;
			p_end.z = 0.0;
			line_marker.points.emplace_back(p_end);
		}
		geometry_msgs::Point p_start;
		p_start.x = vertices.back()->GetValue().x;
		p_start.y = vertices.back()->GetValue().y;
		p_start.z = 0.0;
		line_marker.points.emplace_back(p_start);
		geometry_msgs::Point p_end;
		p_end.x = vertices.front()->GetValue().x;
		p_end.y = vertices.front()->GetValue().y;
		p_end.z = 0.0;
		line_marker.points.emplace_back(p_end);

		line_marker.header.frame_id = "map";
		line_marker.header.stamp = ros::Time::now();
		line_markers.markers.emplace_back(line_marker);
	}
	pub_edge_.publish(line_markers);
}

bool EntranceTracking::JudgeExistVertex(const std::vector<Vertex *> &vertices, const cv::Point2f &point,
										size_t &index)
{
	if (vertices.empty())
	{
		return false;
	}
	for (size_t i = 0U; i < vertices.size(); ++i)
	{
		if (std::abs(vertices[i]->GetValue().x - point.x) < 1e-2 &&
			std::abs(vertices[i]->GetValue().y - point.y) < 1e-2)
		{
			index = i;
			return true;
		}
	}
	return false;
}

bool EntranceTracking::LoadParameterForPoseTracking()
{
	nh_.param("pose_tracking/start_x", params_.start_x, 192.829000);  // 1:-43.829644 2:-27.888224 4: 192.829000 6: -5.575818 -7.021920
	nh_.param("pose_tracking/start_y", params_.start_y, -62.716500);  // 1:-10.606556  2:-8.387609  4: -62.716500 6: -3.215965 -2.831730
	nh_.param("pose_tracking/start_yaw", params_.start_yaw, -2.7723); // 1:-0.4581    2:-2.8758    4: -2.7723 6: 0.26173686 0.23442899
	nh_.param("pose_tracking/tracking_distance_threshold", params_.tracking_distance_threshold, 1.0);
	nh_.param("pose_tracking/tracking_angle_threshold", params_.tracking_angle_threshold, 0.2);
	nh_.param("pose_tracking/distance_association_vertex", params_.distance_association_vertex, 1.0);
	nh_.param("pose_tracking/distance_association_polygon", params_.distance_association_polygon, 1.0);
	nh_.param("pose_tracking/factor_point", params_.factor_vertex, 1.0);
	nh_.param("pose_tracking/factor_line", params_.factor_edge, 1.0);
	nh_.param("pose_tracking/corner_threshold", params_.corner_threshold, 10.0);
	nh_.param("pose_tracking/edge_threshold", params_.edge_threshold, 0.1);
	nh_.param("pose_tracking/corner_feature_num", params_.corner_feature_num, 10);
	nh_.param("pose_tracking/edge_feature_num", params_.edge_feature_num, 50);
	nh_.param("pose_tracking/feature_selection", params_.edge_feature_selection, false);
	nh_.param("pose_tracking/constant_mode", params_.constant_mode, true);

	nh_.param<std::string>("pose_tracking/outfile_pose", params_.outfile_pose_, "~/Download/pose.txt");
	nh_.param<std::string>("pose_tracking/outfile_time", params_.outfile_time_, "~/Download/time.txt");
	return true;
}

void EntranceTracking::Start()
{
	if (!LoadParameterForPoseTracking())
	{
		ROS_ERROR("param load error");
		return;
	}

	feature_association_ = std::make_shared<FeatureAssociation>(map_, params_);
	start_flag_ = true;
	main_processer_ = std::thread(&EntranceTracking::MainProcess, this);
}

void EntranceTracking::Stop()
{
	start_flag_ = false;
	main_processer_.join();
}

void EntranceTracking::ScanCallback(const sensor_msgs::PointCloud2ConstPtr &scan_msg)
{
	std::lock_guard<std::mutex> lock(scan_lock_);
	scan_msgs_.emplace_back(*scan_msg);
	if (scan_msgs_.size() > 7U)
	{ // size 7U
		scan_msgs_.pop_front();
	}
}

void EntranceTracking::MainProcess()
{
	while (start_flag_)
	{
		if (scan_msgs_.empty())
		{
			usleep(50000);
			continue;
		}
		pcl::PointCloud<pcl::PointXYZI> pc_curr;
		{
			std::lock_guard<std::mutex> lock(scan_lock_);
			pcl::fromROSMsg(scan_msgs_.front(), pc_curr);
			odom_.header.stamp = scan_msgs_.front().header.stamp;
			scan_msgs_.pop_front();
		}
		PoseInfo pose = feature_association_->PoseTracking(pc_curr);
		tf::Quaternion q;
		q.setRPY(0.0, 0.0, pose.yaw);
		odom_.pose.pose.orientation.x = q.x();
		odom_.pose.pose.orientation.y = q.y();
		odom_.pose.pose.orientation.z = q.z();
		odom_.pose.pose.orientation.w = q.w();
		odom_.pose.pose.position.x = pose.x;
		odom_.pose.pose.position.y = pose.y;
		odom_.pose.pose.position.z = 0.0;
		pub_odom_.publish(odom_);

		tf::Transform ident;
		ident.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
		ident.setRotation(tf::Quaternion(0, 0, 0, 1));
		tf::StampedTransform base_link_2_laser = tf::StampedTransform(ident, odom_.header.stamp, "base_link", "lidar_link");
		tf_broadcaster_.sendTransform(base_link_2_laser);

		tf::StampedTransform link_2_laser = tf::StampedTransform(ident, odom_.header.stamp, "lidar_link", "laser");
		tf_broadcaster_.sendTransform(link_2_laser);

		tf::StampedTransform map_2_odom = tf::StampedTransform(ident, odom_.header.stamp, "map", "odom");
		tf_broadcaster_.sendTransform(map_2_odom);

		tf::Transform t_cur;
		tf::poseMsgToTF(odom_.pose.pose, t_cur);
		tf::StampedTransform odom_2_base_link = tf::StampedTransform(t_cur, odom_.header.stamp, "odom",
																	 "base_link");
		tf_broadcaster_.sendTransform(odom_2_base_link);

		geometry_msgs::PoseStamped pose_info;
		pose_info.header = odom_.header;
		pose_info.pose = odom_.pose.pose;
		pose_info.header.stamp = odom_.header.stamp;
		path_.poses.emplace_back(pose_info);
		pub_path_.publish(path_);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tracking");
	ros::NodeHandle nh("~");

	std::string scan_topic;
	std::string map_path;
	nh.param<std::string>("scan_topic", scan_topic, "/ground_segmentation/scan");
	nh.param<std::string>("map_path", map_path, "/home/ghm/map/self_dataset/map.bin");
	EntranceTracking enter_node(map_path, nh);

	enter_node.Start();

	ros::Subscriber point_cloud_sub_ = nh.subscribe(scan_topic, 1,
													&EntranceTracking::ScanCallback, &enter_node);
	ros::spin();
	enter_node.Stop();
}