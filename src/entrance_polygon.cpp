//
// Created by ghm on 2024/03/03.
//

#include "entrance_polygon.h"

bool EntrancePolygon::LoadConfigFile(const std::string &file_name)
{
	YAML::Node config = YAML::LoadFile(file_name);
	if (!config)
	{
		std::cout << "Open config File:" << file_name << " failed.";
		return false;
	}
	if (!config["resolution"])
	{
		std::cout << "Open config File:" << file_name << " has no resolution.";
		return false;
	}
	else
	{
		map_params_.resolution_ = config["resolution"].as<float>();
	}

	if (!config["origin"])
	{
		std::cout << "Open config File:" << file_name << " has no origin.";
		return false;
	}
	else
	{
		std::vector<float> origin_info;
		for (YAML::const_iterator it = config["origin"].begin(); it != config["origin"].end(); ++it)
		{
			origin_info.emplace_back(it->as<float>());
		}
		map_params_.origin_x_ = origin_info[0];
		map_params_.origin_y_ = origin_info[1];
		map_params_.origin_z_ = origin_info[2];
	}
	return true;
}

bool EntrancePolygon::Initializtion()
{
	std::string yaml_path = file_path_ + ".yaml";
	std::string map_path = file_path_ + ".pgm";
	if (!LoadConfigFile(yaml_path))
	{
		return false;
	}
	if (!std::filesystem::exists(map_path))
	{
		std::cout << "Open config File:" << map_path << " failed.";
		return false;
	}
	cv::Mat grid_map = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
	size_t rows = grid_map.rows;
	size_t cols = grid_map.cols;
	if (rows > 0 && cols > 0)
	{
		grid_map_ = cv::Mat::zeros(rows, cols, CV_8UC1);
	}
	else
	{
		ROS_ERROR("Invaild grid map?");
		return false;
	}

	for (size_t i = 0U; i < rows; ++i)
	{
		for (size_t j = 0U; j < cols; ++j)
		{
			const uint8_t &value = grid_map.data[i * cols + j];
			if (value < 200U)
			{
				int num_free = 0;
				int num_unknown = 0;
				CalculateNeighbourPixels(grid_map, i, j, num_free, num_unknown);
				if (num_unknown < 40 && (80 - num_free - num_unknown) >= 2)
				{ //>10 for low-speed //  >= 5 or 2 for high-speed
					grid_map_.data[i * cols + j] = 255U;
				}
			}
		}
	}
	cv::imwrite(file_path_ + ".png", grid_map_);
	return true;
}

void EntrancePolygon::CalculateNeighbourPixels(const cv::Mat &grid_map, const size_t i, const size_t j, int &num_free,
											   int &num_unknown)
{
	size_t border_i_min = 0U;
	size_t border_i_max = grid_map.rows - 1;
	size_t border_j_min = 0U;
	size_t border_j_max = grid_map.cols - 1;

	if (i >= 4U)
	{
		border_i_min = i - 4U;
	}
	if ((i + 4U) <= border_i_max)
	{
		border_i_max = i + 4U;
	}
	if (j >= 4U)
	{
		border_j_min = j - 4U;
	}
	if ((j + 4U) <= border_j_max)
	{
		border_j_max = j + 4U;
	}
	for (size_t ii = border_i_min; ii <= border_i_max; ++ii)
	{
		for (size_t jj = border_j_min; jj <= border_j_max; ++jj)
		{
			if (grid_map.data[ii * grid_map.cols + jj] > 220U)
			{
				++num_free;
			}
			else if (grid_map.data[ii * grid_map.cols + jj] > 100U)
			{
				++num_unknown;
			}
			else
			{
				// do nothing
			}
		}
	}
}

void EntrancePolygon::MapBinFileWrite(const std::vector<std::vector<ConvexHullf>> &contours)
{
	std::ofstream out_file(file_path_ + ".bin", std::ios::binary | std::ios::app);
	if (!out_file)
	{
		std::cerr << "Error opening file for writing." << std::endl;
		return;
	}

	for (const auto &contour_s : contours)
	{
		size_t size = 0U;
		std::vector<float> values;
		for (const auto &contour : contour_s)
		{
			double area = cv::contourArea(contour);
			if (area <= 0.0025 * 4)
			{ // 0.05 * 0.05 * number: 0.0025 * 16 for low-speed /// 0.0025 * 4 for high-speed
				continue;
			}
			values.emplace_back(contour.size() * 2);
			for (const auto &point : contour)
			{
				values.emplace_back(point.x);
				values.emplace_back(point.y);
			}
		}
		if (!values.empty())
		{
			size = values.size();
			out_file.write(reinterpret_cast<char *>(&size), sizeof(size_t));
			out_file.write(reinterpret_cast<char *>(values.data()), size * sizeof(float));
		}
	}
	out_file.close();
}

bool EntrancePolygon::Start()
{
	// for segmentation
	std::vector<std::vector<ConvexHullf>> contours;
	ProcessOccupancyMap(contours);
	// write polygon information to the polygon map
	std::cout << "polygon num: " << contours.size() << std::endl;
	MapBinFileWrite(contours);
	return true;
}

void EntrancePolygon::ProcessOccupancyMap(std::vector<std::vector<ConvexHullf>> &find_contours)
{
	cv::Mat out;
	cv::Mat element_one = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	cv::dilate(grid_map_, out, element_one);
	cv::erode(out, out, element_one);

	cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
	cv::dilate(out, out, element);

	std::vector<cv::Vec4i> hierarchy;
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(out, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	std::vector<ConvexHullf> contours_temp;

	for (const auto &convex_single : contours)
	{
		ConvexHullf points_temp;
		for (const auto &point : convex_single)
		{
			cv::Point2f pt;
			pt.x = (point.x) * map_params_.resolution_ + map_params_.origin_x_;
			pt.y = (grid_map_.rows - point.y) * map_params_.resolution_ + map_params_.origin_y_;
			points_temp.emplace_back(pt);
		}
		contours_temp.emplace_back(points_temp);
	}
	ProcessOverlapContour(contours_temp);

	ProcessHugeContour(contours_temp, find_contours);

	std::ofstream outfile_polygon(file_path_ + "_polygon.txt");
	if (!outfile_polygon.is_open())
	{
		return;
	}
	for (const auto &contours : find_contours)
	{
		for (const auto &contour : contours)
		{
			for (const auto &point : contour)
			{
				outfile_polygon << point.x << " " << point.y << std::endl;
			}
			outfile_polygon << std::endl;
		}
	}
	outfile_polygon.close();
}

void EntrancePolygon::ProcessOverlapContour(std::vector<ConvexHullf> &contours)
{
	std::vector<ConvexHullf> contours_temps;
	contours_temps.swap(contours);
	for (auto &contour : contours_temps)
	{
		if (contour.size() < 10U)
		{
			contours.emplace_back(contour);
			continue;
		}

		std::queue<ConvexHullf> open_contours;
		open_contours.emplace(contour);
		while (!open_contours.empty())
		{
			auto current_contour = open_contours.front();
			open_contours.pop();

			ConvexHullf left_contour;
			ConvexHullf right_contour;

			auto size = current_contour.size();
			bool flag = false;
			for (size_t i = 0U; i < (size - 7); ++i)
			{
				for (size_t j = i + 6U; j < size; ++j)
				{
					if (std::abs(current_contour[i].x - current_contour[j].x) < 1e-2 &&
						std::abs(current_contour[i].y - current_contour[j].y) < 1e-2)
					{
						left_contour.insert(left_contour.end(), current_contour.begin(), current_contour.begin() + i);
						left_contour.insert(left_contour.end(), current_contour.begin() + j, current_contour.end());
						right_contour.insert(right_contour.end(), current_contour.begin() + i + 1, current_contour.begin() + j - 1U);
						flag = true;
						break;
					}
				}
				if (flag)
				{
					contours.emplace_back(left_contour);
					if (right_contour.size() < 10U)
					{
						contours.emplace_back(right_contour);
					}
					else
					{
						open_contours.emplace(right_contour);
					}
					break;
				}
			}
			if (!flag)
			{
				contours.emplace_back(current_contour);
			}
		}
	}
}

void EntrancePolygon::ProcessHugeContour(const std::vector<ConvexHullf> &contours,
										 std::vector<std::vector<ConvexHullf>> &find_contours)
{
	find_contours.clear();
	constexpr int polygon_size = 20;
	for (auto &contour : contours)
	{
		std::vector<ConvexHullf> find_contour;
		if (contour.size() < polygon_size)
		{
			find_contour.emplace_back(contour);
			find_contours.emplace_back(find_contour);
			continue;
		}

		std::queue<ConvexHullf> open_contours;
		open_contours.emplace(contour);

		while (!open_contours.empty())
		{
			auto current_contour = open_contours.front();
			open_contours.pop();

			ConvexHullf left_contour;
			ConvexHullf right_contour;
			auto size = current_contour.size();
			auto gap = std::floor(size / 2);
			double distance_min = 1000000.0;
			size_t index_min = 0U;
			bool success = false;
			for (; gap >= 2; gap = std::floor(gap / 2))
			{
				for (size_t i = 0U; (i + gap) < size; ++i)
				{
					auto j = i + gap;
					size_t pre_back = i - 1U;
					size_t pre_forward = i + 1U;
					if (i == 0U)
					{
						pre_back = size - 1U;
					}

					size_t lat_back = j - 1U;
					size_t lat_forward = j + 1U;
					if (j == (size - 1U))
					{
						lat_forward = 0U;
					}

					if (!JudgeSegmentValid(current_contour[pre_back], current_contour[pre_forward],
										   current_contour[i], current_contour[j]) ||
						!JudgeSegmentValid(current_contour[lat_back],
										   current_contour[lat_forward], current_contour[j], current_contour[i]))
					{
						continue;
					}

					if (JudgeInteraction(current_contour, i, j))
					{
						continue;
					}

					double distance = (current_contour[i].x - current_contour[j].x) * (current_contour[i].x - current_contour[j].x) +
									  (current_contour[i].y - current_contour[j].y) * (current_contour[i].y - current_contour[j].y);
					if (distance < distance_min)
					{
						distance_min = distance;
						index_min = i;
					}
				}
				if (distance_min < 100.0)
				{
					left_contour.insert(left_contour.end(), current_contour.begin(), current_contour.begin() + index_min + 1U);
					left_contour.insert(left_contour.end(), current_contour.begin() + index_min + gap, current_contour.end());
					right_contour.insert(right_contour.end(), current_contour.begin() + index_min,
										 current_contour.begin() + index_min + gap + 1U);
					if (left_contour.size() < polygon_size)
					{
						find_contour.emplace_back(left_contour);
					}
					else
					{
						open_contours.emplace(left_contour);
					}

					if (right_contour.size() < polygon_size)
					{
						find_contour.emplace_back(right_contour);
					}
					else
					{
						open_contours.emplace(right_contour);
					}
					success = true;
					break;
				}
			}
			if (!success)
			{
				find_contour.emplace_back(current_contour);
			}
		}
		find_contours.emplace_back(find_contour);
	}
}

bool EntrancePolygon::JudgeSegmentValid(const cv::Point2f &p1, const cv::Point2f &p2, const cv::Point2f &p_source,
										const cv::Point2f &p_target)
{
	auto theta_min = std::atan2(p2.y - p_source.y, p2.x - p_source.x);
	auto theta_max = std::atan2(p1.y - p_source.y, p1.x - p_source.x);
	auto theta_target = std::atan2(p_target.y - p_source.y, p_target.x - p_source.x);
	if (theta_min > theta_max)
	{
		return (theta_target < theta_min || theta_target > theta_max);
	}
	else
	{
		return (theta_target < theta_max && theta_target > theta_min);
	}
}

bool EntrancePolygon::JudgeInteraction(const ConvexHullf &points, const size_t source, const size_t target)
{
	for (size_t i = 0U; i < points.size() - 1U; ++i)
	{
		if (i == source || i == target || (i + 1 == source) || (i + 1 == target))
		{
			continue;
		}
		if (LineInteraction(points[source], points[target], points[i], points[i + 1]))
		{
			return true;
		}
	}
	return false;
}

bool EntrancePolygon::LineInteraction(const cv::Point2f &l1p1, const cv::Point2f &l1p2, const cv::Point2f &l2p1, cv::Point2f l2p2)
{
	auto det = [](float a, float b, float c, float d)
	{
		return a * d - b * c;
	};
	float d_value = det(l1p1.x - l1p2.x, l2p2.x - l2p1.x, l1p1.y - l1p2.y, l2p2.y - l2p1.y);
	float p_value = det(l2p2.x - l1p2.x, l2p2.x - l2p1.x, l2p2.y - l1p2.y, l2p2.y - l2p1.y);
	float q_value = det(l1p1.x - l1p2.x, l2p2.x - l1p2.x, l1p1.y - l1p2.y, l2p2.y - l1p2.y);

	// 判断是否有交点
	if (d_value != 0)
	{
		float lam = p_value / d_value;
		float eta = q_value / d_value;
		if (0 <= lam && lam <= 1 && 0 <= eta && eta <= 1)
		{
			return true;
		}
		return false;
	}
	if (p_value != 0 || q_value != 0)
	{
		return false;
	}
	auto t1 = std::minmax(std::make_pair(l1p1.x, l1p1.y), std::make_pair(l1p2.x, l1p2.y));
	auto t2 = std::minmax(std::make_pair(l2p1.x, l2p1.y), std::make_pair(l2p2.x, l2p2.y));
	if (t1.second < t2.first || t2.second < t1.first)
	{
		return false;
	}
	return true;
}

void EntrancePolygon::SimplifyContour(ConvexHullf &contour)
{
	if (contour.size() <= 4U)
	{
		return;
	}
	ConvexHullf cur_contour;
	cur_contour.swap(contour);
	// anti clock-wise
	std::deque<std::pair<size_t, size_t>> index_vec;
	index_vec.emplace_back(0U, cur_contour.size() - 1U);
	std::vector<size_t> valid_index;
	double outer_threshold = 1.0 * map_params_.resolution_;
	double inner_threshold = 1.0 * map_params_.resolution_;
	valid_index.emplace_back(0U);
	valid_index.emplace_back(cur_contour.size() - 1U);
	while (!index_vec.empty())
	{
		const auto &index_pair = index_vec.front();
		double a = 0.0;
		double b = 0.0;
		double c = 0.0;
		CalculateLineParameters(cur_contour[index_pair.first], cur_contour[index_pair.second], a, b, c);
		auto const_info = std::sqrt(a * a + b * b);
		double distance_max_inner = 0.0;
		double distance_max_outer = 0.0;
		size_t index_max_inner;
		size_t index_max_outer;
		for (size_t i = index_pair.first + 1U; i < index_pair.second; ++i)
		{
			auto distance = std::abs(a * cur_contour[i].x + b * cur_contour[i].y + c) / const_info;
			if (JudgePointLeft(cur_contour[index_pair.first], cur_contour[index_pair.second], cur_contour[i]))
			{
				if (distance > distance_max_outer)
				{
					distance_max_outer = distance;
					index_max_outer = i;
				}
			}
			else
			{
				if (distance > distance_max_inner)
				{
					distance_max_inner = distance;
					index_max_inner = i;
				}
			}
		}
		if (distance_max_inner > inner_threshold)
		{
			index_vec.emplace_back(index_pair.first, index_max_inner);
			index_vec.emplace_back(index_max_inner, index_pair.second);
			valid_index.emplace_back(index_max_inner);
		}
		else if (distance_max_outer > outer_threshold)
		{
			index_vec.emplace_back(index_pair.first, index_max_outer);
			index_vec.emplace_back(index_max_outer, index_pair.second);
			valid_index.emplace_back(index_max_outer);
		}
		else
		{
			// do nothing
		}
		index_vec.pop_front();
	}
	std::sort(valid_index.begin(), valid_index.end());
	for (const auto &index : valid_index)
	{
		contour.emplace_back(cur_contour[index]);
	}
}

void EntrancePolygon::CalculateLineParameters(const cv::Point2f &p1, const cv::Point2f &p2, double &a, double &b, double &c)
{
	a = p2.y - p1.y;
	b = p1.x - p2.x;
	c = p2.x * p1.y - p1.x * p2.y;
}

bool EntrancePolygon::JudgePointLeft(const cv::Point2f &p1, const cv::Point2f &p2, const cv::Point2f &p3)
{
	auto value = (p1.x - p3.x) * (p2.y - p3.y) - (p1.y - p3.y) * (p2.x - p3.x);
	return (value >= 0);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "polygon");
	if (argc < 2)
	{
		std::cout << "Err: ----Please input the map information----" << std::endl;
		return 0;
	}

	EntrancePolygon enter_node(argv[1]);

	static_cast<void>(enter_node.Start());
}