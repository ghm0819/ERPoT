#include "kitti_loader.h"

// global information
using namespace std;
typedef boost::tokenizer<boost::char_separator<char>> tokenizer;

int fileNameFilter(const struct dirent *cur)
{
	std::string str(cur->d_name);
	if (str.find(".bin") != std::string::npos || str.find(".velodata") != std::string::npos || str.find(".pcd") != std::string::npos || str.find(".png") != std::string::npos || str.find(".jpg") != std::string::npos || str.find(".txt") != std::string::npos)
	{
		return 1;
	}
	return 0;
}

bool GetAllFiles(const std::string &dir_in, std::vector<std::string> &files)
{
	if (dir_in.empty())
	{
		return false;
	}
	struct stat s;
	stat(dir_in.c_str(), &s);
	if (!S_ISDIR(s.st_mode))
	{
		return false;
	}
	DIR *open_dir = opendir(dir_in.c_str());
	if (NULL == open_dir)
	{
		std::exit(EXIT_FAILURE);
	}
	dirent *p = nullptr;
	while ((p = readdir(open_dir)) != nullptr)
	{
		struct stat st;
		if (p->d_name[0] != '.')
		{
			std::string name = dir_in + std::string("/") + std::string(p->d_name);
			stat(name.c_str(), &st);
			if (S_ISDIR(st.st_mode))
			{
				GetAllFiles(name, files);
			}
			else if (S_ISREG(st.st_mode))
			{
				boost::char_separator<char> sepp{"."};
				tokenizer tokn(std::string(p->d_name), sepp);
				vector<string> filename_sep(tokn.begin(), tokn.end());
				string type_ = "." + filename_sep[1];
				break;
			}
		}
	}

	struct dirent **namelist;
	int n = scandir(dir_in.c_str(), &namelist, fileNameFilter, alphasort);
	if (n < 0)
	{
		return false;
	}
	for (int i = 0; i < n; ++i)
	{
		std::string filePath(namelist[i]->d_name);
		files.push_back(filePath);
		free(namelist[i]);
	};
	free(namelist);
	closedir(open_dir);
	return true;
}

bool LoadSensorDataPath(std::vector<std::string> &lidarfile_name, string &path)
{
	string lidar_file_path = path;
	cout << lidar_file_path << endl;
	return GetAllFiles(lidar_file_path, lidarfile_name);
}

bool LoadKittiPointcloud(pcl::PointCloud<pcl::PointXYZI> &cloud_in, const string &path)
{
	ifstream inputfile;
	inputfile.open(path, ios::binary);
	if (!inputfile)
	{
		cerr << "ERROR: Cannot open file " << path << "! Aborting..." << endl;
		return false;
	}

	inputfile.seekg(0, ios::beg);
	for (int i = 0; inputfile.good() && !inputfile.eof(); i++)
	{
		pcl::PointXYZI point;
		inputfile.read((char *)&point.x, 3 * sizeof(float));
		inputfile.read((char *)&point.intensity, sizeof(float));
		cloud_in.points.push_back(point);
	}
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kitti_node");
	ros::NodeHandle nh("~");
	;
	///////////////////
	std::string dataset_cali_folder;
	std::string dataset_pose_folder;
	std::string dataset_folder;
	std::string sequence_number;
	std::string output_bag_file;
	int start_index;
	int end_index;
	nh.getParam("dataset_cali_folder", dataset_cali_folder);
	nh.getParam("dataset_pose_folder", dataset_pose_folder);
	nh.getParam("dataset_folder", dataset_folder);
	nh.getParam("sequence_number", sequence_number);
	nh.getParam("start_index", start_index);
	nh.getParam("end_index", end_index);
	std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << '\n';
	bool to_bag;
	nh.getParam("to_bag", to_bag);
	if (to_bag)
	{
		nh.getParam("output_bag_file", output_bag_file);
	}

	int publish_delay;
	nh.getParam("publish_delay", publish_delay);
	publish_delay = publish_delay <= 0 ? 1 : publish_delay;

	///////////// publish information
	ros::Publisher pub_laser_cloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1);
	ros::Publisher pubOdomGT = nh.advertise<nav_msgs::Odometry>("/odometry_gt", 1);
	image_transport::ImageTransport it(nh);

	nav_msgs::Odometry odomGT;
	odomGT.header.frame_id = "odom";
	odomGT.child_frame_id = "lidar_link";

	ros::Publisher pubPathGT = nh.advertise<nav_msgs::Path>("/path_gt", 1);
	nav_msgs::Path pathGT;
	pathGT.header.frame_id = "odom";

	std::string timestamp_path = "/" + sequence_number + "/times.txt";
	std::ifstream timestamp_file(dataset_cali_folder + timestamp_path, std::ifstream::in);

	std::string ground_truth_path = "/" + sequence_number + "_new.txt";
	std::ifstream ground_truth_file(dataset_pose_folder + ground_truth_path, std::ifstream::in);

	std::string calibrate_path = "/" + sequence_number + "/calib.txt";
	std::ifstream calibrate_file(dataset_cali_folder + calibrate_path, std::ifstream::in);

	rosbag::Bag bag_out;
	if (to_bag)
	{
		bag_out.open(output_bag_file, rosbag::bagmode::Write);
	}
	std::string line_cali;
	for (std::size_t times = 0; times < 5U; ++times)
	{
		std::getline(calibrate_file, line_cali);
	}

	std::stringstream cali_stream(line_cali);
	std::string cali_s;
	Eigen::Matrix<double, 3, 4> cali_matrix;
	std::getline(cali_stream, cali_s, ' ');
	for (std::size_t i = 0; i < 3; ++i)
	{
		for (std::size_t j = 0; j < 4; ++j)
		{
			std::getline(cali_stream, cali_s, ' ');
			cali_matrix(i, j) = stof(cali_s);
		}
	}
	Eigen::Quaterniond q_transform(cali_matrix.topLeftCorner<3, 3>());
	q_transform.normalize();
	Eigen::Vector3d t_transform = cali_matrix.topRightCorner<3, 1>();
	auto q_transform_inverse = q_transform.inverse();
	Eigen::Vector3d t_transform_inverse = q_transform_inverse * (-cali_matrix.topRightCorner<3, 1>());
	///////////////////

	// ##################### data path ########################
	std::string lidar_data_path = dataset_folder + "/" + sequence_number + "/velodyne/";

	vector<string> lidarname;
	if (!LoadSensorDataPath(lidarname, lidar_data_path))
	{
		cout << "Detecion file wrong!" << endl;
		std::abort();
	}

	int maxframe = lidarname.size();
	if (end_index > maxframe)
	{
		end_index = maxframe;
	}

	int frame = 0;
	std::string line;

	ros::Rate r(10.0 / publish_delay);

	static tf::TransformBroadcaster tf_broadcaster;

	initscr();
	cbreak();
	keypad(stdscr, TRUE);
	nodelay(stdscr, TRUE);
	bool state = false;

	while (frame < start_index)
	{
		std::getline(timestamp_file, line);
		std::getline(ground_truth_file, line);
		++frame;
	}

	while (ros::ok() && frame < end_index)
	{
		// control state
		int keyStroke = getch();
		if (keyStroke == KEY_UP)
		{
			if (state)
			{
				state = false;
				std::cout << "stop......" << std::endl;
				usleep(100000);
				continue;
			}
			else
			{
				state = true;
				std::cout << "start......" << std::endl;
			}
		}
		if (!state)
		{
			usleep(100000);
			continue;
		}
		if (!std::getline(timestamp_file, line))
		{
			break;
		}
		/////////////////
		std::cout << "current_num: " << frame << std::endl;

		// ############# load pose and timestamp ##############
		float timestamp = stof(line);
		std::getline(ground_truth_file, line);
		std::stringstream pose_stream(line);
		std::string s;
		Eigen::Matrix<double, 3, 4> gt_pose;
		for (std::size_t i = 0; i < 3U; ++i)
		{
			for (std::size_t j = 0; j < 4U; ++j)
			{
				std::getline(pose_stream, s, ' ');
				gt_pose(i, j) = stof(s);
			}
		}

		Eigen::Quaterniond q_w_i(gt_pose.topLeftCorner<3, 3>());
		// Eigen::Quaterniond q = q_transform_inverse * q_w_i * q_transform;
		// q.normalize();
		// Eigen::Vector3d t = q_transform_inverse * q_w_i * t_transform + q_transform_inverse * gt_pose.topRightCorner<3, 1>() +
		// 	t_transform_inverse;
		Eigen::Quaterniond q = q_w_i;
		q.normalize();
		Eigen::Vector3d t = gt_pose.topRightCorner<3, 1>();

		odomGT.header.stamp = ros::Time().fromSec(timestamp);
		odomGT.pose.pose.orientation.x = q.x();
		odomGT.pose.pose.orientation.y = q.y();
		odomGT.pose.pose.orientation.z = q.z();
		odomGT.pose.pose.orientation.w = q.w();
		odomGT.pose.pose.position.x = t(0);
		odomGT.pose.pose.position.y = t(1);
		odomGT.pose.pose.position.z = t(2);
		// pubOdomGT.publish(odomGT);

		tf::Transform ident;
		ident.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
		ident.setRotation(tf::Quaternion(0, 0, 0, 1));
		tf::StampedTransform base_link_2_laser = tf::StampedTransform(ident, odomGT.header.stamp, "base_link", "lidar_link");
		// tf_broadcaster.sendTransform(base_link_2_laser);

		tf::StampedTransform link_2_laser = tf::StampedTransform(ident, odomGT.header.stamp, "lidar_link", "laser");
		// tf_broadcaster.sendTransform(link_2_laser);

		tf::StampedTransform map_2_odom = tf::StampedTransform(ident, odomGT.header.stamp, "map", "odom");
		// tf_broadcaster.sendTransform(map_2_odom);

		tf::Transform tCur;
		tf::poseMsgToTF(odomGT.pose.pose, tCur);
		tf::StampedTransform odom_2_base_link = tf::StampedTransform(tCur, odomGT.header.stamp, "odom",
																	 "base_link");
		// tf_broadcaster.sendTransform(odom_2_base_link);

		geometry_msgs::PoseStamped poseGT;
		poseGT.header = odomGT.header;
		poseGT.pose = odomGT.pose.pose;
		pathGT.header.stamp = odomGT.header.stamp;
		pathGT.poses.emplace_back(poseGT);
		// pubPathGT.publish(pathGT);

		// ##################### load data ########################
		string cloudpath = lidar_data_path + lidarname[frame];

		pcl::PointCloud<pcl::PointXYZI> cloud;

		LoadKittiPointcloud(cloud, cloudpath);
		sensor_msgs::PointCloud2 laser_cloud_msg;
		pcl::toROSMsg(cloud, laser_cloud_msg);
		laser_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);
		laser_cloud_msg.header.frame_id = "lidar_link";
		pub_laser_cloud.publish(laser_cloud_msg);

		if (to_bag)
		{
			bag_out.write("/velodyne_points", ros::Time::now(), laser_cloud_msg);
			bag_out.write("/path_gt", ros::Time::now(), pathGT);
			bag_out.write("/odometry_gt", ros::Time::now(), odomGT);
		}

		cv::waitKey(1);

		r.sleep();
		frame++;
	}
	bag_out.close();
	std::cout << "Done \n";
	endwin();
	return 0;
}