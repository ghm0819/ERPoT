cmake_minimum_required(VERSION 3.0.2)
project(ERPoT)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
	rosbag
	roscpp
  	rospy
  	cv_bridge
  	pcl_conversions
  	std_msgs
  	sensor_msgs
  	geometry_msgs
  	nav_msgs
  	message_generation
	image_transport
	tf
	sparse_bundle_adjustment
)

include_directories(${rosbag_INCLUDE_DIRS})
link_libraries(${rosbag_LIBRARIES})

find_package(OpenCV REQUIRED
	COMPONENTS
	opencv_highgui
	CONFIG
)

find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS filesystem thread system program_options)
pkg_check_modules(ncurses REQUIRED ncurses)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)

catkin_package(
  	CATKIN_DEPENDS
)

include_directories(
    	${catkin_INCLUDE_DIRS}
    	${PCL_INCLUDE_DIRS}
    	${OpenCV_INCLUDE_DIRS}
	${CERES_INCLUDE_DIRS}
	${JSONCPP_INCLUDE_DIRS}
	include
	nearest_obstacle
	concave_polygon
	scan_factor
	feature_association
	slam_karto
	gen_rosbag
)

set(NEAREST_OBSTACLE_FILE
	nearest_obstacle/nearest_obstacle.cpp)

set(POSE_TRACKING_FILE
	feature_association/vertex.cpp
	feature_association/edge.cpp
	feature_association/polygon.cpp
	feature_association/map.cpp
	feature_association/feature_association.cpp
)

set(SLAM_KARTO_FILE
	slam_karto/slam_karto.cpp
	slam_karto/spa_solver.cpp
	slam_karto/Karto.cpp
	slam_karto/Mapper.cpp
)

add_executable(ground_segmentation src/entrance_obstacle.cpp ${NEAREST_OBSTACLE_FILE})
target_link_libraries(ground_segmentation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(generate_polygon src/entrance_polygon.cpp)
target_link_libraries(generate_polygon yaml-cpp ${JSONCPP_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(pose_tracking src/entrance_tracking.cpp ${POSE_TRACKING_FILE})
target_link_libraries(pose_tracking yaml-cpp ${JSONCPP_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(slam_karto src/entrance_gridmap.cpp ${SLAM_KARTO_FILE})
target_link_libraries(slam_karto ${catkin_LIBRARIES})

add_executable(kitti_loader src/kitti_loader.cpp)
target_link_libraries(kitti_loader ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${ncurses_LIBRARIES})