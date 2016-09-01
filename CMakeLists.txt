cmake_minimum_required(VERSION 2.8.3)
project(image_proc_custom)

find_package(catkin REQUIRED COMPONENTS
	camera_info_manager
	dynamic_reconfigure
	image_transport
	nodelet
	roscpp
	rospy
	tf)

find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# Dynamic reconfigure support.
generate_dynamic_reconfigure_options(cfg/CropDecimate.cfg)

# Nodelet library
add_library(${PROJECT_NAME} src/nodelets/crop_decimate.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

catkin_package()