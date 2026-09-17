cmake_minimum_required(VERSION 3.5)
project(resple)

SET(CMAKE_BUILD_TYPE "RELEASE")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -std=c++17 -O3 -march=native -pthread -fopenmp -Wall -DEIGEN_INITIALIZE_MATRICES_BY_NAN")

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(livox_interfaces REQUIRED)
find_package(livox_ros_driver REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(estimate_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenMP REQUIRED)
include(FindOpenMP)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_library(${PROJECT_NAME}
    src/Mapping.cpp
	src/RESPLE.cpp
)
ament_lint_auto_find_test_dependencies()
include_directories(
	include
	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS})
add_executable(Mapping src/Mapping.cpp)
add_executable(RESPLE src/RESPLE.cpp include/ikd-Tree/ikd_Tree.cpp)
	ament_target_dependencies(${PROJECT_NAME} 
	rclcpp
	std_msgs
	geometry_msgs
	nav_msgs
	sensor_msgs
	visualization_msgs
	std_srvs
	livox_interfaces
	livox_ros_driver
	livox_ros_driver2
	estimate_msgs
	pcl_conversions
	ament_lint_auto
	tf2_ros
)
target_link_libraries(RESPLE
		${PROJECT_NAME}
		${PCL_LIBRARIES}
		${YAML_CPP_LIBRARIES}
		${OpenMP_LIBS}
)
target_link_libraries(Mapping
		${PROJECT_NAME}
		${PCL_LIBRARIES}
		${YAML_CPP_LIBRARIES}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
install(TARGETS RESPLE
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS Mapping
  DESTINATION lib/${PROJECT_NAME})
# Exported so glim_ext_resple_bridge can reuse SplineState.h (and its
# utils/common_utils.h, utils/math_tools.h deps) instead of duplicating the
# spline math.
install(DIRECTORY include/
  DESTINATION include)
ament_export_include_directories(include)
# message_runtime is a ROS1/catkin concept with no ROS2 equivalent -- harmless
# for resple's own build, but any package that find_package(resple)'s (e.g.
# glim_ext_resple_bridge) fails resolving it via
# ament_auto_find_build_dependencies(). Dropped rather than left broken.
ament_export_dependencies(rclcpp yaml-cpp livox_ros_driver livox_ros_driver2 livox_interfaces)
# ament_export_libraries(${PROJECT_NAME}) removed: it exports a library named
# "resple" that this CMakeLists builds but never install(TARGETS)'s, so any
# consumer's find_package(resple) fails looking for it. Not needed anyway --
# SplineState.h (what glim_ext_resple_bridge actually uses) is header-only.
ament_package()
