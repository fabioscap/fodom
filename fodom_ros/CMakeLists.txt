cmake_minimum_required(VERSION 3.8)
project(fodom_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


add_library(fodom_ros_components SHARED
  src/fodom_ros.cpp
)
ament_target_dependencies(fodom_ros_components PUBLIC
  rclcpp 
  rclcpp_components
  std_msgs
  tf2_ros
  geometry_msgs
  sensor_msgs
  nav_msgs
)

target_include_directories(fodom_ros_components PUBLIC
	"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
	"$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

rclcpp_components_register_node(
  fodom_ros_components
  PLUGIN "fodom_ros::fodomROS"
  EXECUTABLE fodom_ros_node
)

install(
  TARGETS fodom_ros_components
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install( DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch      
		 FILES_MATCHING PATTERN "*.launch.py" 
)

install( DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config     
		 FILES_MATCHING PATTERN "*" 
)

ament_package()