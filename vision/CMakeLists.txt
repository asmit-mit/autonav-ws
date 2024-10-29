cmake_minimum_required(VERSION 3.0.2)
project(vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++17")

find_package(
  catkin REQUIRED
  COMPONENTS cv_bridge
             rospy
             roscpp
             pcl_ros
             pcl_conversions
             sensor_msgs
             std_msgs
             nav_msgs
             octomap_ros
             octomap_msgs
             tf2_ros
             tf2
             tf2_sensor_msgs
             tf2_geometry_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(pc_mapper_node src/pc_mapper.cpp)
target_link_libraries(pc_mapper_node ${catkin_LIBRARIES})
