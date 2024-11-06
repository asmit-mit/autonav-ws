cmake_minimum_required(VERSION 3.8)
project(transformer)

# Find ament_cmake and other dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the ZedTfNode executable
add_executable(zed_tf src/zed_tf.cpp)
ament_target_dependencies(zed_tf
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# Add the PointCloudTransformer executable
add_executable(point_cloud_transformer src/point_cloud_transformer.cpp)
ament_target_dependencies(point_cloud_transformer
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

# Add the LidarTfNode executable
add_executable(lidar_tf src/lidar_tf.cpp)
ament_target_dependencies(lidar_tf
  rclcpp
  sensor_msgs
)

# Install the executables
install(TARGETS
  zed_tf
  point_cloud_transformer
  lidar_tf
  DESTINATION lib/${PROJECT_NAME}
)

# Install package.xml
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Specify the package
ament_package()
