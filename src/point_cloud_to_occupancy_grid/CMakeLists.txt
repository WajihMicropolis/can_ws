cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_to_occupancy_grid)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  nav_msgs
  cv_bridge
  image_transport
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_conversions pcl_ros nav_msgs cv_bridge image_transport
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(map3d_to_map2d src/converter.cpp)

target_link_libraries(map3d_to_map2d
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

add_dependencies(map3d_to_map2d ${catkin_EXPORTED_TARGETS})
