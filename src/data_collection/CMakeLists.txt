cmake_minimum_required(VERSION 3.0.2)
project(data_collection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++14")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
COMPONENTS
    roscpp
    sensor_msgs
    cv_bridge
    image_transport
    pcl_conversions
    livox_ros_driver
)

find_package(PCL REQUIRED)
find_package(OpenCV)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES data_collection
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidar_camera_collection src/lidar_camera_collection.cpp)
target_link_libraries(lidar_camera_collection ${catkin_LIBRARIES} ${OpenCV_LIBS})

