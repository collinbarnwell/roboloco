cmake_minimum_required(VERSION 2.8.3)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(fspf)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib) 


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS pcl_ros roscpp rospy std_msgs genmsg)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES compute_cdist
#  CATKIN_DEPENDS roscpp visualization_msgs robot_model_helper compute_cdist
#  DEPENDS assimp Eigen mlpack
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(localizer localizer.cpp)
add_executable(linetest linetest.cpp)

target_link_libraries(localizer ${catkin_LIBRARIES})
target_link_libraries(linetest ${catkin_LIBRARIES})
