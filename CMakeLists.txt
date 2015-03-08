cmake_minimum_required(VERSION 2.8.3)
project(fspf)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS pcl_ros roscpp rospy std_msgs genmsg)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(localizer localizer.cpp)
target_link_libraries(localizer ${catkin_LIBRARIES})
