cmake_minimum_required(VERSION 2.8.3)
project(aktest)

set(CMAKE_CXX_FLAGS "-O4 -g -pg -Wall -Wunknown-pragmas -Wno-unknown-pragmas -Wsign-compare")

link_directories(/usr/lib/)

find_package(catkin REQUIRED COMPONENTS cv_bridge pcl roscpp rospy std_msgs pcl_ros sensor_msgs)

find_package(PCL 1.5 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ekz-slam
#  CATKIN_DEPENDS cv_bridge libg2o opencv pcl
#  DEPENDS system_lib
)

