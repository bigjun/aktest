cmake_minimum_required(VERSION 2.8.3)
project(aktest)

set(CMAKE_CXX_FLAGS "-O4 -g -pg -Wall -Wunknown-pragmas -Wno-unknown-pragmas -Wsign-compare")

link_directories(/usr/lib/)

find_package(catkin REQUIRED COMPONENTS cv_bridge pcl roscpp rospy std_msgs pcl_ros sensor_msgs)

find_package(PCL 1.5 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

link_directories(/opt/ros/groovy/lib)
include_directories(/opt/ros/groovy/include/g2o/)
include_directories(/opt/ros/groovy/include/opencv)
include_directories(/opt/ros/groovy/include/opencv2)

include_directories(src)

#include_directories(src/apps)
include_directories(src/FeatureDescriptor)
include_directories(src/mygeometry)
#include_directories(src/FeatureExtractor)
#include_directories(src/RGBDSegmentation)
include_directories(src/core)
#include_directories(src/graphcutting)
#include_directories(src/TransformationFilter)
#include_directories(src/FrameMatcher)
#include_directories(src/Map)
#include_directories(src/g2otypes)

add_library(aktest_frameinput src/core/FrameInput.cpp)
target_link_libraries(aktest_frameinput ${OpenCV_LIBRARIES})

add_library(aktest_FeatureDescriptor src/FeatureDescriptor/FeatureDescriptor.cpp src/FeatureDescriptor/SurfFeatureDescriptor64.cpp src/FeatureDescriptor/FloatHistogramFeatureDescriptor.cpp)

add_library(aktest_mygeometry src/mygeometry/HasDistance.cpp src/mygeometry/Point.cpp src/mygeometry/Line.cpp src/mygeometry/Plane.cpp src/mygeometry/PlaneChain.cpp src/mygeometry/KeyPoint.cpp src/mygeometry/KeyPointChain.cpp src/mygeometry/KeyPointSet.cpp)
target_link_libraries(aktest_mygeometry aktest_FeatureDescriptor aktest_frameinput)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ekz-slam
#  CATKIN_DEPENDS cv_bridge libg2o opencv pcl
#  DEPENDS system_lib
)

