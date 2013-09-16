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
include_directories(src/FeatureExtractor)
include_directories(src/RGBDSegmentation)
include_directories(src/core)
include_directories(src/graphcutting)
include_directories(src/TransformationFilter)
include_directories(src/FrameMatcher)
include_directories(src/Map)
#include_directories(src/g2otypes)

add_library(aktest_frameinput src/core/FrameInput.cpp)
target_link_libraries(aktest_frameinput ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_library(aktest_FeatureDescriptor src/FeatureDescriptor/FeatureDescriptor.cpp src/FeatureDescriptor/SurfFeatureDescriptor64.cpp src/FeatureDescriptor/FloatHistogramFeatureDescriptor.cpp)

add_library(aktest_mygeometry src/mygeometry/HasDistance.cpp src/mygeometry/Point.cpp src/mygeometry/Line.cpp src/mygeometry/Plane.cpp src/mygeometry/PlaneChain.cpp src/mygeometry/KeyPoint.cpp src/mygeometry/KeyPointChain.cpp src/mygeometry/KeyPointSet.cpp)
target_link_libraries(aktest_mygeometry aktest_FeatureDescriptor aktest_frameinput)


add_library(aktest_FeatureExtractor src/FeatureExtractor/FeatureExtractor.cpp src/FeatureExtractor/SurfExtractor.cpp)
target_link_libraries(aktest_FeatureExtractor aktest_mygeometry)
target_link_libraries(aktest_FeatureExtractor ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(aktest_FeatureExtractor opencv_calib3d opencv_core opencv_flann opencv_highgui opencv_legacy opencv_contrib opencv_features2d opencv_gpu  opencv_imgproc opencv_ml opencv_objdetect opencv_video)

add_library(aktest_RGBDSegmentation src/RGBDSegmentation/RGBDSegmentation.cpp)
target_link_libraries(aktest_RGBDSegmentation aktest_mygeometry)

add_library(aktest_core src/core/RGBDFrame.cpp src/core/Transformation.cpp)
target_link_libraries(aktest_core aktest_RGBDSegmentation aktest_frameinput aktest_FeatureExtractor)
target_link_libraries(aktest_core ${catkin_LIBRARIES})

add_library(aktest_graphcutting src/graphcutting/GraphForCut.cpp)

add_library(aktest_TransformationFilter src/TransformationFilter/TransformationFilter.cpp src/TransformationFilter/UpdateWeightFilterv3.cpp)
target_link_libraries(aktest_TransformationFilter aktest_core)

add_library(aktest_FrameMatcher src/FrameMatcher/FrameMatcher.cpp src/FrameMatcher/AICK.cpp src/FrameMatcher/FilterMatcher.cpp src/FrameMatcher/BowAICK.cpp)
target_link_libraries(aktest_FrameMatcher aktest_TransformationFilter)

add_library(aktest_map3d src/Map/Map3D.cpp src/Map/Map3Dv2.cpp)
target_link_libraries(aktest_map3d aktest_FrameMatcher)
target_link_libraries(aktest_map3d ${libg2o_LIBRARIES})
target_link_libraries(aktest_map3d g2o_cli g2o_core g2o_csparse_extension g2o_ext_freeglut_minimal g2o_incremental g2o_interactive g2o_interface g2o_opengl_helper g2o_parser g2o_simulator g2o_solver_cholmod g2o_solver_csparse g2o_solver_dense g2o_solver_pcg g2o_solver_slam2d_linear g2o_solver_structure_only g2o_stuff g2o_types_data g2o_types_icp g2o_types_sba g2o_types_sclam2d g2o_types_sim3 g2o_types_slam2d g2o_types_slam3d)

add_executable(aktest_test4 src/test4.cpp)
target_link_libraries(aktest_test4 ${OpenCV_LIBRARIES})
target_link_libraries(aktest_test4 ${catkin_LIBRARIES})
target_link_libraries(aktest_test4 ${PCL_LIBRARIES})
target_link_libraries(aktest_test4 aktest_core)
target_link_libraries(aktest_test4 aktest_map3d)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ekz-slam
#  CATKIN_DEPENDS cv_bridge libg2o opencv pcl
#  DEPENDS system_lib
)

