#ifndef Map3D_H_
#define Map3D_H_

//other
#include <stdio.h>
#include <string.h>
#include <string>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include "RGBDFrame.h"
#include "Transformation.h" 

#include "FrameMatcher.h"
#include "FeatureExtractor.h"
#include "RGBDSegmentation.h"

#include "core/sparse_optimizer.h"
#include "core/hyper_graph.h"
#include "core/block_solver.h"
#include "core/solver.h"

#include "solvers/structure_only/structure_only_solver.h"
#include "solvers/pcg/linear_solver_pcg.h"

#include "types/slam3d/se3quat.h"
#include "types/slam3d/vertex_se3.h"
#include "types/slam3d/edge_se3.h"

#include <boost/thread/thread.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <iostream>
#include <queue>
#include <iomanip>

//other
#include <stdio.h>
#include <string.h>
#include <string>
#include <iostream>

#include "RGBDFrame.h"
#include "Transformation.h" 

#include "FrameMatcher.h"
#include "FeatureExtractor.h"

#include "FrameInput.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/common/transforms.h>

#include "/opt/ros/groovy/include/g2o/core/sparse_optimizer.h"
#include "/opt/ros/groovy/include/g2o/core/hyper_graph.h"
#include "/opt/ros/groovy/include/g2o/core/block_solver.h"
#include "/opt/ros/groovy/include/g2o/core/solver.h"

#include "/opt/ros/groovy/include/g2o/core/block_solver.h"

#include "/opt/ros/groovy/include/g2o/solvers/structure_only/structure_only_solver.h"
#include "/opt/ros/groovy/include/g2o/solvers/pcg/linear_solver_pcg.h"

#include "/opt/ros/groovy/include/g2o/types/slam3d/se3quat.h"
#include "/opt/ros/groovy/include/g2o/types/slam3d/vertex_se3.h"
#include "/opt/ros/groovy/include/g2o/types/slam3d/edge_se3.h"

#include "core/sparse_optimizer.h"
#include "core/block_solver.h"
#include "core/factory.h"
#include "core/optimization_algorithm_factory.h"
#include "core/optimization_algorithm_gauss_newton.h"
//#include "solvers/cholmod/linear_solver_cholmod.h"

#include "core/sparse_optimizer.h"
#include "core/block_solver.h"
#include "core/solver.h"
#include "core/optimization_algorithm_levenberg.h"
//#include "solvers/csparse/linear_solver_csparse.h"
//#include "solvers/csparse/linear_solver_csparse.h"
//#include "solvers/csparse/linear_solver_csparse.h"

using namespace std;
using namespace Eigen;

struct TodoTransformation {
	double priority;
	RGBDFrame * src;
	RGBDFrame * dst;
};

class CompareTodoTransformation {
	public:
    bool operator()(TodoTransformation& t1, TodoTransformation& t2){return t1.priority > t2.priority;}
};

class Map3D
{
	public:
	FrameMatcher * matcher;
	FrameMatcher * loopclosure_matcher;
	FeatureExtractor * extractor;
	RGBDSegmentation * segmentation;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	
	vector<RGBDFrame *> frames;
	vector<Transformation *> transformations;
	vector<Matrix4f> poses;
	vector<int> largest_component;
	bool show;
	
	Map3D();
	virtual ~Map3D(); 
	
	virtual g2o::SE3Quat getQuat(Eigen::Matrix4f mat);
	virtual unsigned long getCurrentTime();

	virtual vector<int> getLargestComponent();

	virtual void cleanTransformations(float threshold);

	virtual void addFrame(FrameInput * fi);
	virtual void addFrame(RGBDFrame * frame);
	virtual void addTransformation(Transformation * transformation);
	
	virtual void estimate();
	virtual void setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view);
	virtual void visualize();
	virtual void showTuning();
	virtual void savePCD(string path);
};

#include "Map3Dv2.h"

#endif
