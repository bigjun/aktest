#ifndef Transformation_H_
#define Transformation_H_
#include <pcl/visualization/cloud_viewer.h>

//other
#include <utility>
#include "RGBDFrame.h"

using namespace std;

class Transformation {
	public:
		RGBDFrame * src;
		RGBDFrame * dst;
		vector< pair <KeyPoint * ,KeyPoint * > > matches;
		vector< pair <Plane * ,Plane * > > plane_matches;
		Eigen::Matrix4f transformationMatrix;
		double weight;
		
		Transformation * clone();
		void print();
		void show();
		void show(pcl::visualization::CloudViewer * viewer);
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#endif
