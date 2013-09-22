#ifndef MultiTypeMatcher_H_
#define MultiTypeMatcher_H_
//OpenCV
//#include "cv.h"
//#include "highgui.h"
//#include <opencv.hpp>

#include <string>
#include <iostream>
#include <stdio.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Core>

#include "FrameMatcher.h"
#include "TransformationFilter/TransformationFilter.h"

using namespace std;

class MultiTypeMatcher: public FrameMatcher
{
	public:
		vector<FrameMatcher *> matchers;
		TransformationFilter * filter;
		
		void addMatcher(FrameMatcher * f);
		void setFilter(TransformationFilter * f);

		MultiTypeMatcher();
		~MultiTypeMatcher();
		Transformation * getTransformation(RGBDFrame * src, RGBDFrame * dst);
		void update();
};

#endif
