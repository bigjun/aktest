#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//Opencv
#include "cv.h"
#include "highgui.h"
#include <opencv.hpp>




#include "ekz.h"

#include <iostream>
#include <fstream>
#include <string>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <pthread.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

using namespace std;

int counter = 0;
Calibration * cal;
Map3D * m;

int main(int argc, char **argv)
{

	printf("starting testing software2\n");
	for(int i = 0; i < argc; i++){printf("%i -> %s\n",i,argv[i]);}
	string bow_path = "/home/johane/catkin_ws/src/6dslam/output/bowTest7_%i.feature.surf";
	vector<FeatureDescriptor * > words;
/*
	for(int i = 0; i < 500; i++){
		char buff[250];
		sprintf(buff,bow_path.c_str(),i);
		//printf("%s\n",buff);
		words.push_back(new SurfFeatureDescriptor64(string(buff)));
	}
*/
	cal = new Calibration();
	cal->fx			= 525.0;
	cal->fy			= 525.0;
	cal->cx			= 319.5;
	cal->cy			= 239.5;
	cal->ds			= 1.0;
	cal->scale		= 1000*16*(4.0/5.0);
	cal->words 		= words;

	FilterMatcher * fm1 = new FilterMatcher(new AICK(),0);
	fm1->addFilter(new UpdateWeightFilterv3(), 00.80);

	FilterMatcher * fm2 = new FilterMatcher(new AICK(),100);
	fm2->addFilter(new UpdateWeightFilterv3(), 10.95);

	//FilterMatcher * fm1 = new FilterMatcher(new BowAICK(800, 10, 0.4,0.28, 0.08,0.25),0);
	//fm1->addFilter(new UpdateWeightFilterv3(), 00.80);
	//FilterMatcher * fm2 = new FilterMatcher(new BowAICK(800, 10, 0.4,0.28, 0.07,0.25),100);
	//fm2->addFilter(new UpdateWeightFilterv3(), 10.95);

	m = new Map3D();

	m->matcher 				    = fm1;
	m->loopclosure_matcher 		= fm2;
	m->segmentation 			= new RGBDSegmentation();
	m->extractor 				= new SurfExtractor();

	//string input = string(argv[1]);
	string input = string("../new_data/johan-rec/");

	int missing = 0;
	int i;
	for(i = 170; i <= 250 && true ; i+=1){
		char rgbbuf[512];
		char depthbuf[512];
		sprintf(rgbbuf,"%s/RGB%.10i.png",input.c_str(),i+1);
		sprintf(depthbuf,"%s/Depth%.10i.png",input.c_str(),i+1);
		printf("adding %s\n",rgbbuf);
		FrameInput * fi = new FrameInput(cal, string(rgbbuf) , string(depthbuf));
		if(fi->rgb_img == 0 || fi->depth_img == 0 ){
			missing++;
			if(missing > 10){
				printf("blargh\n");break;
			}else{continue;}
		}
		else{missing=0;m->addFrame(fi);}
	}
	m->estimate();
	m->getLargestComponent();
	m->savePCD("test4.pcd");
	return 0;
}
