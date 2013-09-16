#ifndef FrameInput_H_
#define FrameInput_H_
#include <string>
#include <stdio.h>
#include <vector>
#include "Calibration.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>

//Opencv
#include "cv.h"
#include "highgui.h"
#include <opencv.hpp>

using namespace std;

class FrameInput{
	public:
	int width;
	int height;

	double depth_timestamp;
	string depth_path;
    IplImage * depth_img;
	bool depth_in_ram;
	double depth_last_use_timestamp;

	double rgb_timestamp;
	string rgb_path;
    IplImage * rgb_img;
	bool rgb_in_ram;
	double rgb_last_use_timestamp;

	Calibration * calibration;

	FrameInput();
	FrameInput(Calibration * cal, pcl::PointCloud<pcl::PointXYZRGB> & input_cloud, int i, string path);
	FrameInput(Calibration * cal, string rgb_path, string depth_path);
	virtual ~FrameInput();

	double current_time();

	virtual void load_rgb();
	virtual void release_rgb();
	virtual void load_depth();
	virtual void release_depth();

	virtual float *** full_getNormals(int & w, int & h);	
	
	virtual void getWH(float ** x, float ** y, float ** z,float **  w, float **  h);

	virtual void getWH(float x, float y, float z,int & w, int & h);
	virtual void getDiff(Eigen::Matrix4f & tm, vector< float * > points, float * d, int & nr_valid);

	virtual float *** full_getXYZ(int & w, int & h);
	virtual void getXYZ(float & x, float & y, float & z,int w, int h);
	virtual void vector_getXYZ(vector<float> & x, vector<float> & y, vector<float> & z,vector<int> & w, vector<int> & h);

	virtual float *** full_getRGB(int & w, int & h);
	IplImage * get_rgb_img();
	virtual void getRGB(float & r, float & g, float & b,int w, int h);
	virtual void vector_getRGB(vector<float> & r, vector<float> & g, vector<float> & b,vector<int> & w, vector<int> & h);

	virtual pcl::PointCloud<pcl::PointXYZRGB> getCloud();
	virtual pcl::PointCloud<pcl::Normal>::Ptr getNormals();
	//bool operator< (const Frame_input& other) const {return depth_timestamp < other.depth_timestamp;}
	
};
#endif
