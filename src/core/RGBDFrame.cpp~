#include "RGBDFrame.h"
#include "math.h"
#include "ros/ros.h"

int frame_id_counter = 0;
RGBDFrame::RGBDFrame(){}
RGBDFrame::RGBDFrame(FrameInput * fi, FeatureExtractor * extractor, RGBDSegmentation * segmenter)
{

	struct timeval start, end;
	gettimeofday(&start, NULL);

	id = frame_id_counter++;
	input = fi;

	keypoints = extractor->getKeyPointSet(input);
	for(unsigned int i = 0; i < keypoints->valid_key_points.size();i++){	keypoints->valid_key_points.at(i)->frame_id		=id;}
	for(unsigned int i = 0; i < keypoints->invalid_key_points.size();i++){	keypoints->invalid_key_points.at(i)->frame_id	=id;}

	vector<FeatureDescriptor * > words = fi->calibration->words;
	float * bow = new float[words.size()];
	for(unsigned int j = 0; j < words.size();j++){bow[j]=0;}

	for(unsigned int i = 0; i < keypoints->valid_key_points.size();i++){
		int best_id = 0;
		float best = 10000000000;
		KeyPoint * kp = keypoints->valid_key_points.at(i);
		for(unsigned int j = 0; j < words.size();j++){
			float d = kp->descriptor->distance(words.at(j));
			if(d < 0.30){kp->cluster_distance_pairs.push_back(make_pair(j,d));}
			if(d < best){best = d; best_id = j;}
		}
		bow[best_id]++;
		kp->sortDistances();
	}
	for(unsigned int i = 0; i < keypoints->invalid_key_points.size();i++){
		int best_id = 0;
		float best = 10000000000;
		KeyPoint * kp = keypoints->invalid_key_points.at(i);
		for(unsigned int j = 0; j < words.size();j++){
			float d = kp->descriptor->distance(words.at(j));
			if(d < best){best = d; best_id = j;}
		}
		bow[best_id]++;
		kp->sortDistances();
		delete kp;
	}

	float bow_div = float(keypoints->invalid_key_points.size()+keypoints->valid_key_points.size());
	for(unsigned int j = 0; j < words.size();j++){bow[j]/=bow_div;}	
	image_descriptor = new FloatHistogramFeatureDescriptor(bow,words.size());

	segments = segmenter->segment(input);

	int step = 2;
	for(int w = step; w < input->width; w+=step){
		for(int h = step; h < input->height; h+=step){
			float x,y,z;
			input->getXYZ(x,y,z,w,h);

			if(z > 0){
		       	float * vp = new float[3];
		       	vp[0] = x;
		       	vp[1] = y;
		       	vp[2] = z;
		       	validation_points.push_back(vp);
			}
		}
	}
	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	//ROS_INFO("RGBDFrame total cost: %f",time);
}

RGBDFrame::~RGBDFrame(){printf("~RGBDFrame()\n");}
