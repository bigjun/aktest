#include "UpdateWeightFilterv4.h"

using namespace std;
UpdateWeightFilterv4::UpdateWeightFilterv4(){
	limit = 0.02f;
	bad_weight = 2;
	min_points = 50000;
}
UpdateWeightFilterv4::~UpdateWeightFilterv4(){}
Transformation * UpdateWeightFilterv4::filterTransformation(Transformation * input){
	
	struct timeval start, end;
	gettimeofday(&start, NULL);
	RGBDFrame * src = input->src;
	RGBDFrame * dst = input->dst;
	FrameInput * src_fi = src->input;
	FrameInput * dst_fi = dst->input;

	Transformation * transformation = input->clone();

	Eigen::Matrix4f transformationMat  = input->transformationMatrix;
	Eigen::Matrix4f transformationMat_inv = transformationMat.inverse();

	float counter_good = 0;
	float counter_bad = 0;
	float counter_total = 0;

	float total_diff2 = 0;

	float max_pos = 0.05f;
	float max_neg = -0.05f;

	float d1[dst->validation_points.size()];
	int nr_valid = 0;
	src_fi->getDiff(transformationMat_inv , dst->validation_points , d1, nr_valid );
	for(int i = 0; i < nr_valid; i++){
		float diff = d1[i];
		if(diff > max_pos)		{diff = max_pos;}
		else if(diff < max_neg)	{diff = max_neg;}
		total_diff2 += diff*diff;
		counter_total++;
	}

	float d2[src->validation_points.size()];
	nr_valid = 0;
	dst_fi->getDiff(transformationMat , src->validation_points , d2, nr_valid );
	for(int i = 0; i < nr_valid; i++){
		float diff = d2[i];
		if(diff > max_pos)		{diff = max_pos;}
		else if(diff < max_neg)	{diff = max_neg;}
		total_diff2 += diff*diff;
		counter_total++;
	}

	int mp = dst->validation_points.size()+src->validation_points.size();
	mp /= 10;
	//printf("mp:%i\n",mp);
	if(counter_total < mp){counter_total = mp;}
	//printf("total_diff2:%f, counter_total:%f\n",total_diff2,counter_total);
	//transformation->weight = sqrt(total_diff2/counter_total);
	transformation->weight = 1.0 / sqrt(total_diff2/counter_total);

	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	//printf("UpdateWeightFilterv4 cost: %f\n",time);
	return transformation;
}
void UpdateWeightFilterv4::print(){printf("%s\n",name.c_str());}
void UpdateWeightFilterv4::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}
