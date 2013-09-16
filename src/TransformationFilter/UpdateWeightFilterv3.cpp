#include "UpdateWeightFilterv3.h"

using namespace std;
UpdateWeightFilterv3::UpdateWeightFilterv3(){
	limit = 0.02f;
	bad_weight = 2;
	min_points = 1000;
}
UpdateWeightFilterv3::~UpdateWeightFilterv3(){}
Transformation * UpdateWeightFilterv3::filterTransformation(Transformation * input){
	
	struct timeval start, end;
	gettimeofday(&start, NULL);
	RGBDFrame * src = input->src;
	RGBDFrame * dst = input->dst;
	FrameInput * src_fi = src->input;
	FrameInput * dst_fi = dst->input;

	Transformation * transformation = input->clone();

/*	
	Transformation * transformation = new Transformation();

	transformation->transformationMatrix = input->transformationMatrix;
	transformation->src = src;
	transformation->dst = dst;
	
	transformation->weight = input->weight;//0;
	for(int i = 0; i < input->matches.size();i++){
		KeyPoint * src_kp = input->matches.at(i).first;
		KeyPoint * dst_kp = input->matches.at(i).second;
		transformation->matches.push_back(make_pair(src_kp, dst_kp));
	}
*/

	Eigen::Matrix4f transformationMat  = input->transformationMatrix;
	Eigen::Matrix4f transformationMat_inv = transformationMat.inverse();

	float counter_good = 0;
	float counter_bad = 0;
	float counter_total = 0;

	float d1[dst->validation_points.size()];

	int nr_valid = 0;

	src_fi->getDiff(transformationMat_inv , dst->validation_points , d1, nr_valid );
	for(int i = 0; i < nr_valid; i++){
		float diff = d1[i];
		if(fabs(diff) < limit)		{counter_good++;}
		counter_total++;
	}

	float d2[src->validation_points.size()];
	nr_valid = 0;
	dst_fi->getDiff(transformationMat , src->validation_points , d2, nr_valid );
	for(int i = 0; i < nr_valid; i++){
		float diff = d2[i];
		if(fabs(diff) < limit)		{counter_good++;}
		counter_total++;
	}

	if(counter_total < min_points){counter_total = min_points;}
	transformation->weight = 10 + (counter_good-bad_weight*counter_bad)/counter_total;

	//printf("input w: %f update weight results: %f %f %f-> %f\n",input->weight,counter_good,counter_bad,counter_total,transformation->weight);

	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	//printf("UpdateWeightFilterv3 cost: %f\n",time);
	
	return transformation;
}
void UpdateWeightFilterv3::print(){printf("%s\n",name.c_str());}
void UpdateWeightFilterv3::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}
