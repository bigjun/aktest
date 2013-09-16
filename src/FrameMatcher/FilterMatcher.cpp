#include "FilterMatcher.h"

using namespace std;

FilterMatcher::FilterMatcher()	{name = "FilterMatcher";}

FilterMatcher::FilterMatcher(FrameMatcher * fm_, float threshold){
	name = "FilterMatcher";
	fm = fm_;
	thresholds.push_back(threshold);
}

FilterMatcher::~FilterMatcher()	{printf("delete FilterMatcher\n");}

void FilterMatcher::addFilter(TransformationFilter * f){filters.push_back(f);}
void FilterMatcher::addFilter(TransformationFilter * f, float threshold){filters.push_back(f);thresholds.push_back(threshold);}

void FilterMatcher::update(){}

Transformation * FilterMatcher::getTransformation(RGBDFrame * src, RGBDFrame * dst)
{
	struct timeval start, end;
	gettimeofday(&start, NULL);
	Transformation * transformation_old = fm->getTransformation(src,dst);
	if(transformation_old->weight < thresholds.at(0)){
		transformation_old->weight = 0;
		return transformation_old;
	}
	for(int i = 0; i < filters.size(); i++){
		Transformation * transformation_new = filters.at(i)->filterTransformation(transformation_old);
		delete transformation_old;
		transformation_old = transformation_new;
		if(transformation_old->weight < thresholds.at(i+1)){
			transformation_old->weight = 0;
			return transformation_old;
		}
	}
	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	//printf("FilterMatcher cost: %f\n",time);
	return transformation_old;
}
