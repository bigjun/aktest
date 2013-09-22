#include "MultiTypeMatcher.h"

using namespace std;

MultiTypeMatcher::MultiTypeMatcher()	{name = "MultiTypeMatcher";}

MultiTypeMatcher::~MultiTypeMatcher()	{printf("delete MultiTypeMatcher\n");}

void MultiTypeMatcher::addMatcher(FrameMatcher * f){matchers.push_back(f);}
void MultiTypeMatcher::setFilter(TransformationFilter * f){filter = f;}
void MultiTypeMatcher::update(){}

Transformation * MultiTypeMatcher::getTransformation(RGBDFrame * src, RGBDFrame * dst)
{
	struct timeval start, end;
	gettimeofday(&start, NULL);

	Transformation * best = 0;

	for(int i = 0; i < matchers.size(); i++){
		Transformation * transformation 	= matchers.at(i)->getTransformation(src,dst);
		Transformation * transformation_new = filter->filterTransformation(transformation);
		printf("%i:transformation_new->weight: %f\n",i,transformation_new->weight);
		if(best == 0){
			best = transformation_new;
		}else if(best->weight < transformation_new->weight){
			delete best;
			best = transformation_new;
		}else{
			delete transformation_new;
		}
	}


	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	//printf("MultiTypeMatcher cost: %f\n",time);
	return best;
}
