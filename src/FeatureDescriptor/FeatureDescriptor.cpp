#include "FeatureDescriptor.h"

FeatureDescriptor::FeatureDescriptor(){type = unitiated;};

FeatureDescriptor::~FeatureDescriptor(){};

void FeatureDescriptor::print(){}

inline double FeatureDescriptor::distance(FeatureDescriptor * other_descriptor)
{
	if(type != other_descriptor->type){return 999999;}
	else
	{
		if(type == surf64){			return ((SurfFeatureDescriptor64 * )(this))->distance((SurfFeatureDescriptor64 * )other_descriptor);}
		return -1;
	}
}

inline FeatureDescriptor * FeatureDescriptor::add(FeatureDescriptor * feature){return this;}

inline FeatureDescriptor * FeatureDescriptor::mul(float val){return this;}

inline FeatureDescriptor * FeatureDescriptor::clone(){
	if		(type == surf64)	{	return ((SurfFeatureDescriptor64 * 	)(this))->clone();}
	return 0;
}

inline void FeatureDescriptor::update(vector<FeatureDescriptor * > * input){printf("no rule for update...\n");}

inline void FeatureDescriptor::store(string path){printf("no rule for store...\n");}

void FeatureDescriptor::normalize(){}
