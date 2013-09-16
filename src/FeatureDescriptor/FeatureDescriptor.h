#ifndef FeatureDescriptor_H_
#define FeatureDescriptor_H_
#include <vector>
#include <string>

#include <stdio.h>
using namespace std;

enum DescriptorType { unitiated, surf64, rgb, orb, FPFH, IntegerHistogram, FloatHistogram };

class FeatureDescriptor {
	public:
		DescriptorType type;
		virtual double distance(FeatureDescriptor * other_descriptor);
		FeatureDescriptor();
		virtual ~FeatureDescriptor();
		virtual void print();
		virtual void normalize();
		virtual FeatureDescriptor * add(FeatureDescriptor * feature);
		virtual FeatureDescriptor * mul(float val);
		virtual FeatureDescriptor * clone();
		virtual void update(vector<FeatureDescriptor * > * input);
		virtual void store(string path);
};
#include "SurfFeatureDescriptor64.h"
#include "FloatHistogramFeatureDescriptor.h"
#endif
