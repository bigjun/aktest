
#include "Transformation.h"
#include <string>
#include <iostream>

#include <Eigen/Core>
#include <fstream>

Transformation * Transformation::clone(){
	Transformation * transformation = new Transformation();
	transformation->transformationMatrix = transformationMatrix;
	transformation->src		= src;
	transformation->dst		= dst;
	transformation->weight	= weight;
	for(int i = 0; i < matches.size();i++){
		KeyPoint * src_kp = matches.at(i).first;
		KeyPoint * dst_kp = matches.at(i).second;
		transformation->matches.push_back(make_pair(src_kp, dst_kp));
	}

	for(int i = 0; i < plane_matches.size();i++){
		Plane * src_kp = plane_matches.at(i).first;
		Plane * dst_kp = plane_matches.at(i).second;
		transformation->plane_matches.push_back(make_pair(src_kp, dst_kp));
	}

	return transformation;
}
void Transformation::show(){}
void Transformation::show(pcl::visualization::CloudViewer * viewer){};
void Transformation::print(){printf("transformation: %i --> %i , weight: %f\n",src->id,dst->id,weight);}
