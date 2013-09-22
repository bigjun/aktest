#ifndef opensurf_files
#define opensurf_files
#include "OpenSurf/surf.cpp"
#include "OpenSurf/fasthessian.cpp"
#include "OpenSurf/integral.cpp"
#include "OpenSurf/ipoint.cpp"
#include "OpenSurf/utils.cpp"
#include "OpenSurf/surflib.h"
#include "OpenSurf/kmeans.h"
#endif

#include "SurfExtractor.h"
#include <ctime>
#include "SurfFeatureDescriptor64.h"
#include <algorithm>

SurfExtractor::SurfExtractor(){
	total_time = 0;
	total_frames = 0;
	total_keypoints = 0;
	upright 		= true;
	octaves 		= 5;
	//intervals 		= 5;
	intervals 		= 3;
	init_sample 	= 2;
	//thres			= 0.001f;
	thres			= 0.00001f;
}

SurfExtractor::~SurfExtractor(){}
using namespace std;
bool comparison_surf (KeyPoint * i,KeyPoint * j) { return (i->stabilety>j->stabilety); }

KeyPointSet * SurfExtractor::getKeyPointSet(FrameInput * fi){

	struct timeval start, end;
	gettimeofday(&start, NULL);

	IpVec ipts;
	surfDetDes(fi->get_rgb_img(), ipts, upright, octaves, intervals, init_sample, thres);
	
	KeyPointSet * keypoints = new KeyPointSet();
	for(int i = 0; i < (int)ipts.size(); i++)
	{
		
		Ipoint p = ipts.at(i);
		int w = int(p.x+0.5f);
		int h = int(p.y+0.5f);
		int ind = h * 640 + w;
		float * desc = new float[64];
		for(int j = 0; j < 64; j++){desc[j] = p.descriptor[j];}
		SurfFeatureDescriptor64 * descriptor = new SurfFeatureDescriptor64(desc, p.laplacian);

		float r,g,b,x,y,z;
		fi->getRGB(r,g,b,w,h);
		fi->getXYZ(x,y,z,w,h);
		
		KeyPoint * kp = new KeyPoint();
		kp->stabilety = p.stab;
		kp->descriptor = descriptor;
		kp->point = new Point(x,y,z,w,h);
		kp->r = r;
		kp->g = g;
		kp->b = b;
		if(z > 0 && !isnan(z)){
			kp->valid = true;
			kp->index_number = keypoints->valid_key_points.size();
			keypoints->valid_key_points.push_back(kp);
			//cvCircle(rgb_img, cvPoint(w, h), 5, cvScalar(0, 255, 0, 0), 2, 8, 0);
		}else{
			kp->valid = false;
			kp->index_number = keypoints->invalid_key_points.size();
			keypoints->invalid_key_points.push_back(kp);
			//cvCircle(rgb_img, cvPoint(w, h), 5, cvScalar(0, 0, 255, 0), 2, 8, 0);
		}
	}
	//printf("pre sort\n");
	sort(keypoints->valid_key_points.begin(),keypoints->valid_key_points.end(),comparison_surf);
	sort(keypoints->invalid_key_points.begin(),keypoints->invalid_key_points.end(),comparison_surf);
	//cvReleaseImage( &rgb_img );

	//cvNamedWindow("surf image", CV_WINDOW_AUTOSIZE );
	//cvShowImage("surf image", rgb_img);
	//cvWaitKey(0);
	//cvReleaseImage( &rgb_img);
	
	gettimeofday(&end, NULL);
	double time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	total_time += time;
	total_frames++;
	//printf("ipts.size(): %i\n",ipts.size());
	total_keypoints+=ipts.size();//keypoints->valid_key_points.size();
	//printf("avg_time: %f avg_keypoints: %f\n",total_time/double(total_frames),double(total_keypoints)/double(total_frames));	
	return keypoints;
}
