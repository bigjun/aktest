#ifndef Map3Dv3_H_
#define Map3Dv3_H_

#include "Map3D.h"

using namespace std;
using namespace g2o;

class Map3Dv3: public Map3D
{
	public:
	
	pthread_mutex_t frame_input_list_mutex;
	list<FrameInput *> frame_input_list;
	int frame_counter;
		

	pthread_mutex_t transformation_mutex;	
	pthread_mutex_t frames_mutex;
	
	pthread_mutex_t todoTransformation_mutex;
	priority_queue<TodoTransformation, vector<TodoTransformation>, CompareTodoTransformation> todoTransformation;
	
	Map3Dv3();
	~Map3Dv3(); 

	void runWorkerThread();
	void doTransformation(TodoTransformation tmp);
	
	void addFrame(FrameInput * fi);
	void addFrame(RGBDFrame * frame);
	void addTransformation(Transformation * transformation);
	
	void estimate();
	void visualize();
};
#endif
