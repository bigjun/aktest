#include "Map3Dv3.h"

using namespace std;
using namespace g2o;

void * extract_frame_data_thread( void *ptr )	{((Map3Dv3 *)ptr)->runWorkerThread();}

Map3Dv3::Map3Dv3(){
	frame_input_list_mutex 					= PTHREAD_MUTEX_INITIALIZER;
	transformation_mutex				 	= PTHREAD_MUTEX_INITIALIZER;
	frames_mutex 							= PTHREAD_MUTEX_INITIALIZER;
	
	frame_counter = 0;
	
	for(int i = 0; i < 10; i++){
		pthread_t mythread;
		pthread_create( &mythread, NULL, extract_frame_data_thread, (void *)this);
	}
}
Map3Dv3::~Map3Dv3(){}

void Map3Dv3::runWorkerThread(){
	while(true){
		pthread_mutex_lock( &frame_input_list_mutex );
		pthread_mutex_lock( &todoTransformation_mutex );
		if(frame_input_list.size() > 0){
			pthread_mutex_unlock( &todoTransformation_mutex );

			FrameInput * fi = frame_input_list.front();
			frame_input_list.pop_front();
			int current_id = frame_counter++;
			pthread_mutex_unlock( &frame_input_list_mutex );

			RGBDFrame * frame = new RGBDFrame(fi,extractor,segmentation);
			frame->id = current_id;
			addFrame(frame);

		}else if(todoTransformation.size() > 0){
			pthread_mutex_unlock( &frame_input_list_mutex );

			TodoTransformation tmp = todoTransformation.top();
			todoTransformation.pop();
			pthread_mutex_unlock( &todoTransformation_mutex );
			doTransformation(tmp);
		}else{
			pthread_mutex_unlock( &frame_input_list_mutex );
			pthread_mutex_unlock( &todoTransformation_mutex );
			usleep(10000);
		}
	}
}



void Map3Dv3::addFrame(FrameInput * fi){
	pthread_mutex_lock( &frame_input_list_mutex );
	frame_input_list.push_back(fi);
	pthread_mutex_unlock( &frame_input_list_mutex );
}

void Map3Dv3::addFrame(RGBDFrame * frame){
	//printf("add frame: %i\n",frame->id);
	
	pthread_mutex_lock(   &frames_mutex );
	int frames_to_compare_to = frames.size();
	frames.push_back(frame);
	pthread_mutex_unlock( &frames_mutex );

	for(int i = 0; i < frames_to_compare_to; i++){
		TodoTransformation tmp;
		tmp.priority = fabs(frame->id-frames.at(i)->id);
		tmp.src = frame;
		tmp.dst = frames.at(i);
		if(fabs(tmp.src->id - tmp.dst->id) <= 1){tmp.priority = 0;}

		pthread_mutex_lock(   &todoTransformation_mutex );
		todoTransformation.push(tmp);
		pthread_mutex_unlock( &todoTransformation_mutex);
	}
}

void Map3Dv3::doTransformation(TodoTransformation tmp){
	Transformation * t;
	RGBDFrame * src = tmp.src;
	RGBDFrame * dst = tmp.dst;
	if(src->id < dst->id){
		src = tmp.dst;
		dst = tmp.src;
	}
	if(tmp.priority == 0)	{t = matcher->getTransformation(src,dst);}
	else					{t = loopclosure_matcher->getTransformation(src,dst);}
	addTransformation(t);
}

void Map3Dv3::addTransformation(Transformation * transformation){
	//transformation->print();
	//if(fabs(transformation->src->id - transformation->dst->id) > 3){transformation->print();}
	if(transformation->weight > 0){
		if(fabs(transformation->src->id - transformation->dst->id) > 3){
			printf("addTransformation\n");
			transformation->print();
		}
		pthread_mutex_lock( &transformation_mutex );
		transformations.push_back(transformation);
		pthread_mutex_unlock( &transformation_mutex );
	}	
}

void Map3Dv3::estimate(){
	while(frame_input_list.size() > 0){
		printf("processing frames, todo: %i\n",frame_input_list.size());		
		usleep(1000000);
	}
	
	pthread_mutex_lock( &todoTransformation_mutex );
	while(todoTransformation.size() > 0 && todoTransformation.top().priority < 25){
		float prio = todoTransformation.top().priority;
		
		pthread_mutex_unlock( &todoTransformation_mutex );
		printf("processing transf, todo: %i, prio: %f\n",todoTransformation.size(),prio);		
		usleep(1000000);
		pthread_mutex_lock( &todoTransformation_mutex );
	}

	while(todoTransformation.size() > 0){todoTransformation.pop();}

	pthread_mutex_unlock( &todoTransformation_mutex );

	pthread_mutex_lock( &frames_mutex );
	pthread_mutex_lock( &frame_input_list_mutex );
	pthread_mutex_lock( &todoTransformation_mutex );
	pthread_mutex_lock( &transformation_mutex );

	//cleanTransformations(0);

	getLargestComponent();

	printf("transformations.size() = %i\n",transformations.size());

	typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
	typedef LinearSolverPCG<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

	Eigen::Matrix<double, 6, 6, 0, 6, 6> mat;
	mat.setIdentity(6,6);

	// allocating the optimizer
	SparseOptimizer optimizer;
	SlamLinearSolver* linearSolver = new SlamLinearSolver();
	SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
	OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(blockSolver);
	optimizer.setAlgorithm(solver);

	for(int frames_added = 0; frames_added < frames.size(); frames_added++){
		VertexSE3 * frame = new VertexSE3();
		frame->setId(frames.at(frames_added)->id);
		optimizer.addVertex(frame);
		if(frames_added > 0){

		}else{
			frame->setFixed(true);
		}
	}

	for(int registrations_added = 0; registrations_added < transformations.size(); registrations_added++){
		Transformation * transformation = transformations.at(registrations_added);
		if(transformation->weight > 0){
			EdgeSE3 * regist = new EdgeSE3();
			regist->vertices()[0] = optimizer.vertex(transformation->dst->id);
			regist->vertices()[1] = optimizer.vertex(transformation->src->id);

			Eigen::Affine3d a(transformation->transformationMatrix.cast<double>());
			Eigen::Isometry3d b;
			b.translation() = a.translation();
			b.linear()		= a.rotation();

			regist->setMeasurement(b);
			regist->setInformation(5*mat);
			optimizer.addEdge(regist);

		}
	}

	optimizer.initializeOptimization();
	printf("time to optimize\n");
	optimizer.setVerbose(true);
	optimizer.optimize(10);
	printf("done optimize\n");

	poses.resize(frames.size());
	for(int i  = 0; i < frames.size(); i++){
		VertexSE3 * framevertex = (VertexSE3*)(optimizer.vertex(frames.at(i)->id));
		poses.at(i) = (Eigen::Affine3d(framevertex->estimate())).matrix().cast<float>();
		//cout<<"updating pose"<<i<<endl<<poses.at(i)<<endl;
	}

	pthread_mutex_unlock( &frames_mutex );
	pthread_mutex_unlock( &frame_input_list_mutex );
	pthread_mutex_unlock( &todoTransformation_mutex );
	pthread_mutex_unlock( &transformation_mutex );
}
void Map3Dv3::visualize(){}
