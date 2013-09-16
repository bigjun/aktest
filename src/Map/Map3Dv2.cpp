#include "Map3Dv2.h"

using namespace std;
using namespace g2o;

void * extract_frame_data_thread( void *ptr )	{((Map3Dv2 *)ptr)->runWorkerThread();}

Map3Dv2::Map3Dv2(){
	frame_input_list_mutex 					= PTHREAD_MUTEX_INITIALIZER;
	transformation_mutex				 	= PTHREAD_MUTEX_INITIALIZER;
	frames_mutex 							= PTHREAD_MUTEX_INITIALIZER;
	
	frame_counter = 0;
	
	for(int i = 0; i < 10; i++){
		pthread_t mythread;
		pthread_create( &mythread, NULL, extract_frame_data_thread, (void *)this);
	}
}
Map3Dv2::~Map3Dv2(){}

void Map3Dv2::runWorkerThread(){
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



void Map3Dv2::addFrame(FrameInput * fi){
	pthread_mutex_lock( &frame_input_list_mutex );
	frame_input_list.push_back(fi);
	pthread_mutex_unlock( &frame_input_list_mutex );
}

void Map3Dv2::addFrame(RGBDFrame * frame){
	printf("add frame: %i\n",frame->id);
	
	pthread_mutex_lock(   &frames_mutex );
	int frames_to_compare_to = frames.size();
	frames.push_back(frame);
	pthread_mutex_unlock( &frames_mutex );

	for(int i = 0; i < frames_to_compare_to; i++){
		TodoTransformation tmp;
		tmp.priority = frames.at(i)->image_descriptor->distance(frame->image_descriptor);
		tmp.src = frame;
		tmp.dst = frames.at(i);
		if(fabs(tmp.src->id - tmp.dst->id) < 3){tmp.priority = 0;}

		pthread_mutex_lock(   &todoTransformation_mutex );
		todoTransformation.push(tmp);
		pthread_mutex_unlock( &todoTransformation_mutex);
	}
}

void Map3Dv2::doTransformation(TodoTransformation tmp){
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

void Map3Dv2::addTransformation(Transformation * transformation){
	//transformation->print();
	if(transformation->weight > 0){
		//printf("addTransformation\n");
		pthread_mutex_lock( &transformation_mutex );
		transformations.push_back(transformation);
		pthread_mutex_unlock( &transformation_mutex );
	}	
}

void Map3Dv2::estimate(){
	while(frame_input_list.size() > 0){
		printf("processing frames, todo: %i\n",frame_input_list.size());		
		usleep(1000000);
	}
	
	pthread_mutex_lock( &todoTransformation_mutex );
	while(todoTransformation.size() > 0 && todoTransformation.top().priority <= 0.07){
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
/*
    std::vector<std::string> kernels;
    RobustKernelFactory::instance()->fillKnownKernels(kernels);
    cout << "Robust Kernels:" << endl;
    for (size_t i = 0; i < kernels.size(); ++i) {
      cout << kernels[i] << endl;
    }
exit(0);
*/
/*
	AbstractRobustKernelCreator* creator = RobustKernelFactory::instance()->creator(strRobustKernel.toStdString());
    if (! creator) {
      cerr << strRobustKernel.toStdString() << " is not a valid robust kernel" << endl;
      return;
    }
*/
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
/*
			EdgeSE3 * regist = new EdgeSE3();
			regist->vertices()[0] = optimizer.vertex(frames.at(frames_added)->id);
			regist->vertices()[1] = optimizer.vertex(frames.at(frames_added-1)->id);
			regist->setInformation(0.1*mat);
			optimizer.addEdge(regist);
*/
		}else{
			frame->setFixed(true);
		}
	}
/*
	for(int i = 0; i < transformations.size(); i++){
		Transformation * t = transformations.at(i);
		if(t->weight > 0){
			printf("looking at transform: %i %i\n",t->plane_matches.size(),t->matches.size());
			for(int i = 0; i < t->plane_matches.size(); i++){
				t->plane_matches.at(i).first->chain->merge(t->plane_matches.at(i).second->chain);
			}
		}
	}

	set<PlaneChain *> plane_chains;
	for(int i = 0; i < frames.size(); i++){
		RGBDFrame * f = frames.at(i);
		for(int ii = 0; ii < f->segments->planes->size(); ii++){
			Plane * p = f->segments->planes->at(ii);
			plane_chains.insert(p->chain);
		}
	}

	cout << "plane_chains";
	for (set<PlaneChain *>::iterator it=plane_chains.begin(); it!=plane_chains.end(); ++it){
		PlaneChain * pc = (*it);
		printf("id %i size: %i\n",pc->id,pc->planes.size());
		VertexPlane * vertexplane = new VertexPlane();
		vertexplane->setId(frames.size()+pc->id);
		vertexplane->setToOriginImpl();
		optimizer.addVertex(vertexplane);
	}
	cout << '\n';

	for(int i = 0; i < frames.size(); i++){
		RGBDFrame * f = frames.at(i);
		for(int ii = 0; ii < f->segments->planes->size(); ii++){
			Plane * p = f->segments->planes->at(ii);
			PlaneChain * pc = p->chain;
			EdgeSE3Plane * edge = new EdgeSE3Plane();
			edge->vertices()[0] = optimizer.vertex(f->id);
			edge->vertices()[1] = optimizer.vertex(frames.size()+pc->id);
			double est[4];
			est[0]= p->normal_x;
			est[1]= p->normal_y;
			est[2]= p->normal_z;
			est[3]= fabs(p->distance(0,0,0));
			edge->setMeasurement(p);
			//((VertexPlane *)edge->vertices()[1])->setEstimateDataImpl(est);
			optimizer.addEdge(edge);
		}
	}


	for(int i = 0; i < frames.size(); i++){
		RGBDFrame * f = frames.at(i);
		IplImage * img_clone = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
		cvCopy( f->input->rgb_img, img_clone, NULL );

		for(int ii = 0; ii < f->segments->planes->size(); ii++){
			Plane * p = f->segments->planes->at(ii);
			int r = p->chain->r;
			int g = p->chain->g;
			int b = p->chain->b;
			for(int j = 0; j < p->w_vec->size(); j++){
				int w = p->w_vec->at(j);
				int h = p->h_vec->at(j);
				cvRectangle(img_clone,cvPoint(w,h),cvPoint(w,h),cvScalar(r, g, b, 0), 1 , 8, 0);
			}
		}

		cvNamedWindow("segments", CV_WINDOW_AUTOSIZE );
		cvShowImage("segments", img_clone);
		cvWaitKey(0);
		cvReleaseImage( &img_clone );
	}
*/
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
void Map3Dv2::visualize(){}
