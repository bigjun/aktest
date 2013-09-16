#include "Map3D.h"
#include <map>


//#include <opencv.hpp>

using namespace std;

bool comparison_Map3Dbase (Transformation * i,Transformation * j) {
	if(i->src == j->src){return (i->weight<j->weight);}
	else{return (i->src->id<j->src->id);}
}

Map3D::Map3D(){}
Map3D::~Map3D(){}
void Map3D::addFrame(FrameInput * fi){addFrame(new RGBDFrame(fi,extractor,segmentation));}
void Map3D::addFrame(RGBDFrame * frame){
	printf("Map3Dbase::addFrame(RGBDFrame * frame)\n");
	if(frames.size() > 0){transformations.push_back(matcher->getTransformation(frame, frames.back()));}
	frames.push_back(frame);
}
void Map3D::addTransformation(Transformation * transformation){}
void Map3D::estimate(){
	printf("estimate\n");
	sort(transformations.begin(),transformations.end(),comparison_Map3Dbase);
	printf("sorted\n");
	poses.push_back(Matrix4f::Identity());
	for(int i = 0; i < transformations.size(); i++){
		printf("id:%i ---------> %i <--> %i : %f\n",i,transformations.at(i)->src->id,transformations.at(i)->dst->id,transformations.at(i)->weight);
		poses.push_back(poses.back()*transformations.at(i)->transformationMatrix);
	}
	printf("estimate done\n");
}
void Map3D::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}
void Map3D::visualize(){}
void Map3D::showTuning(){}

bool comparison_Map3D (Transformation * i,Transformation * j) {
	if(i->src == j->src){return (i->weight<j->weight);}
	else{return (i->src->id<j->src->id);}
}

vector<int> Map3D::getLargestComponent(){
	sort(transformations.begin(),transformations.end(),comparison_Map3D);

	bool connections[frames.size()][frames.size()];
	bool taken[frames.size()];
	for(int i = 0; i < frames.size(); i++){
		taken[i]=false;
		for(int j = 0; j < frames.size(); j++){
			connections[i][j]=false;
		}
	}

	for(int i = 0; i < transformations.size(); i++){
		Transformation * transformation = transformations.at(i);
		connections[transformation->src->id][transformation->dst->id]=true;
		connections[transformation->dst->id][transformation->src->id]=true;
	}

	vector<vector<int> > all_matched;
	for(int i = 0; i < frames.size(); i++){
		if(!taken[i]){
			all_matched.push_back(vector<int>());
			vector<int> todo;
			todo.push_back(i);
			taken[i] = true;
			printf("Matched segment:"); 
			while(todo.size()>0){
				int id = todo.back();
				all_matched.back().push_back(id);
				printf(" %i",id);
				todo.pop_back();
				for(int j = 0; j < frames.size(); j++){
					if(!taken[j] && connections[id][j]){todo.push_back(j);taken[j] = true;}
				}
			}
			printf("\n");
		}
	}

	largest_component.clear();
	for(int i = 0; i < all_matched.size(); i++){
		if(all_matched.at(i).size() > largest_component.size()){largest_component = all_matched.at(i);}
	}
	return largest_component;
}

void Map3D::cleanTransformations(float threshold){
	printf("cleanTransformations\n");
	for(int i = 0; i < transformations.size();){
		if(transformations.at(i)->weight > threshold){
			i++;
		}else{
			delete transformations.at(i);
			transformations.at(i) = transformations.back();
			transformations.pop_back();
		}
	}

	vector< vector <Transformation *> > cycles;
	vector< float > values;

	vector<Transformation *> trans_arr[frames.size()];

	for(int i = 0; i < frames.size(); i++){trans_arr[i].clear();}

	for(int i = 0; i < transformations.size();i++){
		Transformation * t = transformations.at(i);
		trans_arr[t->src->id].push_back(t);
		trans_arr[t->dst->id].push_back(t);
	}
	printf("transformations.size(): %i\n",transformations.size());
	vector<Transformation *> trans[frames.size()];

	int depth = 5;
	for(int i = 0; i < transformations.size();i++){
		for(int i = 0; i < frames.size();i++){trans[i].clear();}

		Transformation * t = transformations.at(i);
		int start = t->src->id;
		int stop = t->dst->id;

		for(int l = 0; l < trans_arr[start].size(); l++){
			if(trans_arr[start].at(l) == t){
				trans_arr[start].at(l) = trans_arr[start].back();
				trans_arr[start].pop_back();
			}
		}

		for(int l = 0; l < trans_arr[stop].size(); l++){
			if(trans_arr[stop].at(l) == t){
				trans_arr[stop].at(l) = trans_arr[stop].back();
				trans_arr[stop].pop_back();
			}
		}

		trans[start].push_back(t);

		vector< int > todo_now;
		vector< int > todo_next;

		todo_now.push_back(start);
		for(int d = 0; d < depth; d++){
			for(int k = 0; k < todo_now.size(); k++){
				int kk = todo_now.at(k);
				for(int l = 0; l < trans_arr[kk].size(); l++){
					Transformation * t2 = trans_arr[kk].at(l);
					int dst_id = t2->dst->id;
					int src_id = t2->src->id;
					if(src_id == stop || dst_id == stop){
						cycles.push_back(vector< Transformation * >());
						for(int m = 0; m < trans[kk].size();m++){cycles.back().push_back(trans[kk].at(m));}
						cycles.back().push_back(t2);
					}else if(trans[src_id].size() == 0){
						for(int m = 0; m < trans[kk].size();m++){trans[src_id].push_back(trans[kk].at(m));}
						trans[src_id].push_back(t2);
						todo_next.push_back(src_id);
					}else if(trans[dst_id].size() == 0){
						for(int m = 0; m < trans[kk].size();m++){trans[dst_id].push_back(trans[kk].at(m));}
						trans[dst_id].push_back(t2);
						todo_next.push_back(dst_id);
					}
				}
			}
			todo_now = todo_next;
			todo_next.clear();
		}
	}
	
	vector<float> 			score;
	vector< vector<int> >	edges_index;
	vector< float >			edges_score;
	score.resize(transformations.size());
	map<Transformation *, int> m;

	float w_zero = 10.9;
	float w_scale = 5;
	for(int i = 0; i < transformations.size();i++){
		m.insert(std::pair<Transformation *, int>(transformations.at(i),i));
		float w = transformations.at(i)->weight;
		score.at(i) = w_scale*(w-w_zero);
		//if(score.at(i) < -5*w_scale){score.at(i) = -5*w_scale;}
	} 

	float e_zero = 0.005;
	float e_scale = 1;
	for(int i = 0; i < cycles.size(); i++){
		edges_index.push_back( vector<int>() );
		Transformation * current = cycles.at(i).at(0);
		int last_id = current->src->id;
		Eigen::Matrix4f diff = current->transformationMatrix;
		
		edges_index.back().push_back(m.find(current)->second);

		for(int j = 1; j < cycles.at(i).size(); j++){
			current = cycles.at(i).at(j);
			edges_index.back().push_back(m.find(current)->second);

			if(current->dst->id == last_id){
				diff *= current->transformationMatrix;
				last_id = current->src->id;
			}else{
				diff *= current->transformationMatrix.inverse();
				last_id = current->dst->id;
			}
		}

		float avg_error = sqrt(diff(0,3)*diff(0,3)+diff(1,3)*diff(1,3)+diff(2,3)*diff(2,3)) / float(cycles.at(i).size());
		float sc = e_scale*(e_zero-avg_error)/e_zero;
		if(sc < -10*e_scale){sc = -10*e_scale;}
		//cout<< "avg_error "<< avg_error << " sc: " << sc <<endl;

		edges_score.push_back(sc);
		for(int j = 0; j < edges_index.back().size(); j++){
			score.at(edges_index.back().at(j))+=sc;
		}
	}

	vector<int> 			vertex_ind;
	vector< vector<int> >	vertex_edges;

	vertex_ind.resize(score.size());
	vertex_edges.resize(score.size());
	for(int i = 0; i < vertex_ind.size(); i++){vertex_ind.at(i) = i;}

	for(int i = 0; i < edges_index.size(); i++){
		vector<int> ed = edges_index.at(i);
		for(int j = 0; j < ed.size();j++){
			vertex_edges.at(ed.at(j)).push_back(i);
		}
	}

	double worst_score = -1;
	int worst_ind;
	vector<int> to_remove;
	while(worst_score < 0 && vertex_ind.size() > 0){
		int w_ind = 0;
		worst_ind	= vertex_ind.at(w_ind);
		worst_score	= score.at(worst_ind);
		for(int i = 1; i < vertex_ind.size();i++){
			if(score.at(vertex_ind.at(i)) < worst_score){
				w_ind = i;
				worst_ind	= vertex_ind.at(i);
				worst_score	= score.at(vertex_ind.at(i));
			}
		}
		if(worst_score < 0){
			//printf("worst: %i -> %f\n",worst_ind,worst_score);
			//transformations.at(worst_ind)->print();
			to_remove.push_back(worst_ind);
	
			//For all edges remove scores

			for(int i = 0; i < vertex_edges.at(worst_ind).size();i++){
				int e_ind = vertex_edges.at(worst_ind).at(i);
				float sc = edges_score.at(e_ind);
				//printf("e_ind: %i sc: %f\n",e_ind,sc);
				edges_score.at(e_ind) = 0;//LAZY way of removing edge
				for(int j = 0; j < edges_index.at(e_ind).size(); j++){
					int vert = edges_index.at(e_ind).at(j);
					//printf("%i ",vert);
					score.at(vert) -= sc;
				}
				//printf("\n");
			}
			vertex_ind.at(w_ind) = vertex_ind.back();
			vertex_ind.pop_back();
		}
		//exit(0);
	}
	sort(to_remove.begin(),to_remove.end());
	printf("removesize: %i\n",to_remove.size());
	for(int i = to_remove.size()-1; i >=  0; i--){
		//printf("To remove: %i\n",to_remove.at(i));
		delete transformations.at(to_remove.at(i));
		transformations.at(to_remove.at(i)) = transformations.back();
		transformations.pop_back();
	} 

	//exit(0);
}

unsigned long Map3D::getCurrentTime(){
	struct timeval start;
	gettimeofday(&start, NULL);
	return start.tv_sec*1000000+start.tv_usec;
}

g2o::SE3Quat Map3D::getQuat(Eigen::Matrix4f mat){
	Eigen::Affine3f eigenTransform(mat);
	Eigen::Quaternionf eigenRotation(eigenTransform.rotation());
	g2o::SE3Quat poseSE3(Eigen::Quaterniond(eigenRotation.w(), eigenRotation.x(), eigenRotation.y(), eigenRotation.z()),Eigen::Vector3d(eigenTransform(0,3), eigenTransform(1,3), eigenTransform(2,3)));
	return poseSE3;
}

void Map3D::savePCD(string path){
	printf("Saving map in: %s\n",path.c_str());

  // Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setLeafSize (0.0075f, 0.0075f, 0.0075f);
	
	//pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2;
	
	printf("nr frames: %i \n nr poses: %i\n",frames.size(),poses.size());
	for(int i = 0; i < largest_component.size(); i++){
		//printf("largest_component.at(%i)=%i\n",i,largest_component.at(i));
		pcl::PointCloud<pcl::PointXYZRGB> c = frames.at(largest_component.at(i))->input->getCloud();
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
		cloud2->points.resize(c.points.size());
		float tmp_r = 255*float(rand()%1000)/1000.0f;
		float tmp_g = 255*float(rand()%1000)/1000.0f;
		float tmp_b = 255*float(rand()%1000)/1000.0f;
		for(int j = 0; j < c.points.size(); j++){
			cloud2->points[j].x = c.points[j].x;
			cloud2->points[j].y = c.points[j].y;
			cloud2->points[j].z = c.points[j].z;
			cloud2->points[j].r = c.points[j].r;
			cloud2->points[j].g = c.points[j].g;
			cloud2->points[j].b = c.points[j].b;
			//cloud2->points[j].r = tmp_r;
			//cloud2->points[j].g = tmp_g;
			//cloud2->points[j].b = tmp_b;
		}

		pcl::PointCloud<pcl::PointXYZRGB> c2;
		sor.setInputCloud (cloud2);
		sor.filter (c2);

		Eigen::Matrix4f mat = poses.at(largest_component.at(i));
		//cout << "pose" << i << "\n" << mat << "\n";
		pcl::PointCloud<pcl::PointXYZRGB> c_trans;
		pcl::transformPointCloud (c2, c_trans, mat);
		*cloud += c_trans;
	}

	pcl::PointCloud<pcl::PointXYZRGB> voxelcloud;
	sor.setInputCloud (cloud);
	sor.filter (voxelcloud);

	pcl::io::savePCDFileBinary (path, voxelcloud);
	std::cerr << "Saved " << voxelcloud.points.size () << " data points." << std::endl;
}
