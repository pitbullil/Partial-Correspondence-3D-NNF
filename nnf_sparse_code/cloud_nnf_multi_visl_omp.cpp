
#include <cloud_nnf_multi_visl_omp.hpp>

using namespace pcl;
using namespace std;
using namespace dis3;
using namespace pcl::features;

bool FeatureCloud::readCloud(string path, nnfDest d) {
	XYZCloud::Ptr Dest; NormalCloud::Ptr DestN; 
	switch (d) {
	case Q:   Dest = Qc; DestN = QN; break;
	case QKEY:Dest = Qkeypoints; break;
	case T:   Dest = Tc; DestN = TN; break;
	case TKEY:Dest = Tkeypoints; break;
	}
	if (readCloudFile<XYZ>(path, *Dest) == -1) {
		PCL_ERROR("Couldn't read Cloud");
		return false;
	}
	if ((d == Q) || (d == T)) {
		readCloudFile<Normal>(path, *DestN);
	}
	return true;
};

cloud_statistics_s FeatureCloud::CalculateStatistics(nnfDest d, string out) {
	XYZCloud::Ptr Dest;
	cloud_param_s* p;
	switch (d) {
	case Q:   Dest = Qc; p = &P.S;  break;
	case T:   Dest = Tc; p = &P.T; break;
	}
	findResolution(Dest, p->stat, out);
	if (p->k.modelResolution == 0) {
		p->k.modelResolution = p->stat.MeanResolution;
	}
	if (p->k.non_max_radius == 0) {
		p->k.non_max_radius = p->stat.MeanResolution*keypoints_visl::NON_MAX_RADIUS_MULTIPLIER;
	}
	if (p->k.salient_radius == 0) {
		p->k.salient_radius = p->stat.MeanResolution*keypoints_visl::RADIUS_SEARCH_MULTIPLIER;
	}
	if (p->s.cloudResolution == 0) {
		p->s.cloudResolution = p->stat.MeanResolution;
	}

	return p->stat;
}

void FeatureCloud::computeNormal(nnfDest d, bool force) {
	XYZCloud::Ptr Dest; NormalCloud::Ptr DestN; PolygonMeshPtr mesh;  cloud_param_s p;
	vector<Vertices> polygons;
	switch (d) {
	case Q:   Dest = Qc; polygons = this->Qpolygons; DestN = QN; p = P.S;  break;
	case T:   Dest = Tc; polygons = this->Tpolygons; DestN = TN; p = P.T; break;
	}
	if (Dest->size() != DestN->size()) { force = true; }
	if (force) {
		if (P.normal_by_nn) {
			cout << "normal by nn \n";
			computeNormals(Dest, DestN, 10, true);
		}
		else if (P.mesh_normals) {
			cout << "normal by mesh \n";
			computeApproximateMeshNormals(*Dest, polygons, *DestN);
			//computeApproximateNormals(*Dest, polygons, *DestN);
			pcl::io::savePCDFile(p.f.path + "normals.pcd", *DestN);
			XYZNCloud c;
			pcl::concatenateFields(*Dest, *DestN, c);
			cout << "size of c:" << c.size() << endl;
			cout << (p.path + "\\meshnormals.ply");
			pcl::io::savePLYFileASCII(p.path + "\\meshnormals.ply", c);
		}
		else {
			cout << "normal by radius \n";
			computeNormals(Dest, DestN, p.f.searchRadius/2, true);
		}
	}
}

void HKSFeature::computeF(nnfDest d) {
	string path;
	switch (d) {
	case Q:path = P.S.path + "\\HKS.pcd"; break;
	case T:path = P.T.path + "\\HKS.pcd"; break;
	}
	loadF(d, path);
};

void SIHKSFeature::computeF(nnfDest d) {
	string path;
	switch (d) {
	case Q:path = P.S.path + "\\SIHKS.pcd"; break;
	case T:path = P.T.path + "\\SIHKS.pcd"; break;
	}
	loadF(d, path);
};

void LBEIGSFeature::computeF(nnfDest d) {
	string path;
	switch (d) {
	case Q:path = P.S.path + "\\LB.pcd"; break;
	case T:path = P.T.path + "\\LB.pcd"; break;
	}
	loadF(d, path);
};

void SCHKSFeature::computeF(nnfDest d) {
	string path;
	switch (d) {
	case Q:path = P.S.path + "\\SCHKS.pcd"; break;
	case T:path = P.T.path + "\\SCHKS.pcd"; break;
	}
	loadF(d, path);
};

void FPFHFeature::computeF(nnfDest d) {
	switch (d) {
	case Q:computeFPFHFeatures(Qc, Qkeypoints, QN, QF, P.S.f.searchRadius); break;
	case T:computeFPFHFeatures(Tc, Tkeypoints, TN, TF, P.T.f.searchRadius); break;
	}
};

void SFPFHFeature::computeF(nnfDest d) {
	switch (d) {
	//case Q:computeSFPFHFeatures(Qc, Qkeypoints, QN, QF, &DQ, P.S.f.searchRadius); break;
	//case T:computeSFPFHFeatures(Tc, Tkeypoints, TN, TF, &DT, P.T.f.searchRadius); break;
	}
};

void GFPFHFeature::computeF(nnfDest d) {
	switch (d) {
	//case Q:computeGFPFHFeatures(Qc, Qkeypoints, QN, QF, &DQ, P.S.f.searchRadius); break;
	//case T:computeGFPFHFeatures(Tc, Tkeypoints, TN, TF, &DT, P.T.f.searchRadius); break;
	}
};

void GPFHFeature::computeF(nnfDest d) {
	switch (d) {
	//case Q:computeGPFHFeatures(Qc, Qkeypoints, QN, QF, &DQ, P.S.f.searchRadius); break;
	//case T:computeGPFHFeatures(Tc, Tkeypoints, TN, TF, &DT, P.T.f.searchRadius); break;
	}
};

void PFHFeature::computeF(nnfDest d) {
	switch (d) {
	case Q:computePFHFeatures(Qc, Qkeypoints, QN, QF, P.S.f.searchRadius); break;
	case T:computePFHFeatures(Tc, Tkeypoints, TN, TF, P.T.f.searchRadius); break;
	}
};

void ROPSFeature::computeF(nnfDest d) {
	XYZCloud::Ptr cloud; XYZCloud::Ptr keypoints; NormalCloud::Ptr Normals; vector<pcl::Vertices>* polygons; graphMesh * graph;
	XYZCloud::Ptr gridCloud; ROPSCloud* Feat;
	feature_params_s* p;
	switch (d) {
	case Q:   cloud = Qc; keypoints = Qkeypoints; polygons = &Qpolygons; graph = &Q_G; gridCloud = QGridCloud; Feat = &QF; p = &P.S.f; break;
	case T:   cloud = Tc; keypoints= Tkeypoints; polygons = &Tpolygons; graph = &T_G; gridCloud = TGridCloud; Feat = &TF;  p = &P.T.f; break;
	}
	if (polygons->empty()) {
		Reconstruct_Surface(d);
	}
	if ((io::loadPCDFile(p->path, *Feat) == -1) || (Feat->size() != keypoints->size())) {
		computeROPSFeatures(cloud, keypoints, *polygons, *Feat, p->searchRadius);
	};
};

void SHOTFeature::computeF(nnfDest d) {
	switch (d) {
	case Q:computeSHOTFeatures(Qc, Qkeypoints, QN, QF, P.S.f.searchRadius); break;
	case T:computeSHOTFeatures(Tc, Tkeypoints, TN, TF, P.T.f.searchRadius); break;
	}
};

void FeatureCloud::loadNNFField() {
	io::loadPCDFile(P.nnf_dir+"\\nnf.pcd", *nnf1);
}

void FeatureCloud::LoadFocal(string file) {
	load_focal(T_f, file);
	P.focal_ind = T_f.ind;
	P.R_G = T_f.RG;
}
	

void FeatureCloud::Reconstruct_Surface(nnfDest d) {
	XYZCloud::Ptr cloud; NormalCloud::Ptr Normals; vector<pcl::Vertices>* polygons; graphMesh * graph; XYZCloud::Ptr gridCloud; cloud_statistics_s* St;
	XYZNCloud::Ptr out;
	cloud_param_s *p;
	switch (d) {
	case Q:   cloud = Qc; Normals = QN; polygons = &Qpolygons; graph = &Q_G; gridCloud = QGridCloud; p = &P.S; break;
	case T:   cloud = Tc; Normals = TN; polygons = &Tpolygons; graph = &T_G; gridCloud = TGridCloud; p = &P.T; break;
	}
	if (p->s.Method == NONE) { return; }
	if (p->s.Method == LOAD) { LoadGridMesh(p->in, d); return;};

	PolygonMeshPtr S(new PolygonMesh());
	if (!(p->s.save && (io::loadPLYFile(p->s.out_path+"surface.ply", *S) != -1))) {
		if (Normals->size() != cloud->size()) {
			computeNormal(d,true);
		}
		S = ReconstructSurface(cloud, Normals, p->s);
	}
	if (p->s.base) {
		fromPCLPointCloud2(S->cloud, *cloud);
		*polygons = S->polygons;
		if ((p->s.normals)&&(!(p->s.save&&(io::loadPCDFile(p->s.out_path+"normals.pcd",*Normals)!=-1)))){
			computeApproximateNormals(*cloud, *polygons, *Normals);
		}
	}

	if (p->s.grid) {
		graph->mesh2graph(S,P.meshtype == SUPER);
		//if (P.meshtype != NORMAL) { 
		//	copyPointCloud(*graph->dualcloud(), *gridCloud); 
		//}else{
			fromPCLPointCloud2(S->cloud, *gridCloud);
		//}
		if (d == T) {
			if (P.load_d_t) {
				cout << 1;
				//DT = load_csv<Eigen::MatrixXf>(P.T.path + "\\D_"+P.geotype+".csv");
			}
			else {
				DT = createdistancematrixOMP(S, P.threads);
				/*T_f = graph->graphFocal(P.meshtype != NORMAL, true);
				DT = T_f.D;
				T_f.D.resize(1,1);
				vector<int> dist = graph->LimitedSSSP(T_f.ind, INFINITY, P.meshtype != NORMAL);
				int j = 0;
				while (T_f.ind > S->cloud.width - 1) {
					T_f.ind = dist[++j];
				}
				//T_distances.at(T_f.ind) = T_f.D.row(T_f.ind);
				//cout << "size of template distances:" << T_distances.at(T_f.ind).size() << endl;
				dist = graph->LimitedSSSP(T_f.ind, INFINITY, P.meshtype != NORMAL);
				T_f.RG = graph->getVertDist(dist.back(), P.meshtype == SUPER);
				T_f.path = graph->getPath(dist.back(), P.meshtype == SUPER);
				graph->resetDist(P.meshtype == SUPER);
				TGkdtree.setInputCloud(gridCloud);
				PointIndices::Ptr scene_indices(new pcl::PointIndices()); vector<float> sceneDistances;
				TGkdtree.nearestKSearch(gridCloud->at(T_f.ind), gridCloud->size(), scene_indices->indices, sceneDistances);
				T_f.RE = sqrt(sceneDistances.back());
				P.R_E = T_f.RE;
				T_G.resetDist(P.meshtype != NORMAL);
				scene_indices->indices = T_G.LimitedSSSP(T_f.ind, INFINITY, P.meshtype != NORMAL);
				T_f.RG = T_G.getVertDist(scene_indices->indices.back(), P.meshtype != NORMAL);
				P.R_G = T_f.RG;*/
			}
			//TRVec.resize(DT.rows());*/
		}
		else {
			cloud_statistics_s temp;
			findResolution(gridCloud, temp);
			grid_max_resolution = temp.MaxResolution;
		}
	}

	if (p->s.save) {
		savePolygonMeshToFile(S, p->s.out_path + "surface.ply");
		if (p->s.normals) { io::savePCDFile(p->s.out_path + "normals.pcd", *Normals); }
	}
}

void FeatureCloud::LoadGridMesh(string path, nnfDest d) {
	XYZCloud::Ptr cloud; NormalCloud::Ptr Normals; vector<pcl::Vertices>* polygons; graphMesh * graph; XYZCloud::Ptr gridCloud; cloud_statistics_s* St; vector<vector<float>>* D;
	XYZNCloud::Ptr out;
	PolygonMeshPtr S;
	cloud_param_s *p;
	switch (d) {
	case Q:   S = Qmesh; cloud = Qc; Normals = QN; polygons = &Qpolygons; graph = &Q_G; gridCloud = QGridCloud; p = &P.S; D = &DQ; break;
	case T:   S = Tmesh;   cloud = Tc; Normals = TN; polygons = &Tpolygons; graph = &T_G; gridCloud = TGridCloud; p=&P.T; D = &DT; break;
	case QSURF:  S=PolygonMeshPtr(new PolygonMesh());  cloud = QGridCloud; polygons = &Qpolygons; graph = &Q_G; gridCloud = QGridCloud; p = &P.S; D = &DQ; break;
	case TSURF:  S = PolygonMeshPtr(new PolygonMesh()); cloud = TGridCloud; polygons = &Tpolygons; graph = &T_G; gridCloud = TGridCloud; p = &P.T; D = &DT; break;
	}
	if (S->polygons.empty()) {
		io::loadPLYFile(path, *S);
	}
	if (p->s.base) {
		fromPCLPointCloud2(S->cloud, *cloud);
		*polygons = S->polygons;
		if (Normals->size() != cloud->size()) {
			computeNormal(d, true);
			
		}
	}
	if (d == QSURF || d == TSURF || p->s.grid) {
		graph->mesh2graph(S, P.meshtype == SUPER);
		fromPCLPointCloud2(S->cloud, *gridCloud);

		if ((P.load_d_t && (d == TSURF || d == T)) || (P.load_d_q && (d == QSURF|| d == Q))) {
			//D = new Eigen::MatrixXf(gridCloud->size(), gridCloud->size());
			//*D = load_csv<Eigen::MatrixXf>(p->path + "\\D_" + P.geotype + ".csv");
			cout << 1;
		} else {
			*D = createdistancematrixOMP(S, P.threads);
		//if (P.meshtype != NORMAL) {
			//copyPointCloud(*graph->dualcloud(), *gridCloud);
		//}else {
		//}
		/*else if (d == TSURF||d == T) {
			if (P.focal_ind == MAXUINT32) {
				T_f = graph->graphFocal(P.meshtype != NORMAL,true);
			}
			vector<int> dist = graph->LimitedSSSP(T_f.ind, INFINITY, P.meshtype != NORMAL);
			int j = 0;
			while (T_f.ind > S->cloud.width - 1) {
				T_f.ind = dist[++j];
			}
			//T_distances.resize(S->cloud.width);
			//T_distances.at(T_f.ind) = T_f.D.at(T_f.ind);
			//cout << "T_distances size:" << T_distances.at(T_f.ind).size() << endl;
			graph->resetDist(P.meshtype == SUPER);
			dist = graph->LimitedSSSP(T_f.ind, INFINITY, P.meshtype != NORMAL);
			T_f.RG = graph->getVertDist(dist.back(), P.meshtype != NORMAL);
			T_f.path = graph->getPath(dist.back(), P.meshtype != NORMAL);
			graph->resetDist(P.meshtype != NORMAL);

			XYZCloud::Ptr temp = P.meshtype != NORMAL ? graph->dualcloud() : gridCloud;
			TGkdtree.setInputCloud(temp);
			Eigen::Vector4f farthest_pointq;
			pcl::getMaxDistance(*gridCloud, temp->at(T_f.ind).getVector4fMap(), farthest_pointq);
			T_f.RE = (temp->at(T_f.ind).getVector4fMap() - farthest_pointq).norm();
			P.R_E = T_f.RE;
			vector<int> d = graph->LimitedSSSP(T_f.ind, INFINITY, P.meshtype != NORMAL);
			T_f.path = graph->getPath(d.back(), P.meshtype != NORMAL);
			graph->resetDist(P.meshtype != NORMAL);
			PointIndices::Ptr scene_indices(new pcl::PointIndices());
			//scene_indices->indices = T_G.LimitedSSSP(T_f.ind,INFINITY, P.meshtype != NORMAL);
			//T_f.RG = T_G.getVertDist(scene_indices->indices.back());
			P.R_G = T_f.RG;
		} else {
			cloud_statistics_s temp;
			findResolution(gridCloud, temp);
			grid_max_resolution = temp.MaxResolution;*/
		}
	}
}

void FeatureCloud::LoadSurface(string path, nnfDest d) {
	switch (d) {
	case QSURF:   Q_G = graphMesh(path); break;
	case TSURF:   T_G = graphMesh(path); break;
	}

}

void FeatureCloud::LoadMesh(nnfDest d) {
	switch (d) {
	case T: pcl::io::loadPLYFile(P.T.in, *Tmesh); break;
	case Q: pcl::io::loadPLYFile(P.S.in, *Qmesh); break;
	}
};

void FeatureCloud::LoadTFocal(string path) {
	load_focal(T_f, path);
	T_G.resetDist(P.meshtype != NORMAL);
	T_G.LimitedSSSP(T_f.ind,INFINITY, P.meshtype != NORMAL);
}

void FeatureCloud::calcTFocal() {
	T_f = T_G.graphFocal(P.meshtype != NORMAL,true);
	//T_distances.at(T_f.ind) = T_f.D.at(T_f.ind);
	T_G.resetDist(P.meshtype != NORMAL);
	T_G.LimitedSSSP(T_f.ind, INFINITY, P.meshtype != NORMAL);
}

void FeatureCloud::init_sim_structs() {

	num_samples = feature_point_inds.size();
	FsimilarityCloud.resize(num_samples);
	feature_point_map.resize(num_samples);
	MSFsimilarityCloud.resize(num_samples);
	LSFsimilarityCloud.resize(num_samples);
	feature_point_map.resize(num_samples);
	Fsimilarity.resize(num_samples);
	MSFsimilarity.resize(num_samples);
	LSFsimilarity.resize(num_samples);
	model_vertices = Qc->size();
	part_vertices = Tc->size();
	uint32_t thread_load = model_vertices / P.threads;
	for (int t = 0; t < P.threads; t++) {
		thread_start.push_back(t*thread_load);
		thread_end.push_back((t + 1)*thread_load);
	}
	thread_end.at(P.threads - 1) = model_vertices;
	scene_P_D_t.assign(P.threads, vector<ind_r_s>(model_vertices, ind_r_s(MAXUINT32, INFINITY)));
	closest_temp_t.assign(P.threads, vector<uint32_t>(part_vertices, MAXUINT32));
	r_j_t.assign(P.threads, vector<float>(model_vertices, INFINITY));
	Kappa_t.assign(P.threads, vector<uint32_t>(part_vertices, 0));
	Kappa_j_t.assign(P.threads, vector<uint32_t>(model_vertices, 0));
	Patch_Buddies_t.assign(P.threads, vector<uint32_t>(model_vertices, MAXUINT32));
	mindistances_t.assign(P.threads, vector<float>(part_vertices, INFINITY));//distance difference of the minimal corresponding point

	Gmax.resize(part_vertices);
	Mmax.resize(part_vertices);
	Lmax.resize(part_vertices);

	FsimilarityCloudColor.resize(feature_point_inds.size());
	Feature_Patches.resize(feature_point_inds.size());
	TDistances.resize(feature_point_inds.size());
	R_thresh = P.r_thresh * P.S.diam / 100;
	for (uint32_t i = 0; i < feature_point_inds.size(); i++) {
		FsimilarityCloud.at(i) = XYZICloud::Ptr(new XYZICloud);
		FsimilarityCloudColor.at(i) = XYZRGBCloud::Ptr(new XYZRGBCloud);
		copyPointCloud(*Qc, *FsimilarityCloud.at(i));
		Fsimilarity.at(i).resize(model_vertices);

		if (P.similarity == MSWDIS || P.similarity == TRIDIS || P.similarity == TRIDISP) {
			MSFsimilarityCloud.at(i) = XYZICloud::Ptr(new XYZICloud);
			copyPointCloud(*Qc, *MSFsimilarityCloud.at(i));
			MSFsimilarity.at(i).resize(model_vertices);
		}

		if (P.similarity == TRIDIS || P.similarity == TRIDISP) {
			LSFsimilarityCloud.at(i) = XYZICloud::Ptr(new XYZICloud);
			LSFsimilarity.at(i).resize(model_vertices);
			copyPointCloud(*Qc, *LSFsimilarityCloud.at(i));
		}
	}
	if (P.save_feature_patches) {
		for (uint32_t i = 0; i < feature_point_inds.size(); i++) {
			Feature_Patches.at(i) = PolygonMeshPtr(new PolygonMesh);
		}
	}
}

void FeatureCloud::setdensesample() {
	feature_point_inds.clear();
	for (int i = 0; i < Tc->size(); i++) {
		feature_point_inds.push_back(i);
	}
	init_sim_structs();
}

/*void FeatureCloud::sim_to_nnf(vector<vector<ind_dis_s>> corrs, vector<uint32_t> finds) {
	uint32_t rows = finds.size(), cols = corrs.at(finds.at(0)).size();
	for (int i = 0; i < rows; i++) {
		uint32_t ind = finds.at(i);
		//nnf->at(ind).label = corrs.at(ind).at(0).first;
		for (int j = 0; j <1; j++) {
			nnf1->at(ind).labels[j] = corrs.at(ind).at(j).first;
		}
	}
}*/

void FeatureCloud::sim_to_nnf(vector<vector<ind_dis_s>> corrs, vector<uint32_t> finds) {
	uint32_t rows = finds.size(), cols = corrs.at(finds.at(0)).size();
	for (int i = 0; i < rows; i++) {
		uint32_t ind = finds.at(i);
		//nnf->at(ind).label = corrs.at(ind).at(0).first;
		for (int j = 0; j <1; j++) {
			nnf1->at(ind).labels[j] = corrs.at(ind).at(j).first;
		}
	}
}

void FeatureCloud::distance_based_filter(float mean_thresh, float d_thresh) {
	bad_inds.clear();
	good_inds.clear();
	vector<vector<float>> Rho(num_samples, vector<float>(num_samples, 0)), dD(num_samples, vector<float>(num_samples, 0));
	vector<float> Rhom(num_samples, 0);
	vector<bool> non_exclusive(num_samples, false);
	vector<int> metamer(num_samples, 0);
	vector<bool> marked(num_samples, false);
	vector<bool> bad_inds_i(num_samples, false);
	float dt, dq; uint32_t src1,src2, map1,map2;
	for (int i = 0; i < num_samples - 1; i++) {
		src1 = feature_point_inds.at(i); map1 = feature_point_map.at(i);
		for (int j = i + 1; j < num_samples; j++) {
			src2 = feature_point_inds.at(j); map2 = feature_point_map.at(j);
			
			dt = DT.at(src1).at(src2) / P.S.diam;
			dq = DQ.at(map1).at(map2) / P.S.diam;
			dD.at(i).at(j) = abs(dt - dq); dD.at(j).at(i) = dD.at(i).at(j);
			if ((dD.at(i).at(j) > d_thresh) && !(map1 == map2)) {
				Rho.at(i).at(j) = max(dt / dq, dq / dt); 
			}
			else 
			{
				Rho.at(i).at(j)  = 1; 
			}
			if (map1 == map2) {
				non_exclusive.at(i) = true; non_exclusive.at(j) = true; //bad_inds_i.at(i) = true; bad_inds_i.at(j) = true;
				metamer.at(i) = j; metamer.at(j) = i;
				//bad_inds.push_back(i); bad_inds.push_back(j);
				//Rho.at(i).at(j) = 1 + 2 * mean_thresh;
			}
			Rho.at(j).at(i) = Rho.at(i).at(j);
			Rhom.at(i) += Rho.at(i).at(j); 	Rhom.at(j) += Rho.at(i).at(j);
		}
	}

	for (int i = 0; i < num_samples; i++) {
		if (non_exclusive.at(i)) {
			if (Rhom.at(i) > Rhom.at(metamer.at(i)))  bad_inds_i.at(i) = true;
		}
	}

	float Rhosum = 0; uint32_t m = 0;
	for (int i = 0; i < num_samples - 1; i++) {
		Rhom.at(i) = 0;
		if (!(bad_inds_i.at(i))) {
			m++;
			for (int j = 0; j < num_samples; j++) {
				if (!(bad_inds_i.at(j))) Rhom.at(i) += Rho.at(i).at(j);
			}
			Rhosum += Rhom.at(i);
		}
	}

	float meanRho = Rhosum / (m - 1);

	for (int i = 0; i < num_samples; i++) {
		if (bad_inds_i.at(i) || Rhom.at(i) > mean_thresh*meanRho) bad_inds.push_back(i); 
		else good_inds.push_back(i);
	}

}

vector<vector<ind_dis_s>> FeatureCloud::greedy_opt(float d_thresh, bool& change) {
	vector<vector<uint32_t>> llcandidates(num_samples, vector<uint32_t>(gopt_hl_cands*gopt_ml_cands*gopt_ll_cands, MAXUINT32));
	vector<vector<uint32_t>> mlcandidates(num_samples, vector<uint32_t>(gopt_hl_cands*gopt_ml_cands, MAXUINT32));
	vector<vector<uint32_t>> hlcandidates(num_samples, vector<uint32_t>(gopt_hl_cands, MAXUINT32));
	uint32_t num_bad = bad_inds.size(), ms_num, ls_num, ms_max_i, ls_max_i;
	vector<uint32_t> h_mark_inds(model_vertices, MAXUINT32);
	vector<uint32_t> m_mark_inds(model_vertices, MAXUINT32);
	vector<uint32_t> s_mark_inds(model_vertices, MAXUINT32);
	vector<float> hl_distances(model_vertices, INFINITY);
	vector<float> ml_distances(model_vertices, INFINITY);
	vector<float> ll_distances(model_vertices, INFINITY);
	float ms_max, ls_max;
	vector<float> hl_sim(model_vertices, INFINITY);
	vector<float> ml_sim(model_vertices, INFINITY);
	vector<float> ll_sim(model_vertices, INFINITY);

	//////////////////////////collection of candidates//////////////////////////////
	for (int i = 0; i < num_bad; i++) {
		uint32_t h_cand = 0, m_cand = 0, l_cand = 0;
		h_mark_inds.assign(model_vertices, false);
		m_mark_inds.assign(model_vertices, false);
		s_mark_inds.assign(model_vertices, false);

		hl_sim.assign(Fsimilarity.at(bad_inds.at(i)).begin(), Fsimilarity.at(bad_inds.at(i)).end());
		ml_sim.assign(MSFsimilarity.at(bad_inds.at(i)).begin(), MSFsimilarity.at(bad_inds.at(i)).end());
		ll_sim.assign(LSFsimilarity.at(bad_inds.at(i)).begin(), LSFsimilarity.at(bad_inds.at(i)).end());
		for (int j = 0; j < gopt_hl_cands; j++) { //running on High level similarity	
			uint32_t max_h = distance(hl_sim.begin(), max_element(hl_sim.begin(), hl_sim.end()));
			hlcandidates.at(i).at(j) = max_h;
			hl_distances.assign(DQ.at(max_h).begin(), DQ.at(max_h).end());//collecting a patch around the maxima
			for (int l = 0; l < gopt_ml_cands; l++) {
				ms_max = 0; ls_max = 0; ms_max_i = 0; ls_max_i = 0; ms_num = 0;
				for (int d = 0; d < model_vertices; d++) {
					if (hl_distances.at(d) < R_thresh_ms) {
						m_mark_inds.at(ms_num++) = d;
						if (ml_sim.at(d) > ms_max) {
							ms_max = ml_sim.at(d); ms_max_i = d;
						}
						if (hl_distances.at(d) < 0.2*P.S.diam) {//blocking the same maxima from being chosen again
							hl_sim.at(d) = 0;
						}
					}
				}
				mlcandidates.at(i).at(m_cand++) = ms_max_i;
				ml_distances.assign(DQ.at(ms_max_i).begin(), DQ.at(ms_max_i).end());//collecting a patch around the maxima at medium scale
				ls_num = 0;
				for (int d = 0; d < ms_num; d++) {
					if (ml_distances.at(m_mark_inds.at(d)) < R_thresh_ls) {
						s_mark_inds.at(ls_num++) = d;
						if (ll_sim.at(m_mark_inds.at(d)) > ls_max) {
							ls_max = ll_sim.at(m_mark_inds.at(d)); ls_max_i = m_mark_inds.at(d);
						}
						if (ml_distances.at(m_mark_inds.at(d)) < 0.1*P.S.diam) {//blocking the same maxima from being chosen again
							ml_sim.at(m_mark_inds.at(d)) = 0;
						}
					}
				}

				llcandidates.at(i).at(l_cand++) = ls_max_i;
				for (int k = 1; k < gopt_ll_cands; k++) {
					ls_max = 0;
					ll_distances.assign(DQ.at(ls_max_i).begin(), DQ.at(ls_max_i).end());//collecting a patch around the maxima

					for (int d = 0; d < ls_num; d++) {
						if (ll_distances.at(s_mark_inds.at(d)) < 0.03*P.S.diam) {
							ll_sim.at(s_mark_inds.at(d)) = 0;
						}
						else if (ll_sim.at(s_mark_inds.at(d)) > ls_max) {
							ls_max = ll_sim.at(s_mark_inds.at(d)); ls_max_i = s_mark_inds.at(d);
						}
					}
					llcandidates.at(i).at(l_cand++) = ls_max_i;
				}

				ll_distances.assign(DQ.at(ls_max_i).begin(), DQ.at(ls_max_i).end());//blocking the same maxima from being chosen again

				for (int d = 0; d < ls_num; d++) {
					if (ll_distances.at(s_mark_inds.at(d)) < 0.03*P.S.diam) {
						ll_sim.at(s_mark_inds.at(d)) = 0;
					}
				}
			}
		}
	}//////////////////////////end collection of candidates//////////////////////////////
	vector<vector<uint32_t>> all_candidates;
	all_candidates.resize(num_bad);
	for (int i = 0; i < num_bad; i++) {
		if (gopt_use_l) {
			all_candidates.at(i).insert(all_candidates.at(i).end(), llcandidates.at(i).begin(), llcandidates.at(i).end());
		}
		if (gopt_use_m) {
			all_candidates.at(i).insert(all_candidates.at(i).end(), mlcandidates.at(i).begin(), mlcandidates.at(i).end());
		}
		if (gopt_use_h) {
			all_candidates.at(i).insert(all_candidates.at(i).end(), hlcandidates.at(i).begin(), hlcandidates.at(i).end());
		}

	}

	uint32_t changes = true;
	uint32_t src, cand, map;
	float best_score = INFINITY;
	float dc = 0, dD, dobj, dorig;
	//initial pass without bad indices
	for (int i = 0; i < num_bad; i++) {
		src = feature_point_inds.at(bad_inds.at(i));
		best_score = INFINITY;
		for (int c = 0; c < gopt_hl_cands*gopt_ll_cands*gopt_ml_cands; c++) {
			cand = all_candidates.at(i).at(c);
			dc = 0;
			for (int j = 0; j < good_inds.size(); j++) {//creating distortion score
				dobj = DQ.at(cand).at(feature_point_map.at(good_inds.at(j)));
				dorig = DT.at(src).at(feature_point_inds.at(good_inds.at(j)));
				dD = abs(dobj - dorig) / P.S.diam;
				if (dD < d_thresh) dc += 1; else dc += max(dobj / dorig, dorig / dobj);
			}
			if (dc < best_score) { best_score = dc; feature_point_map.at(bad_inds.at(i)) = cand; }
		}
	}

	uint32_t old_res;
	while (changes) {
		changes = false;
		for (int i = 0; i < num_bad; i++) {
			src = feature_point_inds.at(bad_inds.at(i));
			old_res = feature_point_map.at(bad_inds.at(i));
			best_score = INFINITY;
			for (int c = 0; c < gopt_hl_cands*gopt_ll_cands; c++) {
				cand = llcandidates.at(i).at(c);
				dc = 0;
				for (int j = 0; j < num_samples; j++) {//creating distortion score
					if (j != bad_inds.at(i)) {
						dobj = DQ.at(cand).at(feature_point_map.at(j));
						dorig = DT.at(src).at(feature_point_inds.at(j));
						dD = abs(dobj - dorig) / P.S.diam;
						if (dD < 0.01) dc += 1; else dc += max(dobj / dorig, dorig / dobj);
					}
				}
				if (dc < best_score) { best_score = dc; feature_point_map.at(bad_inds.at(i)) = cand; }
			}
		}
	}

	for (int i = 0; i < num_bad; i++) {
		if (Result.at(feature_point_inds.at(bad_inds.at(i))).at(0).first != feature_point_map.at(bad_inds.at(i))) change = true;
		Result.at(feature_point_inds.at(bad_inds.at(i))).at(0).first = feature_point_map.at(bad_inds.at(i));
	}

	return Result;
};

void FeatureCloud::find_sample_points() {
	vector<float> F = DistSumFunctional(DT);
	vector<vector<uint32_t>> neighs = list_neighbors(Tmesh->polygons, Tc->size());
	feature_point_inds = meshMaximas(F, neighs);
	feature_point_inds = geodesicMaximasupression(F, feature_point_inds, DT, P.S.diam, 0.05);

	boundary_label_s BoundIndices = MeshBoundary(Tmesh->polygons, Tc->size(), true);
	sort(BoundIndices.ind.begin(), BoundIndices.ind.end());
	vector<int>::iterator bound = BoundIndices.ind.begin();
	vector<uint32_t> non_boundary_seeds;
	for (int i = 0; i < feature_point_inds.size(); i++) {
		uint32_t ind = feature_point_inds.at(i);
		while (bound < BoundIndices.ind.end()) {
			if (*bound == ind || *bound>ind) break;
			bound++;
		}
		if ((bound == BoundIndices.ind.end()) || (*bound != ind)) non_boundary_seeds.push_back(ind);
	}
	if (!non_boundary_seeds.empty()) feature_point_inds = non_boundary_seeds;
	surface_sampling(feature_point_inds, DT, P.S.diam, 0.05);
	init_sim_structs();
}

void FeatureCloud::LoadFeatureInds(string path="") {
	//if (path == "") path = P.T.path + "\\F_fast.csv";
	if (path == "") path = P.T.path + "\\F_max_no_mds.csv";
	feature_point_inds = load_csv_to_vector<uint32_t>(path);
	num_samples = feature_point_inds.size();
	FsimilarityCloud.resize(num_samples);
	feature_point_map.resize(num_samples);
	MSFsimilarityCloud.resize(num_samples);
	LSFsimilarityCloud.resize(num_samples);
	feature_point_map.resize(num_samples);
	Fsimilarity.resize(num_samples);
	MSFsimilarity.resize(num_samples);
	LSFsimilarity.resize(num_samples);
	model_vertices = Qc->size(); 
	part_vertices = Tc->size(); 
	uint32_t thread_load = model_vertices / P.threads;
	for (int t = 0; t < P.threads; t++) {
		thread_start.push_back(t*thread_load);
		thread_end.push_back((t + 1)*thread_load);
	}
	thread_end.at(P.threads - 1) = model_vertices;
	scene_P_D_t.assign(P.threads,vector<ind_r_s>(model_vertices, ind_r_s(MAXUINT32, INFINITY)));
	closest_temp_t.assign(P.threads, vector<uint32_t>(part_vertices, MAXUINT32));
	r_j_t.assign(P.threads, vector<float>(model_vertices, INFINITY));
	Kappa_t.assign(P.threads, vector<uint32_t>(part_vertices, 0));
	Kappa_j_t.assign(P.threads, vector<uint32_t>(model_vertices, 0));
	Patch_Buddies_t.assign(P.threads, vector<uint32_t>(model_vertices, MAXUINT32));
	mindistances_t.assign(P.threads, vector<float>(part_vertices, INFINITY));//distance difference of the minimal corresponding point

	Gmax.resize(part_vertices);
	Mmax.resize(part_vertices);
	Lmax.resize(part_vertices);

	FsimilarityCloudColor.resize(feature_point_inds.size());
	Feature_Patches.resize(feature_point_inds.size());
	TDistances.resize(feature_point_inds.size());
	R_thresh = P.r_thresh * P.S.diam / 100;
	if (P.save_similarity_clouds || P.similarity == MSWDIS || P.similarity == TRIDIS || P.similarity == TRIDISP) {
		for (uint32_t i = 0; i < feature_point_inds.size(); i++) {
				FsimilarityCloud.at(i) = XYZICloud::Ptr(new XYZICloud);
				FsimilarityCloudColor.at(i) = XYZRGBCloud::Ptr(new XYZRGBCloud);
				copyPointCloud(*Qc, *FsimilarityCloud.at(i));
				Fsimilarity.at(i).resize(model_vertices);

			if (P.similarity == MSWDIS|| P.similarity == TRIDIS|| P.similarity == TRIDISP) {
				MSFsimilarityCloud.at(i) = XYZICloud::Ptr(new XYZICloud);
				copyPointCloud(*Qc, *MSFsimilarityCloud.at(i));
				MSFsimilarity.at(i).resize(model_vertices);
			}

			if (P.similarity == TRIDIS || P.similarity == TRIDISP) {
				LSFsimilarityCloud.at(i) = XYZICloud::Ptr(new XYZICloud);
				LSFsimilarity.at(i).resize(model_vertices);
				copyPointCloud(*Qc, *LSFsimilarityCloud.at(i));
			}


		}
	}
	if (P.save_feature_patches) {
		for (uint32_t i = 0; i < feature_point_inds.size(); i++) {
			Feature_Patches.at(i) = PolygonMeshPtr(new PolygonMesh);
		}
	}

}

void FeatureCloud::saveTFocal(string path) {
	save_focal(T_f, path);
}

void FeatureCloud::extractTemplateBoundaryInds() {
	boundary_label_s BoundIndices = MeshBoundary(Tmesh->polygons, part_vertices,true);
	T_Boundary.resize(part_vertices, false);
	T_BoundaryInds = BoundIndices.ind;
	
	for (int i = 1; i < T_BoundaryInds.size(); i++) {
		TBoundary->push_back(Tc->at(T_BoundaryInds.at(i)));
		T_Boundary.at(T_BoundaryInds.at(i)) = true;
	}

	R_boundary_candidates.resize(T_BoundaryInds.size());
	R_BoundaryInds.resize(T_BoundaryInds.size());
}

void FeatureCloud::findResultBoundary() {
	/*int i = 0;
	for (int i = 0; i < T_BoundaryInds.size(); i++) {
		int T_ind = T_BoundaryInds.at(i);
		float band = DT(T_f.ind,T_ind);
		vector<int> candidate_inds = extract_band_indices(band, 0.5);
		R_boundary_candidates.at(i) = calculateSimilarityband(candidate_inds, T_ind);
		for (int j = 0; j < R_boundary_candidates.at(i).size(); j++) {
			cout << "candidate i score:" << R_boundary_candidates.at(i).at(j).dis<<endl;
		}

		R_BoundaryInds.at(i) = R_boundary_candidates.at(i).at(0).ind;
		Rboundary->push_back(resultkeypoints->at(R_BoundaryInds.at(i)));
	}*/
}

void FeatureCloud::meshstats(nnfDest d) {
	PolygonMesh::Ptr mesh; cloud_param_s* p; XYZCloud::Ptr Dest;
	switch (d) {
		case Q:   Dest = Qc;  mesh = Qmesh; p = &P.S; break;
		case T:   Dest = Tc;  mesh = Tmesh; p = &P.T; break;
	}
	p->area = MeshArea(mesh, Dest);
	p->diam = sqrt(p->area);
	p->mean_edge = 0;
	p->max_edge = 0;
	for (int i = 0; i < mesh->polygons.size(); i++) {
		Eigen::Vector3f p1, p2, p3;
		p1 = Dest->at(mesh->polygons.at(i).vertices.at(0)).getVector3fMap();
		p2 = Dest->at(mesh->polygons.at(i).vertices.at(1)).getVector3fMap();
		p3 = Dest->at(mesh->polygons.at(i).vertices.at(2)).getVector3fMap();
		float v1 = (p3 - p1).norm();
		float v2 = (p2 - p1).norm();
		float v3 = (p3 - p2).norm();
		if (p->max_edge < v1) {
			p->max_edge = v1;
		}
		if (p->max_edge < v2) {
			p->max_edge = v2;
		}
		if (p->max_edge < v3) {
			p->max_edge = v3;
		}
		p->mean_edge += v1 + v2 + v3;
	}
	p->mean_edge /= (3 * mesh->polygons.size());

	if (d == Q && P.RMode == FRACQ) {
		P.T.f.searchRadius = P.frac / 100 * p->diam;
		P.S.f.searchRadius = P.frac / 100 * p->diam;
	}

	if (d == T && P.RMode == FRACT) {
		P.T.f.searchRadius = P.frac / 100 * p->diam;
		P.S.f.searchRadius = P.frac / 100 * p->diam;
	}

	if (P.RMode == FRAC) {
		p->f.searchRadius = P.frac / 100 * p->diam;
	}
}

void FeatureCloud::ComputeKeyPoints(nnfDest d) {
	XYZCloud::Ptr Dest; XYZCloud::Ptr key; vector<pcl::Vertices>* poly;
	cloud_param_s* p;
	switch (d) {
	case Q:   Dest = Qc;         key = Qkeypoints; p = &P.S; poly = &Qpolygons; break;
	case T:   Dest = Tc;         key = Tkeypoints; p = &P.T; poly = &Tpolygons; break;
	}
	if (p->k.Method == ALL) {
		copyPointCloud(*Dest, *key);
	}
	else if (p->k.Method == BOUND) {
		if (p->k.salient_radius == 0) {
			p->k.salient_radius = p->f.searchRadius;
		}
		copyPointCloud(*Dest, *key);
		RemoveBoundary(key, *poly, *p);
	}
	else if (readCloudFile<PointXYZ>(p->k.path, *key) == -1) {
		
		if (p->k.Method == GRID) {

		}
		else {
			if (p->k.modelResolution == 0) 
				p->k.modelResolution = 3 * p->stat.MeanResolution;
			if (p->k.salient_radius == 0) p->k.salient_radius = keypoints_visl::RADIUS_SEARCH_MULTIPLIER*p->stat.MeanResolution;
			if (p->k.non_max_radius == 0) p->k.non_max_radius = keypoints_visl::NON_MAX_RADIUS_MULTIPLIER*p->stat.MeanResolution;

			computeKeyPoints(Dest,
				key,
				p->k.Method,
				p->k.modelResolution, p->k.border, p->k.autodefault);
		}
	}
	
}

void FeatureCloud::extractTemplateCandidate(int i, vector<ind_r_s>& scene_P_D,
	pcl::ExtractIndices<pcl::PointXYZ>::Ptr extract, float R, uint32_t &M ,bool full) {
	switch (P.pMode) {
	case GEODESIC: extractTemplateCandidateGeo(i, scene_P_D, R, M, full); break;
	case SPHERE  : extractTemplateCandidateEuc(i, scene_P_D, extract, R, full); break;
	case GEONN: extractTemplateCandidateGeoNN(i, scene_P_D, extract, R, full); break;
	}
}

void FeatureCloud::extractTemplateCandidateEuc(int i , vector<ind_r_s>& scene_P_D,
	pcl::ExtractIndices<pcl::PointXYZ>::Ptr extract, float R, bool full) {
	PointCloud<PointXYZ>::Ptr target(new PointCloud<PointXYZ>); Eigen::Vector4f centroid;
	PointCloud<PointXYZ>::Ptr source = (full ? Qc : Qkeypoints);
	XYZ Center = Qkeypoints->at(i);
	pcl::PointIndices::Ptr scene_indices; vector<float> sceneDistances;
	kdtree.radiusSearch(Center, T_f.RE, scene_indices->indices, sceneDistances);
	for (int i = 0; i < scene_indices->indices.size(); i++) {
			ind_r_s p(scene_indices->indices.at(i), sceneDistances.at(i));
			scene_P_D.push_back(p);
	}
}

void FeatureCloud::extractTemplateCandidateGeoNN(int i,  vector<ind_r_s>& scene_P_D,
	ExtractIndices<PointXYZ>::Ptr extract, float R, bool full) {
	PointCloud<PointXYZ>::Ptr target(new PointCloud<PointXYZ>),gridTarget(new PointCloud<PointXYZ>); Eigen::Vector4f centroid;
	PointIndices::Ptr scene_indices_temp(new pcl::PointIndices()),point_indice(new pcl::PointIndices());
	PointCloud<PointXYZ>::Ptr source= (full?Qc: Qkeypoints);
	vector<float> sceneDistances_temp, point_distance;
	XYZ Center = Qkeypoints->at(i);
	QGkdtree.nearestKSearch(Center, 1, point_indice->indices, point_distance);
	Q_G.resetDist(P.meshtype != NORMAL);
	vector<int> Ginds = Q_G.LimitedSSSP(point_indice->indices[0], R, P.meshtype != NORMAL);
	for (int i = 0; i < Ginds.size(); i++) {
		if (Ginds.at(i) < Qc->size()) {//not taking points from the dual cloud
			gridTarget->push_back(QGridCloud->at(Ginds.at(i)));
		}
	}
	kdtree.radiusSearch(Center, fmin(T_f.RE, R), scene_indices_temp->indices, sceneDistances_temp);
	pcl::KdTreeFLANN<PointXYZ> GKDTree;
	GKDTree.setInputCloud(gridTarget);
	for (int i = 0; i < sceneDistances_temp.size(); i++) {
		GKDTree.nearestKSearch(source->at(scene_indices_temp->indices.at(i)),1, point_indice->indices, point_distance);
		if (sceneDistances_temp[0] < grid_max_resolution) {//point is indeed near the surface
			ind_r_s p(scene_indices_temp->indices.at(i), Q_G.getVertDist(Ginds.at(point_indice->indices[0]), P.meshtype != NORMAL));
			//sceneDistances.push_back(sceneDistances_temp.at(i));
			scene_P_D.push_back(p);
		}
	}
}

void FeatureCloud::extractTemplateCandidateGeo(int i, vector<ind_r_s>& scene_P_D, float R, uint32_t &M, bool full) {
	float d;
	M = 0;
	//if (P.load_d_q) {
		for (int j = 0; j < model_vertices; j++) {
			//cout << i << "," << j << endl;
			d = DQ.at(i).at(j);
			if (d <= R) {
				scene_P_D.at(M) = (ind_r_s(j, d));
				M++;
			}
		}
	//}
	/*else {
		Q_G.resetDist(P.meshtype != NORMAL);
		std::vector<int> template_indices = Q_G.LimitedSSSP(i, R, P.meshtype != NORMAL);
		for (int j = 0; j < template_indices.size(); j++) {
			if (template_indices.at(j) < Qkeypoints->size()) {
				ind_r_s p(template_indices.at(j), Q_G.getVertDist(template_indices.at(j), P.meshtype != NORMAL));
				scene_P_D.push_back(p);
			}
		}
	}*/
}

ind_dis_s FeatureCloud::lower_scale_sim_max(uint32_t src, uint32_t result,int i, SimilarityMeasure sim= MSWDIS){
	vector<float>& discloud = sim == MSWDIS ? MSFsimilarity.at(i) : LSFsimilarity.at(i);
	float thresh = sim == MSWDIS ? R_thresh_ms : R_thresh_ls;
	float d,s;
	ind_dis_s max_dis(0, 0);
	for (int j = 0; j < model_vertices; j++) {//Passing over all 
			//cout << i << "," << j << endl;
		d = DQ.at(result).at(j);
		if (d < thresh) {
			s = discloud.at(j);
			if (s > max_dis.second) max_dis = ind_dis_s(j, s);
		};
	}
	//discloud->at(max_dis.first).intensity = 0;
	return max_dis;
}


vector<float> FeatureCloud::getTemplateDistances(patchMode Mode, int i,float& R) {
	R = 0;
	switch (Mode) {
	case SPHERE:   return Template_Euclid_Distance(i,R);
	case GEODESIC: return Template_Geo_Distance(i,R);
	case GEONN:    return Template_Geo_NNDistance(i,R);
	}
}

vector<float> FeatureCloud::Template_Euclid_Distance(int i,float& R) {
	vector<float> result(Tkeypoints->size());
	Eigen::Vector4f centroid;compute3DCentroid(*Tkeypoints, centroid);
	for (size_t i = 0; i < Tkeypoints->size(); ++i) {
		result.at(i) = (Tkeypoints->at(i).getVector3fMap() - centroid.head<3>()).norm();
		if (result.at(i) > R) R = result.at(i);
	}
	return result;
}

vector<float> FeatureCloud::Template_Geo_Distance(int ind,float& R) {
	vector<float> result(Tkeypoints->size());
	//if (P.load_d_t) {
		for (size_t j = 0; j < Tkeypoints->size(); ++j) {
			result.at(j) = DT.at(ind).at(j);
			if (result.at(j) > R) R = result.at(j);
		};
	/*}
	else {
		T_G.resetDist(P.meshtype != NORMAL);
		T_G.LimitedSSSP(ind, INFINITY, P.meshtype != NORMAL);
		for (size_t j = 0; j < Tkeypoints->size(); ++j) {
			result.at(j) = T_G.getVertDist(j, P.meshtype != NORMAL);
			if (result.at(j) > R) R = result.at(j);
		};
	}*/
	return result;
}

vector<float> FeatureCloud::Template_Geo_NNDistance(int i,float& R) {
	vector<float> result(Tkeypoints->size());
	vector<float> temp;
	vector<int> indices;
	for (size_t i = 0; i < Tkeypoints->size(); ++i) {
		TGkdtree.nearestKSearch(Tkeypoints->at(i), 1, indices, temp);
		result.at(i)=T_G.getVertDist(indices[0],P.meshtype!=NORMAL);
		if (result.at(i) > R) R = result.at(i);

	}
	return result;
}

HistogramInt<5> FeatureCloud::GTDIS(vector<barycentric_polygon_s> gt_bar) {
	HistogramInt<5> result;
	vector<bool> gt_i(Qkeypoints->size(), false);
	vector<int> gt_inds; vector<bool> T_inds(Tkeypoints->size(), false);
	for(int i = 0; i < Tkeypoints->size();i++){
		Vertices poly = Qmesh->polygons.at(gt_bar.at(i).index);
		for (int j = 0; j < poly.vertices.size(); j++) {
			if (!gt_i.at(poly.vertices.at(j))) {
				gt_i.at(poly.vertices.at(j)) = true;
				gt_inds.push_back(poly.vertices.at(j));
			}
		}
	}

	for (int k = 0; k < 5; k++) {
		result.counter[k] = 0;
		for (int i = 0; i < gt_inds.size(); i++) {
			if (!(T_inds.at(nnf5->at(gt_inds.at(i)).labels[k]))) {
				T_inds.at(nnf5->at(gt_inds.at(i)).labels[k]) = true;
				result.counter[k]++;
			};
		}
	}
	return result;
};

void FeatureCloud::extractCandidatestats(vector<ind_r_s>& scene_P_D, uint32_t M, vector<float>& tDistances, vector<float>& mindistances,
	vector<uint32_t>& Kappa,vector<uint32_t>& Kappa_j, vector<uint32_t>& Patch_Buddies, vector<float>& r_j,vector<uint32_t>& closest, HistogramInt<5>& DIS,int feature) {
	Kappa.assign(part_vertices, 0);
	int NN = P.nnfType == NNF5 ? 5 : 1;
	int DISs = 0;
	uint32_t Template_idx;
	for (int k = 0; k < NN; k++) {
		for (size_t j = 0; j < M; ++j) {
			int i = scene_P_D.at(j).first;
		
			switch (P.nnfType) {
			case NNF: Template_idx = nnf->at(i).label; break;
			case NNF1: Template_idx = nnf1->at(i).labels[0]; break;

			//case NNF1: Template_idx = (P.r_thresh<100)?nnf_f.at(feature)->at(i).labels[0] :nnf1->at(i).labels[0]; break;
			case NNF5: Template_idx = nnf5->at(i).labels[k]; NN = 5; break;
			}
			Patch_Buddies[j] = Template_idx;
			if ((Template_idx < MAXUINT32) && (tDistances.at(Template_idx) < R_thresh)) {
				//if (k == 0) {
					
					r_j[j] = abs(scene_P_D.at(j).second - tDistances.at(Template_idx));
					if (mindistances.at(Template_idx) > r_j[j]) {
						mindistances.at(Template_idx) = r_j[j];
						closest.at(Template_idx) = i;
					}
				//}
				if (Kappa.at(Template_idx) == 0) {
					DISs++;
				}
				Kappa[Template_idx]++; 
				//Kappac->at(Template_idx).counter[k]= Kappa[Template_idx];
			}
			DIS.counter[k] = DISs;
		}

		/*if (k == 0)
			for (size_t j = 0; j < M; j++) {
				Kappa_j.at(j) = (Patch_Buddies[j] != MAXUINT32) ? Kappa.at(Patch_Buddies[j]) : MAXINT;
			}*/

	}
}

float FeatureCloud::DDISfunc(int M, vector<uint32_t>& Kappa_j, vector<float>& r_j) {
	float DDIS_Score = 0;
	for (size_t j = 0; j < M; j++) {
		if (Kappa_j.at(j) > M || r_j[j] > R_thresh) continue;
		float tmp = 1;
		float normal = Tkeypoints->size() > M ? ((Tkeypoints->size()) / static_cast<float>(M)) : 1;
		//float normal = Tkeypoints->size()>M ? ((Tkeypoints->size()) / static_cast<float>(M)):1;
		float denom = exp(1 - int(Kappa_j.at(j)));// ((Tkeypoints->size()) / static_cast<float>(M)));
		float nom = 1 / (r_j[j] / (P.div_frac / 100 * P.S.diam) + 1);
		DDIS_Score += nom * denom;
	}
	return DDIS_Score;
}

float FeatureCloud::WDISfunc(int M, vector<uint32_t>& Kappa, vector<float>& mindistances) {
	float WDIS_score = 0;
	for (int i = 0; i < part_vertices; i++) {
		if (Kappa.at(i)!=0)
		{
			WDIS_score += (1 / ((mindistances.at(i)/ (P.div_frac / 100 * P.S.diam)) + 1));
		}
	}
	return WDIS_score;
};

tris_dis_s FeatureCloud::MSWDISfunc(int M, vector<uint32_t>& Kappa, vector<float>& mindistances, vector<float>& Distances) {
	float WDIS_score = 0;
	float LWDIS_score = 0;

	for (int i = 0; i < part_vertices; i++) {
		if (Kappa.at(i) != 0)
		{
			float addition = (1 / ((mindistances.at(i) / (P.div_frac / 100 * P.S.diam)) + 1));
			WDIS_score += addition;
			if (Distances.at(i) < R_thresh_ms) {
				LWDIS_score += addition;
			}
		}
	}
	return tris_dis_s(WDIS_score,LWDIS_score,0);
};

tris_dis_s FeatureCloud::TRISDISfunc(int M, vector<uint32_t>& Kappa, vector<float>& mindistances, vector<float>& Distances) {
	float WDIS_score = 0;
	float MWDIS_score = 0;
	float LWDIS_score = 0;

	for (int i = 0; i < part_vertices; i++) {
		if (Kappa.at(i) != 0)
		{
			float addition = (1 / ((mindistances.at(i) / (P.div_frac / 100 * P.S.diam)) + 1));
			WDIS_score += addition;
			if (Distances.at(i) < R_thresh_ms) {
				MWDIS_score += addition;
			}
			if (Distances.at(i) < R_thresh_ls) {
				LWDIS_score += addition;
			}

		}
	}
	return tris_dis_s(WDIS_score, MWDIS_score, LWDIS_score);
};

float FeatureCloud::WDISPfunc(int M, vector<uint32_t>& Kappa, vector<float>& mindistances, vector<float>& Distances) {
	float WDIS_score = 0;
	for (int i = 0; i < part_vertices; i++) {
		if (Kappa.at(i) != 0)
		{
			WDIS_score += (1 / ((mindistances.at(i) / (P.div_frac / 100 * P.S.diam)) + 1))*exp(-Distances.at(i)/R_thresh);//giving higher weight to matches near the origin
		}
	}
	return WDIS_score;
};


float FeatureCloud::HDISfunc(int M, vector<uint32_t>& Kappa, vector<float>& mindistances) {
	float HDIS_score = 0;
	for (int i = 0; i < part_vertices; i++) {
		if (Kappa.at(i) != 0)
		{
			HDIS_score += exp(-mindistances.at(i) / (P.div_frac / 100 * P.S.diam));
		}
	}
	return HDIS_score;
};

tris_dis_s FeatureCloud::TRIDISPfunc(int M, vector<uint32_t>& Kappa, vector<float>& mindistances, vector<float>& Distances) {
	float WDIS_score = 0;
	float MWDIS_score = 0;
	float LWDIS_score = 0;

	for (int i = 0; i < part_vertices; i++) {
		if (Kappa.at(i) != 0)
		{
			float addition = exp(-mindistances.at(i) / (P.div_frac / 100 * P.S.diam));//(1 / ((pow(mindistances.at(i),2) / (P.div_frac / 100 * P.S.diam)) + 1));
			WDIS_score += addition;
			if (Distances.at(i) < R_thresh_ms) {
				MWDIS_score += addition;// exp2(-pow(mindistances.at(i), 2) / (2 * 2 / 3 * P.div_frac / 100 * P.S.diam));
			}
			if (Distances.at(i) < R_thresh_ls) {
				LWDIS_score += addition;// exp2(-pow(mindistances.at(i), 2) / (2 / 3 * P.div_frac / 100 * P.S.diam));;
			}

		}
	}
	return tris_dis_s(WDIS_score, MWDIS_score, LWDIS_score);
};

float FeatureCloud::HDISPfunc(int M, vector<uint32_t>& Kappa, vector<float>& r_j){
	float HDIS_score = 0;
	for (size_t j = 0; j < M; ++j) {
		if (Kappa.at(j) != 0)
		{
			HDIS_score += exp(-r_j[j] / (P.div_frac / 100 * P.S.diam));
		}
	}
	return HDIS_score;

}

/*vector<ind_r_s>  FeatureCloud::extract_patch_match(uint32_t t_i, uint32_t q_i) {
	vector<ind_r_s> scene_P_D;
	float R;
	uint32_t M;
	vector <float> D = getTemplateDistances(P.pMode, t_i, R);
	extractTemplateCandidate(q_i, scene_P_D, extract, R_thresh*1.05,M);//to be done for 1st feature point and trimmed down with each smaller radius point
	vector<uint32_t> closest_temp(Tkeypoints->size(), MAXUINT32);//to be done for each point
	std::vector<float> r_j(M);
	std::vector<uint32_t> Kappa_j(M);
	std::vector<uint32_t> Patch_Buddies(M);
	vector<float> mindistances(Tkeypoints->size(), INFINITY);//distance difference of the minimal corresponding point

	extractCandidatestats(scene_P_D, M, D, mindistances, Kappa, Kappa_j, Patch_Buddies, r_j, closest_temp, DIS_cummulative->at(t_i),q_i);
	vector<ind_r_s> result;
	for (int i = 0; i < Tkeypoints->size(); i++) {
		result.push_back(ind_r_s(closest_temp.at(i), mindistances.at(i)));
	}
	return result;
};*/


void FeatureCloud::icp() {
	Eigen::Matrix4f transform,inverseTransform ;
	cout << "preforming icp";

	get_icp_transform(Tc, Qc, transform);
	cout << "icp calculated";
	print4x4Matrix(transform.cast<float>());
	inverseTransform = transform.inverse();
	transformPointCloud(*Tc, *Tc, transform);
	transformPointCloud(*Tkeypoints, *Tkeypoints, transform);
	transformPointCloud(*distc, *distc, transform);
	transformPointCloud(*kappac, *kappac, transform);
	transformPointCloud(*DDIS_refined_NNF, *DDIS_refined_NNF, transform);
}

void FeatureCloud::icp_mds() {
	Eigen::Matrix4f transform, inverseTransform;
	cout << "preforming icp";
	copyPointCloud(*Tc, *Tkeypoints);
	get_icp_transform(Tc, Qc, transform);
	cout << "icp calculated";
	print4x4Matrix(transform.cast<float>());
	inverseTransform = transform.inverse();
	transformPointCloud(*Tc, *Tc, transform);
}

void FeatureCloud::show_correspondences(int num) {
	PointCloud<PointXYZ>::Ptr templateCloud(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr sceneCloud(new PointCloud<PointXYZ>);
	fromPCLPointCloud2(Qmesh->cloud, *sceneCloud);	fromPCLPointCloud2(Tmesh->cloud, *templateCloud);
	pcl::PCA<PointXYZ> pca(*sceneCloud);
	Eigen::Matrix3f xyz_coords = pca.getEigenVectors();
	Eigen::Vector3f ax(xyz_coords(0), xyz_coords(1), xyz_coords(2));
	Eigen::Vector3f ax1(xyz_coords(3), xyz_coords(4), xyz_coords(5));
	Eigen::Vector3f ax2(xyz_coords(6), xyz_coords(7), xyz_coords(8));
	string input = "";
	float c1 = 1 / sqrt(3);	float c2 = 1 / sqrt(3); float c3 = 1 / sqrt(3);

	cout << "choose ratio of primary ax \n";
	string::size_type sz;
	/*std::getline(cin, input);
	if (check_num(input)) {
		c1 = stof(input, &sz);
	}

	cout << "choose ratio of secondary ax \n";
	std::getline(cin, input);
	if (check_num(input)) {
		c2 = stof(input, &sz);
	}

	cout << "choose ratio of minor ax \n";
	std::getline(cin, input);
	if (check_num(input)) {
		c3 = stof(input, &sz);
	}*/

	Eigen::Vector3f n_1 = { 1,0,0 }; //c1 * ax.normalized() + +c2 * ax1.normalized() + c3 * ax2.normalized();
	n_1.normalize();
	Eigen::Vector4f n(n_1(0), n_1(1), n_1(2), 0);
	Eigen::Vector4f centroid; Eigen::Vector4f farthest_point;
	pcl::compute3DCentroid(*sceneCloud, centroid);
	pcl::getMaxDistance(*sceneCloud, centroid, farthest_point);
	Eigen::Vector3f radi = (farthest_point - centroid).head<3>();
	pcl::demeanPointCloud(*sceneCloud, centroid, *sceneCloud);
	Eigen::Vector4f centroidT; 	pcl::compute3DCentroid(*templateCloud, centroidT);
	pcl::demeanPointCloud(*templateCloud, centroid, *templateCloud);
	pcl::CorrespondencesPtr result_corrs(new pcl::Correspondences());
	float multiplier = 1;
	/*cout << "choose Number of scene radiuses from Center \n";
	std::getline(cin, input);
	if (check_num(input)) {
		multiplier = stof(input, &sz);
	}*/

	Eigen::Vector4f ntrans = n * multiplier * radi.norm();
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	if (multiplier != 0) {
		transform_1(0, 3) = ntrans(0);
		transform_1(1, 3) = ntrans(1);
		transform_1(2, 3) = ntrans(2);
	}
	XYZCloud cloud;
	transformPointCloud(*templateCloud, cloud, transform_1);
	toPCLPointCloud2(cloud, Tmesh->cloud);
	toPCLPointCloud2(*sceneCloud, Qmesh->cloud);
	bool show = true;
	//cout << "enter base point size from 1-20:";
	//std::getline(cin, input);
	float template_point_size = 10;
	/*if (check_num(input)) {
		template_point_size = stof(input, &sz);
	}*/
	int v4(0);
	PointCloud<PointXYZRGB>::Ptr shownCorrs(new PointCloud<PointXYZRGB>);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("3D Viewer 1"));
	viewer2->setBackgroundColor(255, 255, 255);
	viewer2->initCameraParameters();
	viewer2->addPolygonMesh(*Tmesh, "temp1");
	viewer2->addPolygonMesh(*Qmesh, "scn1");
	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, "temp1");
	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, "scn1");
	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "temp1");

	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "scn1");

	bool alter=false;
	for (size_t i = 0; i < Result.at(num).size(); i++) {
		int r = lcmap[i % 1000][0];				int g = lcmap[i % 1000][1];    int b = lcmap[i % 1000][2];
		pcl::PointXYZRGB  p_src;
		copyPoint(cloud.at(feature_point_inds.at(i)), p_src);
		p_src.r = r; p_src.g = g; p_src.b = b;
		pcl::PointXYZRGB  p_tgt;
		copyPoint(sceneCloud->at(Result.at(num).at(i).first), p_tgt);
		int c = min(int(255*Result.at(num).at(i).second),255);
		p_tgt.r = r; p_tgt.g = g; p_tgt.b = b;
		shownCorrs->push_back(p_src);
		shownCorrs->push_back(p_tgt);
		std::stringstream ss("line");
		ss << i;
		std::stringstream sss("spheresource");
		sss << i;
		std::stringstream ssss("spheretarget");
		ssss << i;
		if (alter)
		{
			viewer2->addLine(p_src, p_tgt, icmap[c][0], icmap[c][1], icmap[c][2], sss.str());
			viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, sss.str());
		}
		else
		{
			viewer2->addLine(p_src, p_tgt, icmap[c][0], icmap[c][1], icmap[c][2], ss.str());
			viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, ss.str());
		}
		alter = !alter;
	}
	string corrs = "Corrs";
	viewer2->addPointCloud(shownCorrs, corrs);
	//viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, corrs);

	//savePointCloudToFile(shownCorrs, "snapshot.ply");
	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, corrs);
	viewer2->setWindowBorders(true);
	viewer2->resetCamera();
	while (!viewer2->wasStopped())
	{
		viewer2->spinOnce();
	}

};


void FeatureCloud::show_correspondences_side(int num) {
	int v2(0);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerL(new pcl::visualization::PCLVisualizer("3D Viewer 1"));
	string::size_type sz;

	viewerL->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	int v3(0);
	viewerL->createViewPort(0.0, 0.0, 0.5, 1.0, v3);

	PointCloud<PointXYZ>::Ptr templateCloud(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr sceneCloud(new PointCloud<PointXYZ>);
	fromPCLPointCloud2(Qmesh->cloud, *sceneCloud);	fromPCLPointCloud2(Tmesh->cloud, *templateCloud);

	pcl::CorrespondencesPtr result_corrs(new pcl::Correspondences());
	cout << "enter base point size from 1-20:";
	string input;
	//std::getline(cin, input);
	float template_point_size = 10;
	/*if (check_num(input)) {
		template_point_size = stof(input, &sz);
	}*/
	int v4(0);
	PointCloud<PointXYZRGB>::Ptr shownCorrsT(new PointCloud<PointXYZRGB>), shownCorrsS(new PointCloud<PointXYZRGB>);
	viewerL->addPolygonMesh(*Tmesh, "temp1",v2);
	viewerL->addPolygonMesh(*Qmesh, "scn1",v3);
	viewerL->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "temp1",v2);

	viewerL->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "scn1",v3);

	viewerL->setBackgroundColor(255, 255, 255,v2);
	viewerL->setBackgroundColor(255, 255, 255, v3);

	//viewerL->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, "temp1", v2);
	//viewerL->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, "scn1" , v3);
	bool alter = false;
	for (size_t i = 0; i < Result.at(num).size(); i++) {
		int r = lcmap[i % 1000][0];				int g = lcmap[i % 1000][1];    int b = lcmap[i % 1000][2];
		pcl::PointXYZRGB  p_src;
		copyPoint(templateCloud->at(feature_point_inds.at(i)), p_src);
		p_src.r = r; p_src.g = g; p_src.b = b;
		pcl::PointXYZRGB  p_tgt;
		copyPoint(sceneCloud->at(Result.at(num).at(i).first), p_tgt);
		p_tgt.r = r; p_tgt.g = g; p_tgt.b = b;
		shownCorrsT->push_back(p_src);
		shownCorrsS->push_back(p_tgt);
	}
	string corrs = "Corrs";
	viewerL->addPointCloud(shownCorrsT, "tempc", v2);	viewerL->addPointCloud(shownCorrsS, "scnc", v3);

	//viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, corrs);

	//savePointCloudToFile(shownCorrs, "snapshot.ply");
	viewerL->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, "tempc", v2);
	viewerL->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, "scnc", v3);

	viewerL->setWindowBorders(true);
	viewerL->resetCamera();
	while (!viewerL->wasStopped())
	{
		viewerL->spinOnce();
	}
};

void FeatureCloud::load_feature_patch(int num) {
	string path = P.result_dir + "\\feature_patch" + to_string(num) + ".ply";
	io::loadPLYFile(path, *Feature_Patches.at(num));
};

void FeatureCloud::load_feature_similarity(int num) {
	string path = P.result_dir + "\\F_" + to_string(num) + "_similarity.pcd";
	io::loadPLYFile(path, *FsimilarityCloud.at(num));
	path = P.result_dir + "\\F_" + to_string(num) + "_similarity.ply";
	io::loadPLYFile(path, *FsimilarityCloudColor.at(num));
};

void FeatureCloud::visualizeWDIS(uint32_t src, uint32_t target, float multiplier){
	vector<ind_r_s> T_P_D,S_P_D;
	vector<ind_r_s> min_matches;
	PolygonMeshPtr TemplatePatch = extract_feature_patch(src, T_P_D, T);
	PolygonMeshPtr ScenePatch = extract_feature_patch(target, S_P_D, Q);
	vector<uint32_t> T_map(part_vertices,MAXUINT32);
	XYZRGBCloud T_D, S_D;
	PointCloud<PointXYZRGB>::Ptr shownCorrs(new PointCloud<PointXYZRGB>);
	//min_matches.resize(T_P_D.size());
	uint32_t t_center;
	for (int i = 0; i < T_P_D.size(); i++) {
		PointXYZRGB p; copyPoint(Tc->at(T_P_D.at(i).first),p);
		int cind = int(T_P_D.at(i).second / (R_thresh*1.05) * 255);
		p.r = icmap[cind][0];		p.g = icmap[cind][1];		p.b = icmap[cind][2];
		T_D.push_back(p);
		T_map.at(T_P_D.at(i).first) = i;
		min_matches.push_back(ind_r_s(MAXUINT32, INFINITY));//initializing matches to no match and infinite distance
	}

	for (int i = 0; i < S_P_D.size(); i++) {
		PointXYZRGB p; copyPoint(Qc->at(S_P_D.at(i).first), p);
		int cind = int(S_P_D.at(i).second / (R_thresh * 1.05) * 255);
		p.r = icmap[cind][0];		p.g = icmap[cind][1];		p.b = icmap[cind][2];
		S_D.push_back(p);
		if (S_P_D.at(i).second==0) t_center = i;
		uint32_t origl = nnf1->at(S_P_D.at(i).first).labels[0];
		float d = INFINITY;
		if (origl < MAXUINT32 &&T_map.at(origl) < MAXUINT32) {//we have a real match on our patch
			d = abs(S_P_D.at(i).second - T_P_D.at(T_map.at(origl)).second);
			if (min_matches.at(T_map.at(origl)).second > d) min_matches.at(T_map.at(origl)) = ind_r_s(i, d);
		}
	}

	Eigen::Vector3f n_1 = { 1,0,0 }; //c1 * ax.normalized() + +c2 * ax1.normalized() + c3 * ax2.normalized();
	n_1.normalize();
	Eigen::Vector4f n(n_1(0), n_1(1), n_1(2), 0);
	Eigen::Vector4f centroid; Eigen::Vector4f farthest_point;
	pcl::compute3DCentroid(S_D, centroid);
	pcl::getMaxDistance(S_D, centroid, farthest_point);
	Eigen::Vector3f radi = (farthest_point - centroid).head<3>();
	pcl::demeanPointCloud(S_D, centroid, S_D);
	Eigen::Vector4f centroidT; 	pcl::compute3DCentroid(T_D, centroidT);
	pcl::demeanPointCloud(T_D, centroidT, T_D);
	pcl::CorrespondencesPtr result_corrs(new pcl::Correspondences());
	//float multiplier = 2;
	/*cout << "choose Number of scene radiuses from Center \n";
	std::getline(cin, input);
	if (check_num(input)) {
	multiplier = stof(input, &sz);
	}*/

	Eigen::Vector4f ntrans = n * multiplier * radi.norm();
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	if (multiplier != 0) {
		transform_1(0, 3) = ntrans(0);
		transform_1(1, 3) = ntrans(1);
		transform_1(2, 3) = ntrans(2);
	}
	XYZCloud cloud;
	transformPointCloud(T_D, T_D, transform_1);
	toPCLPointCloud2(T_D, TemplatePatch->cloud);
	toPCLPointCloud2(S_D, ScenePatch->cloud);
	bool show = true;
	//cout << "enter base point size from 1-20:";
	//std::getline(cin, input);
	float template_point_size = 20;
	/*if (check_num(input)) {
	template_point_size = stof(input, &sz);
	}*/
	int v4(0);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("3D Viewer 1"));
	viewer2->setBackgroundColor(255, 255, 255);
	viewer2->initCameraParameters();
	viewer2->addPolygonMesh(*TemplatePatch, "temp1");
	viewer2->addPolygonMesh(*ScenePatch, "scn1");
	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, "temp1");
	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "temp1");

	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, "scn1");
	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "scn1");

	bool alter = false;

	PointXYZRGB p_src = T_D.at(T_map.at(src));
	p_src.r = 255; p_src.g = 0; p_src.b = 255;

	PointXYZRGB p_tgt = S_D.at(t_center);
	p_tgt.r = 255; p_tgt.g = 0; p_tgt.b = 255;
	shownCorrs->push_back(p_src);
	shownCorrs->push_back(p_tgt);

	for (size_t i = 0; i < min_matches.size(); i++) {
		if (min_matches.at(i).first < MAXUINT32) {
			int cind = int(min_matches.at(i).second / (R_thresh * 1.05) * 255);

			int r = icmap[cind][0];				
			int g = icmap[cind][1];
			int b = icmap[cind][2];
			pcl::PointXYZRGB  p_src;
			copyPoint(T_D.at(i), p_src);
			p_src.r = r; p_src.g = g; p_src.b = b;
			pcl::PointXYZRGB  p_tgt;
			copyPoint(S_D.at(min_matches.at(i).first), p_tgt);
			p_tgt.r = r; p_tgt.g = g; p_tgt.b = b;
			//shownCorrs->push_back(p_src);
			//shownCorrs->push_back(p_tgt);
			std::stringstream ss("line");
			ss << i;
			std::stringstream sss("spheresource");
			sss << i;
			std::stringstream ssss("spheretarget");
			ssss << i;
			if (alter)
			{
				viewer2->addLine(p_src, p_tgt, r, g, b, sss.str());
				viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, sss.str());
			}
			else
			{
				viewer2->addLine(p_src, p_tgt, r, g, b, ss.str());
				viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, ss.str());
			}
			alter = !alter;
		}
	}
	string corrs = "Corrs";
	viewer2->addPointCloud(shownCorrs, corrs);
	//viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, corrs);

	//savePointCloudToFile(shownCorrs, "snapshot.ply");
	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, corrs);
	viewer2->setWindowBorders(true);
	viewer2->resetCamera();
	while (!viewer2->wasStopped())
	{
		viewer2->spinOnce();
	}
	viewer2->close();
}

void FeatureCloud::visualizeDDIS(uint32_t src, uint32_t target, float multiplier) {
	vector<ind_r_s> T_P_D, S_P_D;
	vector<ind_r_s> min_matches;
	PolygonMeshPtr TemplatePatch = extract_feature_patch(src, T_P_D, T);
	PolygonMeshPtr ScenePatch = extract_feature_patch(target, S_P_D, Q);
	vector<uint32_t> T_map(part_vertices, MAXUINT32);
	XYZRGBCloud T_D, S_D;
	PointCloud<PointXYZRGB>::Ptr shownCorrs(new PointCloud<PointXYZRGB>);
	//min_matches.resize(T_P_D.size());
	uint32_t t_center;
	for (int i = 0; i < T_P_D.size(); i++) {
		PointXYZRGB p; copyPoint(Tc->at(T_P_D.at(i).first), p);
		int cind = int(T_P_D.at(i).second / (R_thresh*1.05) * 255);
		p.r = icmap[cind][0];		p.g = icmap[cind][1];		p.b = icmap[cind][2];
		T_D.push_back(p);
		T_map.at(T_P_D.at(i).first) = i;
		min_matches.push_back(ind_r_s(MAXUINT32, INFINITY));//initializing matches to no match and infinite distance
	}
	vector<ind_r_s> dnnf(S_P_D.size(), ind_r_s(MAXUINT32, INFINITY));

	for (int i = 0; i < S_P_D.size(); i++) {
		PointXYZRGB p; copyPoint(Qc->at(S_P_D.at(i).first), p);
		int cind = int(S_P_D.at(i).second / (R_thresh * 1.05) * 255);
		p.r = icmap[cind][0];		p.g = icmap[cind][1];		p.b = icmap[cind][2];
		S_D.push_back(p);
		if (S_P_D.at(i).second == 0) t_center = i;
		uint32_t origl = nnf1->at(S_P_D.at(i).first).labels[0];
		float d = INFINITY;
		if (origl < MAXUINT32 &&T_map.at(origl) < MAXUINT32) {//we have a real match on our patch
			d = abs(S_P_D.at(i).second - T_P_D.at(T_map.at(origl)).second);
			dnnf.at(i) = ind_r_s(T_map.at(origl), d);
			if (min_matches.at(T_map.at(origl)).second > d) min_matches.at(T_map.at(origl)) = ind_r_s(i, d);
		}
	}

	Eigen::Vector3f n_1 = { 1,0,0 }; //c1 * ax.normalized() + +c2 * ax1.normalized() + c3 * ax2.normalized();
	n_1.normalize();
	Eigen::Vector4f n(n_1(0), n_1(1), n_1(2), 0);
	Eigen::Vector4f centroid; Eigen::Vector4f farthest_point;
	pcl::compute3DCentroid(S_D, centroid);
	pcl::getMaxDistance(S_D, centroid, farthest_point);
	Eigen::Vector3f radi = (farthest_point - centroid).head<3>();
	pcl::demeanPointCloud(S_D, centroid, S_D);
	Eigen::Vector4f centroidT; 	pcl::compute3DCentroid(T_D, centroidT);
	pcl::demeanPointCloud(T_D, centroidT, T_D);
	pcl::CorrespondencesPtr result_corrs(new pcl::Correspondences());
	//float multiplier = 2;
	/*cout << "choose Number of scene radiuses from Center \n";
	std::getline(cin, input);
	if (check_num(input)) {
	multiplier = stof(input, &sz);
	}*/

	Eigen::Vector4f ntrans = n * multiplier * radi.norm();
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	if (multiplier != 0) {
		transform_1(0, 3) = ntrans(0);
		transform_1(1, 3) = ntrans(1);
		transform_1(2, 3) = ntrans(2);
	}
	XYZCloud cloud;
	transformPointCloud(T_D, T_D, transform_1);
	toPCLPointCloud2(T_D, TemplatePatch->cloud);
	toPCLPointCloud2(S_D, ScenePatch->cloud);
	bool show = true;
	//cout << "enter base point size from 1-20:";
	//std::getline(cin, input);
	float template_point_size = 20;
	/*if (check_num(input)) {
	template_point_size = stof(input, &sz);
	}*/
	int v4(0);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("3D Viewer 1"));
	viewer2->setBackgroundColor(255, 255, 255);
	viewer2->initCameraParameters();
	viewer2->addPolygonMesh(*TemplatePatch, "temp1");
	viewer2->addPolygonMesh(*ScenePatch, "scn1");
	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, "temp1");
	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.9, "temp1");

	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, "scn1");
	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.9, "scn1");

	bool alter = false;

	PointXYZRGB p_src = T_D.at(T_map.at(src));
	p_src.r = 255; p_src.g = 0; p_src.b = 255;

	PointXYZRGB p_tgt = S_D.at(t_center);
	p_tgt.r = 255; p_tgt.g = 0; p_tgt.b = 255;
	shownCorrs->push_back(p_src);
	shownCorrs->push_back(p_tgt);

	for (size_t i = 0; i < min_matches.size(); i++) {
		if (min_matches.at(i).first < MAXUINT32) {
			int cind = int(min_matches.at(i).second / (R_thresh * 1.05) * 255);

			int r = icmap[cind][0];
			int g = icmap[cind][1];
			int b = icmap[cind][2];
			pcl::PointXYZRGB  p_src;
			copyPoint(T_D.at(i), p_src);
			p_src.r = r; p_src.g = g; p_src.b = b;
			pcl::PointXYZRGB  p_tgt;
			copyPoint(S_D.at(min_matches.at(i).first), p_tgt);
			p_tgt.r = r; p_tgt.g = g; p_tgt.b = b;
			//shownCorrs->push_back(p_src);
			//shownCorrs->push_back(p_tgt);
			std::stringstream ss("line");
			ss << i;
			std::stringstream sss("spheresource");
			sss << i;
			std::stringstream ssss("spheretarget");
			ssss << i;
			if (alter)
			{
				viewer2->addLine(p_src, p_tgt, r, g, b, sss.str());
				viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, sss.str());
			}
			else
			{
				viewer2->addLine(p_src, p_tgt, r, g, b, ss.str());
				viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, ss.str());
			}
			alter = !alter;
		}
	}
	string corrs = "Corrs";
	viewer2->addPointCloud(shownCorrs, corrs);
	//viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, corrs);

	//savePointCloudToFile(shownCorrs, "snapshot.ply");
	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, corrs);
	viewer2->setWindowBorders(true);
	viewer2->resetCamera();
	while (!viewer2->wasStopped())
	{
		viewer2->spinOnce();
	}
	viewer2->close();
}

void FeatureCloud::show_similarity_side(int num) {
	int v2(0);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerL(new pcl::visualization::PCLVisualizer("3D Viewer 1"));
	string::size_type sz;

	viewerL->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	int v3(0);
	viewerL->createViewPort(0.0, 0.0, 0.5, 1.0, v3);

	PointCloud<PointXYZ>::Ptr templateCloud(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr Tc2(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr sceneCloud(new PointCloud<PointXYZ>);
	copyPointCloud(*FsimilarityCloud.at(num), *sceneCloud);	fromPCLPointCloud2(Feature_Patches.at(num)->cloud, *templateCloud);
	copyPointCloud(*Tc, *Tc2);
	pcl::CorrespondencesPtr result_corrs(new pcl::Correspondences());
	//cout << "enter base point size from 1-20:";
	string input;
	//std::getline(cin, input);
	float template_point_size = 15;
	/*if (check_num(input)) {
	template_point_size = stof(input, &sz);
	}*/
	int v4(0);
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*sceneCloud, centroid);
	pcl::demeanPointCloud(*FsimilarityCloudColor.at(num), centroid, *FsimilarityCloudColor.at(num));
	Eigen::Vector4f centroidT; 	pcl::compute3DCentroid(*Tc, centroidT);
	pcl::demeanPointCloud(*templateCloud, centroidT, *templateCloud);
	pcl::demeanPointCloud(*Tc2, centroidT, *Tc2);
	toPCLPointCloud2(*templateCloud, Feature_Patches.at(num)->cloud);
	PointCloud<PointXYZRGB>::Ptr shownCorrsT(new PointCloud<PointXYZRGB>), shownCorrsS(new PointCloud<PointXYZRGB>);
	toPCLPointCloud2(*FsimilarityCloudColor.at(num), Qmesh->cloud);

	viewerL->addPolygonMesh(*Feature_Patches.at(num), "temp1", v2);
	viewerL->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "temp1", v2);

	toPCLPointCloud2(*FsimilarityCloudColor.at(num), Qmesh->cloud);
	viewerL->addPolygonMesh(*Qmesh, "scn21", v3);
	viewerL->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "scn21", v3);

	//viewerL->addPointCloud(FsimilarityCloudColor.at(num), "scn1", v3);
	//viewerL->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, "temp1", v2);
	//viewerL->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, "scn1" , v3);
	bool alter = false;
	pcl::PointXYZRGB  p_src;
	copyPoint(Tc2->at(feature_point_inds.at(num)), p_src);
	p_src.r = 255; p_src.g = 0; p_src.b = 255;

	shownCorrsT->push_back(p_src);
	for (size_t i = 0; i < 5; ++i) {
		int r = hcmap[i*50][0];				int g = hcmap[i * 50][1];    int b = hcmap[i * 50][2];
		pcl::PointXYZRGB  p_tgt;
		copyPoint(FsimilarityCloudColor.at(num)->at(Result.at(i).at(num).first), p_tgt);
		p_tgt.r = r; p_tgt.g = g; p_tgt.b = b;
		shownCorrsT->push_back(p_src);
		shownCorrsS->push_back(p_tgt);
	}
	string corrs = "Corrs";
	viewerL->addPointCloud(shownCorrsT, "tempc", v2);	viewerL->addPointCloud(shownCorrsS, "scnc", v3);

	//viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, corrs);

	//savePointCloudToFile(shownCorrs, "snapshot.ply");
	viewerL->setBackgroundColor(255, 255, 255, v2);
	viewerL->setBackgroundColor(255, 255, 255, v3);

	viewerL->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, "tempc", v2);
	viewerL->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, "scnc", v3);
	viewerL->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scn1", v3);

	viewerL->setWindowBorders(true);
	viewerL->resetCamera();
	while (!viewerL->wasStopped())
	{
		viewerL->spinOnce();
	}
};

//A function which shows relevant visual information on the matching
void FeatureCloud::showNNF(uint32_t ID) {
	/*PointCloud<PointXYZ>::Ptr templateCloud(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr sceneCloud(new PointCloud<PointXYZ>);

	float DIS = calculateSimilarity(ID);
	extractResult(false);
	this->registration();
	copyPointCloud(*Tc, *templateCloud); copyPointCloud(*Qc, *sceneCloud);
	XYZRGBCloud::Ptr RNNF_color = visualizeCloud<XYZL>(resultnnf);
	XYZRGBCloud::Ptr dist_color = visualizeCloud<XYZI>(distc);
	XYZRGBCloud::Ptr kappa_color = visualizeCloud<XYZI>(kappac);

	PointCloud<PointXYZ>::Ptr sceneCorr(new PointCloud<PointXYZ>), TemplateCorr(new PointCloud<PointXYZ>);
	Eigen::Vector4f centroid; Eigen::Vector4f farthest_point;
	pcl::compute3DCentroid(*sceneCloud, centroid);
	pcl::getMaxDistance(*sceneCloud, centroid, farthest_point);
	Eigen::Vector3f radi = (farthest_point - centroid).head<3>();
	PointCloud<PointXYZ>::Ptr pcacloud(new PointCloud<PointXYZ>);
	copyPointCloud(*sceneCloud, *pcacloud);
	pcl::PCA<PointXYZ> pca(*pcacloud);
	Eigen::Matrix3f xyz_coords = pca.getEigenVectors();
	Eigen::Vector3f ax(xyz_coords(0), xyz_coords(1), xyz_coords(2));
	Eigen::Vector3f ax1(xyz_coords(3), xyz_coords(4), xyz_coords(5));
	Eigen::Vector3f ax2(xyz_coords(6), xyz_coords(7), xyz_coords(8));
	string input = "";
	float c1 = 1 / sqrt(3);	float c2 = 1 / sqrt(3); float c3 = 1 / sqrt(3);

	cout << "choose ratio of primary ax \n";
	string::size_type sz;
	std::getline(cin, input);
	if (check_num(input)) {
		c1 = stof(input, &sz);
	}

	cout << "choose ratio of secondary ax \n";
	std::getline(cin, input);
	if (check_num(input)) {
		c2 = stof(input, &sz);
	}

	cout << "choose ratio of minor ax \n";
	std::getline(cin, input);
	if (check_num(input)) {
		c3 = stof(input, &sz);
	}
	
	Eigen::Vector3f n_1 = c1*ax.normalized() + +c2*ax1.normalized() + c3*ax2.normalized();
	n_1.normalize();
	Eigen::Vector4f n(n_1(0), n_1(1), n_1(2), 0);
	pcl::demeanPointCloud(*sceneCloud, centroid, *sceneCloud);
	pcl::demeanPointCloud(*RNNF_color, centroid, *RNNF_color);

	Eigen::Vector4f centroidT; 	pcl::compute3DCentroid(*templateCloud, centroidT);
	pcl::demeanPointCloud(*templateCloud, centroid, *templateCloud);
	pcl::demeanPointCloud(*dist_color, centroid, *dist_color);
	pcl::demeanPointCloud(*kappa_color, centroid, *kappa_color);
	pcl::demeanPointCloud(*DDIS_refined_NNF, centroid, *DDIS_refined_NNF);

	pcl::CorrespondencesPtr result_corrs(new pcl::Correspondences());
	float multiplier = 2;
	cout << "choose Number of scene radiuses from Center \n";
	std::getline(cin, input);
	if (check_num(input)) {
		multiplier = stof(input, &sz);
	}

	Eigen::Vector4f ntrans = n * multiplier * radi.norm();
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	if (multiplier != 0) {
		transform_1(0, 3) = ntrans(0);
		transform_1(1, 3) = ntrans(1);
		transform_1(2, 3) = ntrans(2);
	}
	XYZCloud cloud;
	transformPointCloud(*templateCloud, cloud, transform_1);
	toPCLPointCloud2(cloud, Tmesh->cloud);
	transformPointCloud(*dist_color, *dist_color, transform_1);
	transformPointCloud(*kappa_color, *kappa_color, transform_1);
	transformPointCloud(*DDIS_refined_NNF, *DDIS_refined_NNF, transform_1);

	toPCLPointCloud2(*sceneCloud, Qmesh->cloud);
	bool show = true;

	cout << "choose precentage of correspondences to show \n";
	std::getline(cin, input);
	float precent = 100;
	if (check_num(input)) {
		precent = stof(input, &sz);
	}

	cout << "enter base point size from 1-20:";
	std::getline(cin, input);
	float template_point_size = 1;
	if (check_num(input)) {
		template_point_size = stof(input, &sz);
	}

	cout << "enter point size for correspondences from 1-20:";
	float point_size_corrs = 20;
	std::getline(cin, input);
	if (check_num(input)) {
		point_size_corrs = stof(input, &sz);
	}

		int v4(0);
		//pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> scn_h(sceneCloud, 0, 255, 0);
		PointCloud<PointXYZRGB>::Ptr shownCorrs(new PointCloud<PointXYZRGB>);
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("3D Viewer 1"));
		viewer2->setBackgroundColor(255, 255, 255);
		viewer2->initCameraParameters();
		viewer2->addPolygonMesh(*Tmesh, "temp1");		
		viewer2->addPolygonMesh(*Qmesh, "scn1");
		viewer2->addPointCloud(RNNF_color, "RNNF");
		viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, "temp1");
		viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, "RNNF");
		viewer2->addPointCloud(dist_color, "dist");
		viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, "dist");


		//viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "temp1");
		//Add template
		viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, "scn1");

		//viewer2->addCorrespondences<PointXYZ>( templateCloud, sceneCloud, *result_corrs,"corrs", v4);
		cout << "Showing top " << closest_matches.size()*precent / 100 << "Correspondences\n";
		bool alter = false;
		cout << "pre correspondece showing " << closest_matches.size() << ".\n";
		for (size_t i = 0; i < closest_matches.size(); ++i)
		{
			if ((rand() % 100 < precent)&& (closest_matches.at(i)<MAXUINT32)){

				int r = lcmap[i % 1000][0];				int g = lcmap[i % 1000][1];    int b = lcmap[i % 1000][2];
				pcl::PointXYZRGB  p_src;
				copyPoint(DDIS_refined_NNF->at(i), p_src);
				p_src.r = r; p_src.g = g; p_src.b = b;
				pcl::PointXYZRGB  p_tgt;
				if (!(DDIS_refined_NNF->at(i).label < Qkeypoints->size())) continue;
				copyPoint(sceneCloud->at(DDIS_refined_NNF->at(i).label), p_tgt);
				p_tgt.r = r; p_tgt.g = g; p_tgt.b = b;
				//shownCorrs->push_back(p_src);
				shownCorrs->push_back(p_tgt);
				// Generate a unique string for each line 
				std::stringstream ss("line");
				ss << i;
				std::stringstream sss("spheresource");
				sss << i;
				std::stringstream ssss("spheretarget");
				ssss << i;
				if (alter)
				{
					viewer2->addLine(p_src, p_tgt, r, g, b, sss.str());
					viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, sss.str());
				}
				else
				{
					viewer2->addLine(p_src, p_tgt, r, g, b, ss.str());
					viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, ss.str());
				}
				alter = !alter;
			}
		}
		cout << "pre correspondece showing 2" << result_corrs->size() << ".\n";
		string corrs = "Corrs";
		viewer2->addPointCloud(shownCorrs, corrs);
		viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, corrs);

		//savePointCloudToFile(shownCorrs, "snapshot.ply");
		viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_corrs, corrs);
		viewer2->setWindowBorders(true);
		string txt = "Showing " + to_string(shownCorrs->size() / 2) + " highest curvature matches out of " + to_string(sceneCorr->size());
		viewer2->resetCamera();
		viewer2->addText(txt, 0, 0, 18, 0, 0, 0, corrs);
		while (!viewer2->wasStopped())
		{
			viewer2->spinOnce();
		}
		
	cout << "post correspondece showing " << result_corrs->size() << ".\n";*/

}

//Takes a snapshot of the simalarity function/chosen window on the cloud
void FeatureCloud::snapshot() {
	PointCloud<PointXYZRGB>::Ptr label_color_cloud = visualizeCloud<PointXYZI>(resultlabel, 1);
	PointCloud<PointXYZRGB>::Ptr similarity_color_cloud = visualizeCloud<PointXYZI>(similarityCloud, 0);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerL(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_l(label_color_cloud);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler(similarity_color_cloud);
	int v2(0), v3(0);
	viewerL->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewerL->createViewPort(0.0, 0.0, 0.5, 1.0, v3);
	viewerL->setBackgroundColor(255, 255, 255, v2);
	viewerL->setBackgroundColor(255, 255, 255, v3);
	viewerL->addText("Similarity Score Heat Map", 60, 10, 18, 0, 0, 0, "DDIS_2", v2);
	string id = "DDIS";
	string id_l = "NNF";
	cout << "showing NNF" << endl;
	viewerL->addPointCloud<pcl::PointXYZRGB>(similarity_color_cloud, handler, id, v2);
	viewerL->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id, v2);
	viewerL->setWindowBorders(true);
	viewerL->setSize(960, 540);
	string id_sc = "SceneC";
	viewerL->addPointCloud(label_color_cloud, handler_l, id_sc, v3);
	viewerL->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_sc, v3);

	viewerL->resetCamera();
	viewerL->spinOnce(1000);

	string shot1 = P.T.path +"_"+ P.S.path +"_"+P.feature+"_"+ftos(P.S.f.searchRadius,3)+"_"+ftos(P.T.f.searchRadius, 3)+"_"+ P.sim+".jpg";
	viewerL->saveScreenshot(P.result_dir + "\\" + shot1);

}

void FeatureCloud::show_registered(){
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerL(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewerL->initCameraParameters();
	int v2(0);
	viewerL->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	int v3(0);
	viewerL->createViewPort(0.0, 0.0, 0.5, 1.0, v3);
	viewerL->setBackgroundColor(0, 0, 0, v2);
	viewerL->setBackgroundColor(0, 0, 0, v3);

	string id_R = "RANSAC";
	string id_R_m = "RANSACM";
	string id_I = "ICP";
	string id_I_m = "ICPM";

	cout << "showing NNF" << endl;
	visualization::PointCloudColorHandlerCustom<PointXYZ> ransac_h(Tkeypoints, 255, 0, 255);
	visualization::PointCloudColorHandlerCustom<PointXYZ> icp_h(Tc, 255, 0, 255);

	viewerL->addPointCloud(Tkeypoints, ransac_h, id_R, v3);
	viewerL->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, id_R, v3);

	viewerL->addPolygonMesh(*Qmesh, id_R_m, v3);
	viewerL->addPointCloud(Tc, icp_h, id_I, v2);
	viewerL->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, id_I, v3);

	viewerL->addPolygonMesh(*Qmesh, id_I_m, v2);
	viewerL->addText("RANSAC", 60, 10, 18, 255, 0, 0, "DDIS_2", v3);
	viewerL->addText("ICP", 60, 10, 18, 255, 0, 0, "DDIS", v2);

	while (!viewerL->wasStopped())
	{
		viewerL->spinOnce();
	}

}


//DEBUG FEATURES
vector<int> FeatureCloud::Refined_GT_map(vector<barycentric_polygon_s> gt_map) {
	vector<int> new_gt(gt_map.size());
	for (int i = 0; i < gt_map.size(); i++) {
		barycentric_polygon_s pol_bar = gt_map.at(i);
		Vertices poly = Qmesh->polygons.at(pol_bar.index);
		new_gt.at(i)=barycentric_closest(pol_bar, poly);
	}
	return new_gt;
};

vector<int> FeatureCloud::GT_indices(vector<barycentric_polygon_s> gt_map) {
	vector<int> new_gt;
	vector<bool> in(Qc->size(), false);
	for (int i = 0; i < gt_map.size(); i++) {
		barycentric_polygon_s pol_bar = gt_map.at(i);
		Vertices poly = Qmesh->polygons.at(pol_bar.index);
		for (int j = 0; j < 3; j++) {
			if (!in.at(poly.vertices[j])) {
				in.at(poly.vertices[j]) = true;
				new_gt.push_back(poly.vertices[j]);
			}
		}
	}
	return new_gt;
};

vector<int> FeatureCloud::patch_indices(int i) {
	pcl::PointIndices::Ptr scene_indices(new pcl::PointIndices());
	std::vector<float> pointRadiusSquaredDistance;
	pcl::ExtractIndices<pcl::PointXYZ>::Ptr extract(new ExtractIndices<pcl::PointXYZ>);
	extract->setNegative(false);
	extract->setInputCloud(Qkeypoints);
	cout << "blah!\n";
	uint32_t M;
	vector<ind_r_s> Scene_P_D;
	extractTemplateCandidate(i, Scene_P_D, extract,T_f.RG,M);
	return scene_indices->indices;
};

float FeatureCloud::gt_scene_ratio(vector<int> scene_indices, vector<int> gt_indices) {
	vector<bool> gt_ind(Qc->size(), false);
	for (int i = 0; i < gt_indices.size(); i++) {
		gt_ind.at(gt_indices.at(i)) = true;
	}
	int counter = 0;
	for (int i = 0; i < scene_indices.size(); i++) {
		if (gt_ind.at(scene_indices.at(i))) counter++;
	}
	return float(counter / float(gt_indices.size()));
}

pcl::PointCloud<NNFX<5>>::Ptr FeatureCloud::cutnnf(vector<int> gt_indices) {
	PointCloud<NNFX<5>>::Ptr result= PointCloud<NNFX<5>>::Ptr(new PointCloud<NNFX<5>>());;
	for (int i = 0; i < gt_indices.size(); i++) {
		result->push_back(nnf5->at(gt_indices.at(i)));
	}
	return result;
}

XYZCloud::Ptr FeatureCloud::cutgt(vector<int> gt_indices) {
	XYZCloud::Ptr result = XYZCloud::Ptr(new XYZCloud());;
	for (int i = 0; i < gt_indices.size(); i++) {
		result->push_back(Qc->at(gt_indices.at(i)));
	}
	return result;
}

//DEBUG FEATURES
distance_stat FeatureCloud::Distance_Difference(int center, vector<int> gt_map) {
	distance_stat result;	int N = Tkeypoints->size();
	Q_G.resetDist(P.meshtype != NORMAL);
	float R;
	vector<float> TDists = getTemplateDistances(P.pMode,T_f.ind,R);
	vector<float> QDists(Tkeypoints->size());
	vector<float> Diffs(Tkeypoints->size());
	Q_G.LimitedSSSP(center, INFINITY, P.meshtype != NORMAL);
	for (int i = 0; i < N; i++) {
		QDists.at(i) = Q_G.getVertDist(gt_map.at(i), P.meshtype != NORMAL);
		Diffs.at(i) = abs(QDists.at(i) - TDists.at(i));
		result.mu += Diffs.at(i);
	}
	result.mu /= float(N);
	for (int i = 0; i < N; i++) {
		result.sigma += pow((Diffs.at(i)-result.mu),2);
	}
	result.sigma = sqrt(result.sigma/float(N));
	return result;
}

vector<distance_stat> FeatureCloud::distance_stats(PointCloud<Histogram<5>>::Ptr distogram, vector<int> gt_map) {
	PointCloud<Histogram<5>>::Ptr min_distances(new PointCloud<Histogram<5>>());
	copyPointCloud(*distogram, *min_distances);
	vector<distance_stat> result(5);
	vector<bool> exact(gt_map.size(),false), lessE(gt_map.size(), false), less2E(gt_map.size(), false),inrange(gt_map.size(), false);
	
	for (int i = 0; i < gt_map.size(); i++) {
		result.at(0).mu += min_distances->at(gt_map.at(i)).histogram[0];
		if (min_distances->at(gt_map.at(i)).histogram[0] < 2) { result.at(0).total_inrange++; inrange.at(i) = true; }
		if (min_distances->at(gt_map.at(i)).histogram[0] < 0.01) { result.at(0).exact++; exact.at(i) = true; }
		else if (min_distances->at(gt_map.at(i)).histogram[0] < 1) { result.at(0).lessE++; lessE.at(i) = true;}
		else if (min_distances->at(gt_map.at(i)).histogram[0] < 2) { result.at(0).less2E++; less2E.at(i) = true;}
	}

	for (int i = 0; i < gt_map.size(); i++) {
		for (int j = 1; j < 5; j++) {
			if (min_distances->at(gt_map.at(i)).histogram[j] < 2) { result.at(j).total_inrange++; inrange.at(i) = true; }
			min_distances->at(gt_map.at(i)).histogram[j] = distogram->at(gt_map.at(i)).histogram[j-1]; 
			if (distogram->at(gt_map.at(i)).histogram[j] < 0.01) { result.at(j).exact++; exact.at(i) = true;}
			else if ((distogram->at(gt_map.at(i)).histogram[j] < 1)&&(!(exact.at(i)|| lessE.at(i)))) {
				result.at(j).lessE++; lessE.at(i) = true;
			}
			else if ((distogram->at(gt_map.at(i)).histogram[j] < 2) && 
				    (!(exact.at(i) || lessE.at(i)|| less2E.at(i)))) {
				result.at(j).less2E++; less2E.at(i) = true;
			}
			if ((min_distances->at(gt_map.at(i)).histogram[0] < 2) && !inrange.at(i)) { result.at(j).total_inrange++; inrange.at(i) = true; }
			if (min_distances->at(gt_map.at(i)).histogram[j] > distogram->at(gt_map.at(i)).histogram[j]) {
				min_distances->at(gt_map.at(i)).histogram[j] = distogram->at(gt_map.at(i)).histogram[j];
			}
			result.at(j).mu += min_distances->at(gt_map.at(i)).histogram[j];
		}
	}
	
	for (int j = 0; j < 5; j++) {
		result.at(j).mu /= gt_map.size();
		result.at(j).exact /= gt_map.size();
		result.at(j).lessE /= gt_map.size();
		result.at(j).less2E /= gt_map.size();
		result.at(j).total_inrange /= gt_map.size();
		float mu = result.at(j).mu;
		for (int i = 0; i < gt_map.size(); i++) {
			result.at(j).sigma += pow(mu - min_distances->at(gt_map.at(i)).histogram[j], 2);
		}
		result.at(j).sigma = sqrt(result.at(j).sigma / gt_map.size());
	}
	return result;
}

NNFX<5> FeatureCloud::extract_top() {
	NNFX<5> result;
	XYZICloud::Ptr tempsim(similarityCloud);
	float global_max = 0;
	for (int i = 0; i < 5; i++) {
		float max = 0;
		for (int j = 0; j < tempsim->size(); j++) {
			if (tempsim->at(j).intensity > global_max) global_max = tempsim->at(j).intensity;
			if (tempsim->at(j).intensity > max) {
				max = tempsim->at(j).intensity;
				result.labels[i] = j;
				result.distance[i] = max;
			}
		}
		tempsim->at(result.labels[i]).intensity = 0;
	}
	for (int i = 0; i < 5; i++) {
		result.distance[i] *= ((float)255 / global_max);
	}

	return result;
};

HistogramInt<5> FeatureCloud::exact_match_counter(vector<int> gt_map) {
	HistogramInt<5> result;
	for (int i = 0; i < 5; i++) {
		result.counter[i] = 0;
		for (int j = 0; j < nnf5->size(); j++) {
			int match = nnf5->at(j).labels[i];
			if (j == match) {
				result.counter[i]++;
			}
		}
	}
	return result;
}

vector<matches_stat_s> FeatureCloud::exact_match_counter(vector<barycentric_polygon_s> gt_map) {
	vector<matches_stat_s> result(5);
	float Size = gt_map.size();
	vector<vector<bool>> exact(5,vector<bool>(nnf5->size(), false));
	vector<vector<bool>> lessone(5, vector<bool>(nnf5->size(), false));
	vector<vector<bool>> lesstwo(5, vector<bool>(nnf5->size(), false));

	vector<bool> exactall(nnf5->size(), false);
	vector<bool> lessoneall(nnf5->size(), false);
	vector<bool> lesstwoall(nnf5->size(), false);

	for (int j = 0; j < nnf5->size(); j++) {//j runs on Q vertices
		Q_G.resetDist(P.meshtype != NORMAL);
		float e = P.T.mean_edge;
		Q_G.LimitedSSSP(j, e * 3, P.meshtype != NORMAL);
		for (int i = 0; i < 5; i++) {
			int match = nnf5->at(j).labels[i];//an index of the vertex on T
			barycentric_polygon_s pol_bar = gt_map.at(match);
			Vertices poly = Qmesh->polygons.at(pol_bar.index); //vertices on Q which are GT
			float min_dist = min(min(Q_G.getVertDist(poly.vertices[0], P.meshtype != NORMAL), Q_G.getVertDist(poly.vertices[1], P.meshtype != NORMAL)), Q_G.getVertDist(poly.vertices[2], P.meshtype != NORMAL));
			if ((j == poly.vertices[0]) || (j == poly.vertices[1]) || (j == poly.vertices[2])) exact.at(i).at(j) = true;
			if (min_dist <= e) lessone.at(i).at(j) = true;
			if (min_dist <= 2*e) lesstwo.at(i).at(j) = true;
		}
	}

	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < nnf5->size(); j++) {
			if (exact.at(i).at(j)) exactall.at(j) = true;
			if (lessone.at(i).at(j)) lessoneall.at(j) = true;
			if (lesstwo.at(i).at(j)) lesstwoall.at(j) = true;

			if (exactall.at(j)) result.at(i).exact++;
			if (lessoneall.at(j)) result.at(i).lessone++;
			if (lesstwoall.at(j)) result.at(i).lesstwo++;
		}
		result.at(i).exact /= Size;
		result.at(i).lessone /= Size;
		result.at(i).lesstwo /= Size;
	}

	return result;
}

vector<float> FeatureCloud::top_distances(int center, NNFX<5> top, float maxdist) {
	Q_G.resetDist(P.meshtype != NORMAL);
	Q_G.LimitedSSSP(center, maxdist, P.meshtype != NORMAL);
	vector<float> result(5,0);
	for (int i = 0; i < top.labelSize(); i++) {
		result.at(i) = Q_G.getVertDist(top.labels[i], P.meshtype != NORMAL);
	}
	return result;
}

void FeatureCloud::save_distance_matrix(string path, nnfDest d) {
	/*fstream Dmatfile(path, std::fstream::out);
	focal_s* f = &T_f;
	uint32_t Dsize = Tc->size();
	if (d == R) { 
		Dsize = result->size();
		f = &R_f;
	}
	for (int i = 0; i < Dsize; i++) {
		Dmatfile << fD(i,0);
		for (int j = 1; j < Dsize; j++) {
			Dmatfile <<","<< f.at(i).at(j);
		}
		Dmatfile << endl;
	}
	Dmatfile.close();*/
	//f->D.clear();//done due to memory constraints
}

void FeatureCloud::createRDistanceTree() {
	R_G.mesh2graph(Rmesh, P.meshtype == SUPER);
	cout << 1<<endl;
	R_f.ind = 0;//the way the algorithm is built 0 is always the center
	cout << 1 << endl;
	R_G.resetDist();
	Q_G = R_G;
	vector<int> indices = R_G.LimitedSSSP(0, INFINITY, P.meshtype != NORMAL);
	R_distances.resize(result->size());
	cout << 1 << endl;
	R_distances.at(R_f.ind).resize(result->size());
	for (int j = 0; j < result->size(); j++) {
		R_distances.at(R_f.ind).at(j) = R_G.getVertDist(j, P.meshtype != NORMAL);
	}
};

vector<int> FeatureCloud::extract_band_indices(float distance, float Ewidth) {
	vector<int> indices;
	for (int j = 0; j < R_distances.size(); j++) {
		float D = R_distances.at(R_f.ind).at(j);
		if (abs(D - distance) < Ewidth*P.T.stat.MeanResolution) {
			indices.push_back(j);
		}
	}
	return indices;
	cout << "band indices extracted\n";
}

void FeatureCloud::createresultDmatrix() {
	R_G.mesh2graph(Rmesh, P.meshtype == SUPER);
	R_f = R_G.graphFocal(true, P.meshtype != NORMAL);
};

void FeatureCloud::load_correspondences(string path,bool throw_header) {
	vector<CorrespondencesPtr> result;
	if (path.compare(path.size() - 4, path.size(),"corr")==0) {
		result = load_feature_corrs_from_corr(path, throw_header);
	}
	else {
		result = load_feature_corrs_from_csv(path, throw_header);
	}
	Result.resize(result.size());
	for (int i = 0; i < result.size(); i++) {
		for (int j = 0; j < result.at(i)->size(); j++) {
			Result.at(i).push_back(ind_r_s(result.at(i)->at(j).index_match, result.at(i)->at(j).distance));
			feature_point_inds.push_back(result.at(i)->at(j).index_query);
		}
	}
	if (P.save_similarity_clouds) {
		for (int i = 0; i < result.at(0)->size(); i++) {
			load_feature_similarity(i);
		}
	}
	if (P.save_feature_patches) {
		for (int i = 0; i < result.at(0)->size(); i++) {
			load_feature_patch(i);
		}
	}
};

/*void FeatureCloud::load_result(string path) {
	CorrespondencesPtr result = load_result_corrs_from_csv(path);
	Result.resize(result.size());
	for (int i = 0; i < result.size(); i++) {
		for (int j = 0; j < result.at(i)->size(); j++) {
			Result.at(i).push_back(ind_r_s(result.at(i)->at(j).index_match, result.at(i)->at(j).distance));
		}
	}
	if (P.save_similarity_clouds) {
		for (int i = 0; i < result.at(0)->size(); i++) {
			load_feature_similarity(i);
		}
	}
	if (P.save_feature_patches) {
		for (int i = 0; i < result.at(0)->size(); i++) {
			load_feature_patch(i);
		}
	}
};*/


PolygonMesh::Ptr FeatureCloud::extract_feature_patch(int i, vector<ind_r_s>& scene_P_D, nnfDest Dest=T) {
	XYZCloud::Ptr cloud;  vector<vector<float>>* D;
	XYZNCloud::Ptr out;
	PolygonMeshPtr S;
	cloud_param_s *p;
	switch (Dest) {
	case Q:   S = Qmesh; D = &DQ; break;
	case T:   S = Tmesh; D = &DT; break;
	}

	for (int j = 0; j < D->at(0).size(); j++) {
		if (D->at(i).at(j) < R_thresh) scene_P_D.push_back(ind_r_s(j, D->at(i).at(j)));
	}
	PolygonMesh::Ptr Patch = cut_mesh_from_points(S, scene_P_D);
	return Patch;
};

void FeatureCloud::extract_all_feature_patches(string path){
	for (int i = 0; i < feature_point_inds.size(); i++) {
		vector<ind_r_s> Scene_P_D;
		PolygonMesh::Ptr X = extract_feature_patch(feature_point_inds.at(i), Scene_P_D);
		io::savePLYFile(path + "\\feature_patch" + to_string(i) + ".ply", *X);
	}
};



PointCloud<Histogram<5>>::Ptr FeatureCloud::match_distances(vector<int> gt_map, float maxdist) {
	PointCloud<Histogram<5>>::Ptr result(new PointCloud<Histogram<5>>());
	result->resize(nnf5->size());
	vector<int> indicator(nnf5->size(), INFINITY);//Q->T Map
	
	//creating a reverse GT map - from Q->T
	for (int j = 0; j < Tkeypoints->size(); j++) {
		indicator.at(gt_map.at(j)) = j;
	}

	//Setting the entire distances to infinity and logging the list
	for (int j = 0; j < nnf5->size(); j++) {
		for (int i = 0; i < 5; i++) {
			result->at(j).histogram[i] = INFINITY;
		}
	}

	//logging the distances
	for (int j = 0; j < nnf5->size(); j++) {
		if (indicator.at(j) != INFINITY) {
			T_G.resetDist(P.meshtype != NORMAL);
			T_G.LimitedSSSP(indicator.at(j), maxdist, P.meshtype != NORMAL);
			for (int i = 0; i < 5; i++) {
				result->at(j).histogram[i] = T_G.getVertDist(nnf5->at(j).labels[i], P.meshtype != NORMAL);
			}
		}
	}
	T_G.resetDist(P.meshtype != NORMAL);
	T_G.LimitedSSSP(T_f.ind,INFINITY, P.meshtype != NORMAL);
	return result;
};

int FeatureCloud::Triangulate_match(int src, int detectors) {
	return 0;
};

void FeatureCloud::reject_outliers() {

};

void FeatureCloud::Densify_template_match(int f_ind) {
	int src = feature_point_inds.at(f_ind);//the src point on the partial object
	int dst = feature_point_map.at(f_ind);//the src point on the full object
	float R = F_R.at(f_ind).second;//setting the current point maximal radius
	uint32_t M;
	vector<ind_r_s> scene_P_D;//holds indices of the piece and distance for each one.
	extractTemplateCandidate(dst, scene_P_D, extract, R,M);
	std::vector<float> r_j(M, INFINITY);
	std::vector<uint32_t> Kappa(Tkeypoints->size());
	std::vector<uint32_t> Kappa_j(M);
	std::vector<uint32_t> Patch_Buddies(M);
	vector<float> mindistances(Tkeypoints->size(), INFINITY);//distance difference of the minimal corresponding point	
	vector<uint32_t> closest_temp(Tkeypoints->size(), MAXUINT32);//holds the correspondences with minimal distance difference
	extractCandidatestats(scene_P_D, M, TDistances.at(F_R.at(f_ind).first), mindistances, Kappa, Kappa_j, Patch_Buddies, r_j, closest_temp, DIS_cummulative->at(dst), F_R.at(f_ind).first);
	for (int i = 0; i < Tkeypoints->size(); i++) {
		if ((mindistances.at(i) / P.S.diam) < 0.005) {
			if ((Result.at(i).size() == 0)||(mindistances.at(i) < Result.at(i).at(0).second)) {
				ind_dis_s r(closest_temp.at(i), mindistances.at(i));
				if (Result.at(i).size() == 0) Result.at(i).push_back(r);
				Result.at(i).at(0).first = closest_temp.at(i);
				Result.at(i).at(0).second = mindistances.at(i);
			}
		}
	}
}

vector<vector<ind_dis_s>> FeatureCloud::Densify_correspondences() {
	for (int i = 0; i < feature_point_inds.size(); i++) {
		Densify_template_match(i);
	}
	return Result;
}

void FeatureCloud::save_similarity(string path) {
	std::fstream hsim_out(path + "_h_sim.csv", std::fstream::out);
	std::fstream msim_out(path + "_m_sim.csv", std::fstream::out);
	std::fstream lsim_out(path + "_l_sim.csv", std::fstream::out);

	for (int i = 0; i<num_samples; i++) {
		vector<float> HS = Fsimilarity.at(i);
		vector<float> MS = MSFsimilarity.at(i);
		vector<float> LS = LSFsimilarity.at(i);

		hsim_out << HS.at(0);


		for (int j = 1; j < model_vertices; j++) {
			hsim_out << "," << HS.at(j);
		}
		if (P.similarity == MSWDIS || P.similarity == TRIDIS || P.similarity == TRIDISP) {
			msim_out << MS.at(0);
			for (int j = 1; j < model_vertices; j++) {
				msim_out << "," << MS.at(j);
			}
			msim_out << "\n";

		}
		if (P.similarity == TRIDIS || P.similarity == TRIDISP) {
			lsim_out << LS.at(0);

			for (int j = 1; j < model_vertices; j++) {
				lsim_out << "," << LS.at(j);
			}
			lsim_out << "\n";
		}
		hsim_out << "\n";
	}
};
