
#include <cloud_surface_visl.hpp>

using namespace pcl;
using namespace std;

void FastTriangulation(XYZCloud::Ptr cloud, NormalCloud::Ptr Normals, PolygonMesh::Ptr mesh, float searchRadius, float Mu, int NN) {
	PointCloud<PointNormal>::Ptr cloud_with_normals(new PointCloud<PointNormal>);
	// Concatenate the XYZ and normal fields*
	concatenateFields(*cloud, *Normals, *cloud_with_normals);

	// Create search tree*
	search::KdTree<PointNormal>::Ptr tree2(new search::KdTree<PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	GreedyProjectionTriangulation<PointNormal> gp3;
	PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(searchRadius);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(NN);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(*mesh);

}

void MLS(XYZCloud::Ptr cloud, NormalCloud::Ptr Normals, XYZCloud::Ptr cloudout,
	float searchR, float upsamplingR, float upsamplingStep) {
	copyPointCloud(*cloud, *cloudout);
	MLS_func(cloud, cloudout, searchR, upsamplingR, upsamplingStep);
	computeNormals(cloudout, Normals, 10, true);
	PointCloud<PointNormal>::Ptr cloudnormals(new PointCloud<PointNormal>);
}

void MLS_func(XYZCloud::Ptr cloud, XYZCloud::Ptr out, float searchR, float upsamplingR, float upsamplingStep) {
	MovingLeastSquares<PointXYZ, PointXYZ> mls;
	mls.setInputCloud(cloud);
	mls.setSearchRadius(0.01);
	mls.setPolynomialFit(true);
	mls.setPolynomialOrder(2);
	mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
	mls.setUpsamplingRadius(0.005);
	mls.setUpsamplingStepSize(0.003);
	PointCloud<PointXYZ>::Ptr cloud_smoothed(new PointCloud<PointXYZ>());
	mls.process(*out);
}

PolygonMesh::Ptr cut_mesh1(PolygonMesh::Ptr inmesh, vector<barycentric_polygon_s> gt) {
	PolygonMesh::Ptr mesh(new PolygonMesh());
	XYZCloud::Ptr meshcloud(new XYZCloud());
	XYZCloud::Ptr newmeshcloud(new XYZCloud());
	fromPCLPointCloud2(inmesh->cloud, *meshcloud);
	vector<int> map(meshcloud->size(), 0);
	vector<bool> vertin(meshcloud->size(), false);
	int ind = 0;
	for (int i = 0; i < gt.size(); i++) {
		Vertices poly = inmesh->polygons.at(gt.at(i).index);
		for (int j = 0; j < 3; j++) {
			if (!vertin.at(poly.vertices[j])) {//entering a new vertice to the cloud and mapping accordingly
				newmeshcloud->push_back(meshcloud->at(poly.vertices[j]));
				vertin.at(poly.vertices[j]) = true;
				map.at(poly.vertices[j]) = ind++;
			}
		}
	}
	for (int i = 0; i < inmesh->polygons.size(); i++) {
		Vertices poly = inmesh->polygons.at(i);
		bool push = true;
		for (int j = 0; j < 3; j++) {
			if (!vertin.at(poly.vertices[j])) {
				push = false;
				break;
			}
			poly.vertices[j] = map.at(poly.vertices[j]);
		}
		if (push) mesh->polygons.push_back(poly);
	}

	toPCLPointCloud2(*newmeshcloud, mesh->cloud);
	return mesh;
}

PolygonMesh::Ptr cut_mesh(PolygonMesh::Ptr inmesh, vector<barycentric_polygon_s> gt) {
	PolygonMesh::Ptr mesh(new PolygonMesh());
	XYZCloud::Ptr meshcloud(new XYZCloud());
	XYZCloud::Ptr newmeshcloud(new XYZCloud());
	fromPCLPointCloud2(inmesh->cloud, *meshcloud);
	vector<int> map(meshcloud->size(), 0);
	vector<bool> vertin(meshcloud->size(), false);
	int ind = 0;
	for (int i = 0; i < gt.size(); i++) {
		Vertices poly = inmesh->polygons.at(gt.at(i).index);
		for (int j = 0; j < 3; j++) {
			if (!vertin.at(poly.vertices[j])) {//entering a new vertice to the cloud and mapping accordingly
				newmeshcloud->push_back(meshcloud->at(poly.vertices[j]));
				vertin.at(poly.vertices[j]) = true;
				map.at(poly.vertices[j]) = ind++;
			}
			poly.vertices[j] = map.at(poly.vertices[j]);//changing polygon vertices according to new mapping;
		}
		mesh->polygons.push_back(poly);
	}
	toPCLPointCloud2(*newmeshcloud, mesh->cloud);
	return mesh;
}

PolygonMesh::Ptr cut_mesh_from_points(PolygonMesh::Ptr inmesh, vector<ind_r_s>& scene_P_D) {
	PolygonMesh::Ptr mesh(new PolygonMesh());
	XYZCloud::Ptr meshcloud(new XYZCloud());
	XYZCloud::Ptr newmeshcloud(new XYZCloud());
	fromPCLPointCloud2(inmesh->cloud, *meshcloud);
	vector<int> map(meshcloud->size(), 0);
	vector<bool> vertin(meshcloud->size(), false);
	int ind = 0;
	for (int i = 0; i < scene_P_D.size(); i++) {
		if (!vertin.at(scene_P_D.at(i).first)) {
			newmeshcloud->push_back(meshcloud->at(scene_P_D.at(i).first));
			vertin.at(scene_P_D.at(i).first) = true;
			map.at(scene_P_D.at(i).first) = ind++;
		}
	}
	//cout << inmesh->polygons.size() << endl;
	for (int i = 0; i < inmesh->polygons.size(); i++) {
		Vertices poly = inmesh->polygons.at(i);
		bool push = true;
		for (int j = 0; j < 3; j++) {
			if (!vertin.at(poly.vertices[j])) {
				push = false;
				break;
			}
			poly.vertices[j] = map.at(poly.vertices[j]);
		}
		if (push) mesh->polygons.push_back(poly);
	}

	toPCLPointCloud2(*newmeshcloud, mesh->cloud);
	return mesh;
}

void clean_mesh(PolygonMesh::Ptr mesh, XYZCloud::Ptr cloud, float resolution) {

	XYZCloud::Ptr meshcloud(new XYZCloud()),newcloud(new XYZCloud());
	fromPCLPointCloud2(mesh->cloud, *meshcloud);
	vector <Vertices> triangles;
	search::KdTree<PointXYZ>::Ptr kdtree(new search::KdTree<PointXYZ>);
	kdtree->setInputCloud(cloud);
	float R;
	vector<int> map(meshcloud->size());
	vector<int> pointIdxSearch(1);
	vector<float> pointSquaredDistance(1);

	for (int i = 0; i < meshcloud->size(); i++) {
		PointXYZ point = meshcloud->at(i);
		kdtree->nearestKSearch(point, 1, pointIdxSearch, pointSquaredDistance);
		if (i == 117919) {
			cout << "NN dist:" << pointSquaredDistance[0] << endl;
		}
		if (sqrt(pointSquaredDistance[0]) < resolution) { //pointsbelonging on the cloud
			newcloud->push_back(point);
			map.at(i) = newcloud->size() - 1;
		}
		else { //points to delete
			//cout << "Bad Vertex:" << i << endl;
			map.at(i) = -1;
		}
	}
	toPCLPointCloud2(*newcloud, mesh->cloud);
	for (int i = 0; i < mesh->polygons.size(); i++) {
		Vertices v = mesh->polygons.at(i);
		int v0 = v.vertices.at(0);
		int v1 = v.vertices.at(1);
		int v2 = v.vertices.at(2);

		if (!((map.at(v0) == -1) || (map.at(v1) == -1) || (map.at(v2) == -1))) {
			v.vertices.at(0) = map.at(v0);
			v.vertices.at(1) = map.at(v1);
			v.vertices.at(2) = map.at(v2);
			triangles.push_back(v);
		}
	}
	cout << "original triangles" << mesh->polygons.size();

	cout << "cropped triangles" << triangles.size();
	mesh->polygons = triangles;
}

void PoissonReconstruction(XYZCloud::Ptr cloud, NormalCloud::Ptr Normals, PolygonMesh::Ptr mesh, int depth) {
	PointCloud<PointNormal>::Ptr cloud_with_normals(new PointCloud<PointNormal>);
	concatenateFields(*cloud, *Normals, *cloud_with_normals);
	Poisson<PointNormal> poisson;
	poisson.setDepth(depth);
	poisson.setInputCloud(cloud_with_normals);
	poisson.reconstruct(*mesh);
}

PolygonMesh::Ptr ReconstructSurface(XYZCloud::Ptr cloud, NormalCloud::Ptr Normals, surface_recon_params_s params) {
	PolygonMesh::Ptr result(new PolygonMesh());
	if (params.Method == POISSON || params.Method == FTRIANGLE) {
		cout << "4\n";
		// Computing fpfh:
		cout << "Surface Reconstruction Commencing\n";
		if (params.Method == POISSON) {
			PoissonReconstruction(cloud, Normals, result, params.depth);
			if (params.clean) {
				clean_mesh(result, cloud, params.clean_multiplier * params.cloudResolution);
			}
		}

		else if (params.Method == FTRIANGLE) {
			FastTriangulation(cloud, Normals, result, 5 * params.cloudResolution);
		} else {
			cout << "Features method eneterd by user is illegal" << endl;
			exit(0);
		}
	}
	return result;
}

//this recieves mesh polygons and the number of vertices in a mesh and returns a vector which consists of the boundary vertices
boundary_label_s MeshBoundary(vector<Vertices>& in,int size,bool longest_only = true) {
	vector<vector<boundary_count_s>> detector(size);//a vector keeping track of edges. the 1st index indicates a vertex index. the second vector includes a field of connected vertices and the num ber of edges connected. vertices with an element with less than 2 are a boundary
	vector<boundary_label_s> boundaries;
	vector<bool> boundary(size);
	vector<bool> visited(size);
	boundary_label_s res;
	vector<int> result;
	int p[3];
	//counting edge reccurence on polygons
	for (int i = 0; i < in.size(); i++) { 
		//cout << i << endl;
		for (int j = 0; j < 3; j++) {
			p[j] = in.at(i).vertices.at(j);//filling in the vertices of the polygon
		}
		for (int j = 0; j < 2; j++) {//going over all 3 vertices in the polygon
			int lsize = detector.at(p[j]).size();
			for (int k = j+1; k < 3; k++) {//going over the vertices connected
				bool done = false;
				if (lsize != 0) {
					for (int l = 0; l < lsize; l++) {
						if (detector.at(p[j]).at(l).ind == p[k]) {//checking if this edge already appeared
							done = true; detector.at(p[j]).at(l).count++;//if the edge was found we don't need to check the list for this vertex - move on to the next
							break;
						}
					}
				}
				if (!done) {
					boundary_count_s s; s.ind = p[k]; s.count = 1;
					detector.at(p[j]).push_back(s);//adding the edge to the vertex list
					boundary_count_s t; t.ind = p[j]; t.count = 1;
					detector.at(p[k]).push_back(t);//addign the edge to the connected vertex list
				}
				else {
					for (int l = 0; l < detector.at(p[k]).size(); l++) {
						if (detector.at(p[k]).at(l).ind == p[j]) {
							detector.at(p[k]).at(l).count++; break;//adding to the count of edges connecting the 2 vertices
						}
					}
				}
			}
		}
	}
	//marking vertices with only 1 shared polygon to be boundary
	for (int i = 0; i < size; i++) {
		for (int j = 0; j < detector.at(i).size(); j++) {
			if (detector.at(i).at(j).count < 2) {
				result.push_back(i); 
				boundary.at(i) = true;
				break;
			}
		}
	}
	vector<int> labels_temp(size);

	int label = -1;
	int longest_chain = 0;
	int longest_chain_label = 0;

	for (int i = 0; i < result.size(); i++) {//creating a boundary index list
		int curr = result.at(i);
		if (!visited.at(curr)) {
			label++;
			boundary_label_s chain;
			vector<int> stack;
			visited.at(curr) = true;
			stack.push_back(curr);
			while (!(stack.empty())) {
				curr = stack.back(); stack.pop_back(); chain.ind.push_back(curr); chain.label.push_back(label);
				res.ind.push_back(curr);	res.label.push_back(label);
				for (int j = 0; j < detector.at(curr).size(); j++) {
					int ind2 = detector.at(curr).at(j).ind;
					if (boundary.at(ind2) && !visited.at(ind2)) {
						stack.push_back(ind2); visited.at(ind2) = true;
					}
				}
			}
			if (chain.ind.size() > longest_chain) { longest_chain = chain.ind.size(), longest_chain_label = label; };
			boundaries.push_back(chain);
		}
	}
	if (longest_only&& !boundaries.empty()) res = boundaries.at(longest_chain_label);
	return res;
}

void RemoveBoundary(XYZCloud::Ptr cloud, vector<Vertices>& poly, cloud_param_s& p) {
	boundary_label_s boundary = MeshBoundary(poly, cloud->size());
	vector<int> BoundIndices2 = Region_Growth(cloud, boundary.ind, p.k.salient_radius);
	pcl::ExtractIndices<pcl::PointXYZ>::Ptr extract(new ExtractIndices<pcl::PointXYZ>);
	cout << "3";
	extract->setNegative(true);
	extract->setInputCloud(cloud);
	PointIndices::Ptr scene_indices(new PointIndices());
	scene_indices->indices = BoundIndices2;
	extract->setIndices(scene_indices);
	extract->filter(*cloud);
};


float PolygonArea(Vertices poly, XYZCloud::Ptr cloud) {
	Eigen::Vector3f p1, p2, p3;
	p1 = cloud->at(poly.vertices.at(0)).getVector3fMap();
	p2 = cloud->at(poly.vertices.at(1)).getVector3fMap();
	p3 = cloud->at(poly.vertices.at(2)).getVector3fMap();
	Eigen::Vector3f v1 = p3 - p1;
	Eigen::Vector3f v2 = p2 - p1;
	return v1.cross(v2).norm() / 2;
}

float MeshArea(PolygonMesh::Ptr mesh, XYZCloud::Ptr cloud) {
	float result = 0;
	for (int i = 0; i < mesh->polygons.size(); i++) {
		result += PolygonArea(mesh->polygons.at(i), cloud);
	}
	return result;
}

vector<float> DistSumFunctional(vector<vector<float>> D) {
	vector<float> result;
	int vertnum = D.size();
	for (int i = 0; i < vertnum; i++) {
		result.push_back(accumulate(D.at(i).begin(), D.at(i).end(), 0.0));
	}
	return result;
}

vector<vector<uint32_t>> list_neighbors(vector<Vertices>& poly, int size,int neighbors) {
		vector<vector<bool>> detector(size, vector<bool>(size,false));//a vector keeping track of edges. the 1st index indicates a vertex index. the second vector includes a field of connected vertices and the num ber of edges connected. vertices with an element with less than 2 are a boundary
		vector<vector<uint32_t>> result(size, vector<uint32_t>());
		uint32_t trinum = poly.size();
		int p[3];
		//counting edge reccurence on polygons
		for (int i = 0; i < trinum; i++) {
			for (int j = 0; j < 3; j++) {
				p[j] = poly.at(i).vertices.at(j);//filling in the vertices of the polygon
			}
			for (int j = 0; j < 2; j++) {//going over all 3 vertices in the polygon
				for (int k = j + 1; k < 3; k++) {//going over the vertices connected
					if (!detector.at(p[j]).at(p[k])) {
						detector.at(p[j]).at(p[k]) = true;
						detector.at(p[k]).at(p[j]) = true;
						result.at(p[j]).push_back(p[k]);
						result.at(p[k]).push_back(p[j]);
					}
				}
			}
		}
		return result;
}

vector<uint32_t> meshMaximas(vector<float>& F, vector<vector<uint32_t>>& neigbor_list) {
	vector<uint32_t> result;
	int M = F.size();
	float val;
	for (int i = 0; i < M; i++) {
		val = F.at(i);
		int N = neigbor_list.at(i).size();
		result.push_back(i);
		for (int j = 0; j < N; j++) {
			if (F.at(neigbor_list.at(i).at(j)) > val) {
				result.pop_back();
				break;
			}
		}
	}
	sort(result.begin(), result.end());
	return result;
}

vector<uint32_t> geodesicMaximasupression(vector<float>& F, vector<uint32_t>& maximas, vector<vector<float>>& D,float diam, float thresh) {
	uint32_t M = maximas.size();
	float score, d;
	uint32_t ind;
	vector<bool> max_ind(M, true); MAXQ<ind_r_s> maxheap;
	for (int i = 0; i < M; i++) {
		maxheap.push(ind_r_s(i, F.at(maximas.at(i))));
	}
	vector<uint32_t> result;
	for (int i = 0; i < M; i++) {
		ind_r_s cand = maxheap.top();
		maxheap.pop();
		score = cand.second; ind = cand.first;
		if (max_ind.at(i)) {
			result.push_back(maximas.at(ind));
			for (int j = 0; j < M; j++) {
				if (D.at(maximas.at(ind)).at(maximas.at(j)) < thresh*diam)
						max_ind.at(j) = false;
			}
		}
	}
	sort(result.begin(), result.end());
	return result;
}

void surface_sampling(vector<uint32_t>& seeds, vector<vector<float>>& D, float diam, float thresh) {
	uint32_t M = D.at(0).size();
	vector<uint32_t> marked = seeds;
	vector<bool> ismarked(M,false);
	vector<uint32_t> unmarked;
	uint32_t next_seed;
	float rthresh = thresh * diam;
	vector<float> min_dist_v(M, INFINITY);
	float min_dist=INFINITY;
	uint32_t min_dist_ind= seeds.at(0);
	
	float dd;
	for (int i = 0; i < M; i++) {
		vector<float>& d = D.at(i);
		for (int j = 0; j < seeds.size(); j++) {
			dd = d.at(seeds.at(j));
			if (dd < rthresh) {
				break;
			}
 			else if (dd < min_dist_v.at(i)) {
				min_dist_v.at(i) = dd;
			}

		}
		if (dd < rthresh) {
			continue;
		}
		if (min_dist_v.at(i) < min_dist_v.at(min_dist_ind)) { unmarked.push_back(min_dist_ind); min_dist_ind = i; }
		else { 
			unmarked.push_back(i);
		}
	}
	if ((unmarked.size() != 0))
	unmarked.erase(unmarked.begin(), unmarked.begin()+1);

	while (unmarked.size()!=0) {
		seeds.push_back(min_dist_ind);
		min_dist_v.at(min_dist_ind) = INFINITY;
		next_seed = min_dist_ind;
		vector<uint32_t> current_unmarked = unmarked;
		min_dist_ind = current_unmarked.at(0);
		unmarked.clear();
		vector<float>& d = D.at(next_seed);
		int j = 0;
		for (j = 0; j < current_unmarked.size(); j++) {
			uint32_t ind = current_unmarked.at(j);
			dd = d.at(ind);
			if (dd >= rthresh) {
				min_dist_ind = current_unmarked.at(j);
				break;
			}
		}
		for (int i = j; i < current_unmarked.size(); i++) {
			uint32_t ind = current_unmarked.at(i);
			dd = d.at(ind);
			if (dd < rthresh) continue;
			if (min_dist_v.at(ind) > dd) {
				min_dist_v.at(ind) = dd;
			}
			if (min_dist_v.at(ind) < min_dist_v.at(min_dist_ind)) { 
				unmarked.push_back(min_dist_ind);  min_dist_ind = ind;
			}
			else {
				unmarked.push_back(ind);
			}
		}
	}
	sort(seeds.begin(), seeds.end());
}
