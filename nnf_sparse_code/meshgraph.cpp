#include <meshgraph.h>
using namespace std;
using namespace pcl;

wEdge::wEdge(int src, int dst, float weight) :source(src), dest(dst), wt(weight) {};
wEdge::wEdge(int src, int dst, float weight, int face, bool temp) :source(src), dest(dst), wt(weight) {
	if (temp) { face1 = face; }
	else { face2 = face; };
};
bool wEdge::operator <(const wEdge &E) {
	return (wt < E.wt);
};

adjVert::adjVert(int i) :id(i) { dist = INFINITY; mark = false; };

void adjVert::addEdge(wEdge edge) {
	for (int i = 0; i < adjlist.size(); i++) {
		if (adjlist.at(i).dest == edge.dest) {
			if (adjlist.at(i).face2 == -1) {
				adjlist.at(i).face2 = edge.face2;
			}
			if (adjlist.at(i).face1 == -1) {
				adjlist.at(i).face1 = edge.face1;
			}
			return;
		}
	}
	adjlist.push_back(edge);
};

bool wEdge::isboundary() {
	return ((face1 < 0) ||( face2 < 0));
}

bool adjVert::hasEdge(int dest) {
	for (int i = 0; i < adjlist.size(); i++) {
		if (adjlist.at(i).dest == dest) { return true; }
	}
	return false;
};

int adjVert::getEdgeNum() {
	return adjlist.size();
};

wEdge adjVert::getEdge(int i) {
	assert(i < getEdgeNum());
	return adjlist.at(i);
};

float adjVert::getDist() const{
	return dist;
};

int adjVert::getParent() const {
	return parent;
};

int graphMesh::getVertParent(int i, bool dual) {
	vector<adjVert>& graph = dual ? dualgraphadjlist : graphadjlist;
	return graph.at(i).getParent();
}

graphpath_s graphMesh::getPath(int i, bool dual) {
	graphpath_s result;
	result.chain.push_back(i);
	result.distance.push_back(getVertDist(i, dual));
	while (result.distance.back() != 0) {
		result.chain.push_back(getVertParent(result.chain.back(), dual));
		result.distance.push_back(getVertDist(result.chain.back(), dual));
	}
	return result;
}

bool adjVert::visit(adjVert& S, wEdge& E) {
	bool oldmark = mark;
	mark = true;
	if ((S.getDist() + E.wt) < getDist()) {
		parent = S.getID();
		dist = S.getDist() + E.wt;
	}
	return oldmark;
};

void adjVert::resetDist() {
	dist = INFINITY; mark = false; parent = id;
}

void adjVert::setSource() {
	dist = 0;
}

bool adjVert::operator<(adjVert V) {
	return (dist < V.getDist());
}

void graphMesh::mesh2graph(PolygonMesh::Ptr mesh,bool mixed_mesh) {
	meanedge = 0;
	float count = 0;
	//vector<Eigen::Triplet<int>> triplets;
	//vector<Eigen::Triplet<int>> tripletsd;
	//vector<Eigen::Triplet<int>> Dtriplets;
	//vector<Eigen::Triplet<int>> Dtripletsd;
	pcl::fromPCLPointCloud2(mesh->cloud, *Cloud);
	orig_vertex_size = Cloud->size();
	int a = (mixed_mesh) ?Cloud->size():0;
	if (mixed_mesh) copyPointCloud(*Cloud, *DualCloud);
	int orgsize = Cloud->size();
	for (int i = 0; i < Cloud->size(); i++) {
		graphadjlist.push_back(adjVert(i));
		if (mixed_mesh) dualgraphadjlist.push_back(adjVert(i));
	};
	for (int i = 0; i < mesh->polygons.size(); i++) {
		int dualgraphid =  i + a;
		dualgraphadjlist.push_back(adjVert(dualgraphid));
		PointXYZ barycenter;
		int v1 = mesh->polygons.at(i).vertices.at(0); int v2 = mesh->polygons.at(i).vertices.at(1); int v3 = mesh->polygons.at(i).vertices.at(2);
		PointXYZ p1 = Cloud->at(v1);
		PointXYZ p2 = Cloud->at(v2);
		PointXYZ p3 = Cloud->at(v3);
		//tripletsd.push_back(Eigen::Triplet<int>(v1, v2, i));
		//tripletsd.push_back(Eigen::Triplet<int>(v2, v3, i));
		//tripletsd.push_back(Eigen::Triplet<int>(v3, v1, i));
		//triplets.push_back(Eigen::Triplet<int>(v1, v2, 1));
		//triplets.push_back(Eigen::Triplet<int>(v2, v3, 1));
		//triplets.push_back(Eigen::Triplet<int>(v3, v1, 1));
		float wt12 = (p1.getVector3fMap() - p2.getVector3fMap()).norm();
		float wt23 = (p2.getVector3fMap() - p3.getVector3fMap()).norm();
		float wt31 = (p3.getVector3fMap() - p1.getVector3fMap()).norm();
		meanedge += wt12 + wt23 + wt31;
		count += 3;
		barycenter.x = (wt23*p1.x + wt31*p2.x + wt12*p3.x)/(wt12+ wt23+ wt31);
		barycenter.y = (wt23*p1.y + wt31*p2.y + wt12*p3.y) / (wt12 + wt23 + wt31);
		barycenter.z = (wt23*p1.z + wt31*p2.z + wt12*p3.z) / (wt12 + wt23 + wt31);
		if (mixed_mesh) {
			float wt1x = (p1.getVector3fMap() - barycenter.getVector3fMap()).norm();
			float wt2x = (p2.getVector3fMap() - barycenter.getVector3fMap()).norm();
			float wt3x = (p3.getVector3fMap() - barycenter.getVector3fMap()).norm();
			wEdge E1x(v1, dualgraphid, wt1x, i, true); dualgraphadjlist.at(v1).addEdge(E1x);
			wEdge E2x(v2, dualgraphid, wt2x, i, true); dualgraphadjlist.at(v2).addEdge(E2x);
			wEdge E3x(v3, dualgraphid, wt3x, i, true); dualgraphadjlist.at(v3).addEdge(E3x);
			wEdge Ex1(dualgraphid, v1, wt1x, i, false); dualgraphadjlist.at(dualgraphid).addEdge(Ex1);
			wEdge Ex2(dualgraphid, v2, wt2x, i, false); dualgraphadjlist.at(dualgraphid).addEdge(Ex2);
			wEdge Ex3(dualgraphid, v3, wt3x, i, false); dualgraphadjlist.at(dualgraphid).addEdge(Ex3);

		}
		wEdge E12(v1, v2, wt12,i,true); graphadjlist.at(v1).addEdge(E12);
		wEdge E23(v2, v3, wt23,i, true); graphadjlist.at(v2).addEdge(E23);
		wEdge E31(v3, v1, wt31,i, true); graphadjlist.at(v3).addEdge(E31);
		wEdge E21(v2, v1, wt12,i, false); graphadjlist.at(v2).addEdge(E21);
		wEdge E32(v3, v2, wt23,i, false); graphadjlist.at(v3).addEdge(E32);
		wEdge E13(v1, v3, wt31,i, false); graphadjlist.at(v1).addEdge(E13);
		if (mixed_mesh) {
			dualgraphadjlist.at(v1).addEdge(E12);
			dualgraphadjlist.at(v2).addEdge(E23);
			dualgraphadjlist.at(v3).addEdge(E31);
			dualgraphadjlist.at(v2).addEdge(E21);
			dualgraphadjlist.at(v3).addEdge(E32);
			dualgraphadjlist.at(v1).addEdge(E13);
		}
		DualCloud->push_back(barycenter);
	}
	meanedge /= count;
	
	for (int i = 0; i < graphadjlist.size(); i++) {
		adjVert V = graphadjlist.at(i);
		//building edges between dual vertices
		for (int j = 0; j < V.getEdgeNum(); j++) {
			wEdge E = V.getEdge(j);
			if (!E.isboundary()) {//nonboudary edges are translated to edges in the dual graph
				wEdge ED(a+E.face1,a+E.face2, (DualCloud->at(a+E.face1).getVector3fMap() - DualCloud->at(a+E.face2).getVector3fMap()).norm());
				//Dtriplets.push_back(Eigen::Triplet<int>(E.face1, E.face2,1));
				dualgraphadjlist.at(a+E.face1).addEdge(ED);
			}
		}
	}
	//adjmat.setFromTriplets(triplets.begin(),triplets.end());
	//adjmat = adjmat + Eigen::SparseMatrix<int>(adjmat.transpose());
	//adjmatD.setFromTriplets(tripletsd.begin(), tripletsd.end());
	//dualadjmat.setFromTriplets(Dtriplets.begin(), Dtriplets.end());
	//dualadjmat = dualadjmat + Eigen::SparseMatrix<int>(dualadjmat.transpose());

};

void graphMesh::make_second_order_dual() {
	vector<adjVert> temp = dualgraphadjlist;
	for (int i = 0; i < dualgraphadjlist.size(); i++) {
		adjVert V = temp.at(i);
		for (int j = 0; j < V.getEdgeNum(); j++) {
			adjVert VN = temp.at(V.getEdge(j).dest);
			for (int k = 0; k < VN.getEdgeNum(); k++) {
				int dest = VN.getEdge(k).dest;
				float wt = (DualCloud->at(i).getVector3fMap() - DualCloud->at(dest).getVector3fMap()).norm();
				wEdge W(i, dest, wt);
				dualgraphadjlist.at(i).addEdge(W);
			}
		}
	}
}

XYZCloud::Ptr graphMesh::dualcloud() {
	return DualCloud;
};

graphMesh::graphMesh() {
	DualCloud = XYZCloud::Ptr(new XYZCloud); Cloud = XYZCloud::Ptr(new XYZCloud);
}

graphMesh::graphMesh(PolygonMesh::Ptr mesh, bool mixed_mesh): faces(&mesh->polygons), basemesh(mesh){
	//Eigen::SparseMatrix<int> M(mesh.cloud.width, mesh.cloud.width);
	//adjmat = M;
	//adjmatD = M;
	//Eigen::SparseMatrix<int> DM(mesh.polygons.size(), mesh.polygons.size());
	//dualadjmat = DM;
	//dualadjmatD = DM;
	DualCloud = XYZCloud::Ptr(new XYZCloud); Cloud = XYZCloud::Ptr(new XYZCloud);
	mesh2graph(mesh, mixed_mesh);
}

graphMesh::graphMesh(const graphMesh& G) {
	graphadjlist = G.graphadjlist;
	dualgraphadjlist = G.dualgraphadjlist;
	faces = G.faces;
	Radiuses = G.Radiuses;
	basemesh = G.basemesh;
};

graphMesh::graphMesh(string filename) {
	ifstream vec_file(filename);
	//faces = *(new vector<Vertices>);
	string line;
	int i = 0;
	if (vec_file.is_open()) {
		while (getline(vec_file, line)) {
			std::istringstream stm(line);
			adjVert V(i);
			int dest; float wt;
			while ((stm >> dest) && (stm >> wt)) {
				V.addEdge(wEdge(i, dest, wt));
			}
			graphadjlist.push_back(V);
			i++;
		}
	}
}

bool graphMesh::savegraphMesh(string filename,bool dual) {
	ofstream file;
	file.open(filename);
	vector<adjVert>& graph = dual ? dualgraphadjlist : graphadjlist;
	for (int i = 0; i < graph.size(); i++) {
		for (int j = 0; j < graph.at(i).getEdgeNum(); j++) {
			file << graph.at(i).getEdge(j).dest << " " << graph.at(i).getEdge(j).wt << " ";
		}
		file << endl;
	}
	file.close();
	return true;
}

vector<int> graphMesh::QuasiLimitedSSSP(int Sind, float maxR) {//when default value is set this is essentialy BFS
	vector<int> indices;
	assert(Sind < graphadjlist.size());
	queue<adjVert> Q;
	graphadjlist.at(Sind).setSource();
	Q.push(graphadjlist.at(Sind));
	while (!Q.empty()) {
		adjVert V = Q.front();
		Q.pop();
		for (int i = 0; i < V.getEdgeNum(); i++) {
			wEdge E = V.getEdge(i);
			adjVert &W = graphadjlist.at(E.dest);
			if ((!W.visit(V, E)) & (W.getDist() < maxR)){
				Q.push(W);
			}
		}
		indices.push_back(V.getID());
	}
	return indices;
}

vector<int> graphMesh::dualtoindexes(vector<int> dualindexes) {
	vector<bool> indicators(graphadjlist.size(),false);//indicating if point is already added
	vector<int> result;
	for (int i = 0; i < dualindexes.size(); i++) {//loop over all corresponding faces
		vector<uint32_t> Vertice = faces->at(dualindexes.at(i)).vertices;
		for (int j = 0; j < Vertice.size(); j++) {
			if (!indicators.at(Vertice.at(i))) result.push_back(Vertice.at(i));//collect vertices on the original mesh
		}
	}
	return result;
};

vector<int> graphMesh::BFSSample(int rate,int src, float dist, bool maj, bool dual) {
	vector<adjVert>& graph = dual ? dualgraphadjlist : graphadjlist;
	vector<int> BFSDist(graph.size(), MAXINT); BFSDist.at(src) = 0;
	vector<int> indices;
	queue<adjVert> Q;
	Q.push(graph.at(src));
	while (!Q.empty()&&(Q.front().getDist()< dist)) {
		adjVert V = Q.front();
		int d = BFSDist.at(V.getID());
		bool push = maj ? (d%rate != 0) : (d%rate == 0);
		Q.pop();
		for (int i = 0; i < V.getEdgeNum(); i++) {
			wEdge E = V.getEdge(i);
			adjVert &W = graph.at(E.dest);
			if (BFSDist.at(W.getID())>d+1) {
				BFSDist.at(W.getID()) = d+1;
				Q.push(W);
			}
		}
		if (push) indices.push_back(V.getID());
	}

	return indices;
}

vector<int> graphMesh::LimitedSSSP(int Sind, float maxR, bool dual,int N) {//when default value is set this is essentialy BFS
	vector<int> indices;
	vector<adjVert>& graph = dual ? dualgraphadjlist : graphadjlist;
	vector<bool> indicators(graph.size(),false);
	//indices.push_back(Sind);
	//indicators.at(Sind) = true;
	priority_queue < adjVert, vector<adjVert>, VCompare> pq;
	adjVert& S = graph.at(Sind);
	S.setSource();
	pq.push(S);
	
	while (!pq.empty()&&(pq.top().getDist() <= maxR)&& (indices.size() < N))
	{
		// The first vertex in pair is the minimum distance
		// vertex, extract it from priority queue.
		// vertex label is stored in second of pair (it
		// has to be done this way to keep the vertices
		// sorted distance (distance must be first item
		// in pair)
		adjVert V = pq.top();
		pq.pop();
		if (!indicators.at(V.getID())) {
			indices.push_back(V.getID());
			indicators.at(V.getID()) = true;
			float d = V.getDist();
			// 'i' is used to get all adjacent vertices of a vertex
			for (int i = 0; i < V.getEdgeNum(); ++i)
			{
				// Get vertex label and weight of current adjacent
				// of u.
				wEdge E = V.getEdge(i);
				adjVert &W = graph.at(E.dest);
				//  If there is shorted path to v through u.
				float nwt = d + E.wt;
				if ((nwt < W.getDist()) && (nwt < maxR))
				{
					// Updating distance of v
					W.visit(V, E);
					pq.push(W);
				}
			}
		}
	}
	return indices;

};

void graphMesh::resetDist(bool dual) {
	vector<adjVert>& graph = dual ? dualgraphadjlist : graphadjlist;

	for (int i = 0; i < graph.size(); i++) {
		graph.at(i).resetDist();
	}
}

float graphMesh::getVertDist(int i,bool dual) {
	vector<adjVert>& graph = dual ? dualgraphadjlist : graphadjlist;
	return graph.at(i).getDist();
}
float graphMesh::getRadius(int i) {
	return Radiuses.at(i);
}

focal_s graphMesh::boundary_middist(vector<int> boundary, bool dual) {
	focal_s equidist;
	vector<int> candidates;
	vector<adjVert>& graph = dual ? dualgraphadjlist : graphadjlist;
	vector<bool> indicators(graph.size(), false);
	int maxind = 0;
	float Rmax = 0;
	for (int i = 0; i < boundary.size(); i++) {
		resetDist(dual);
		vector<int> indices = LimitedSSSP(boundary.at(i), INFINITY, dual);
		float R = getVertDist(indices.back(), dual);
		if (R > Rmax) {
			Rmax = R;
			maxind = boundary.at(i);
		}
	}
	resetDist(dual);
	vector<int> indices = LimitedSSSP(maxind, INFINITY, dual);
	for (int j = 0; j < indices.size(); j++) {
		if ((abs(Rmax / 2 - getVertDist(indices.at(j), dual)) < meanedge * 2)) {
			candidates.push_back(indices.at(j));
		}
	}
	float minVar = INFINITY;
	for (int i = 0; i < candidates.size(); i++) {
		resetDist(dual);
		vector<int> indices = LimitedSSSP(candidates.at(i), INFINITY, dual);
		float mu = 0, var = 0;
		for (int j = 0; j < boundary.size(); j++) {
			mu += getVertDist(boundary.at(j), dual) / boundary.size();
		}
		for (int j = 0; j < boundary.size(); j++) {
			var += pow((mu - getVertDist(boundary.at(j), dual)), 2) / boundary.size();
		}

		if (var < minVar) {
			minVar = var;
			equidist.ind = candidates.at(i);
			equidist.RG = getVertDist(indices.back(), dual);
		}
	}
	resetDist(dual);
	return equidist;
}


focal_s graphMesh::boundary_equidist(vector<int> boundary, bool dual) {
	focal_s equidist;
	vector<int> candidates;
	vector<adjVert>& graph = dual ? dualgraphadjlist : graphadjlist;
	vector<bool> indicators(graph.size(), false);
	for (int i = 0; i < boundary.size(); i++) {
		resetDist(dual);
		vector<int> indices = LimitedSSSP(boundary.at(i), INFINITY, dual);
		float R = getVertDist(indices.back(), dual);
		for (int j = 0; j < indices.size(); j++) {
			if ((abs(R / 2 - getVertDist(indices.at(j),dual)) < meanedge * 2) && !indicators.at(j)) {
				indicators.at(j) = true;
				candidates.push_back(j);
			}
		}
	}

	float minVar = INFINITY;
	for (int i = 0; i < candidates.size(); i++) {
		resetDist(dual);
		vector<int> indices = LimitedSSSP(candidates.at(i), INFINITY, dual);
		float mu = 0, var = 0;
		for (int j = 0; j < boundary.size(); j++) {
			mu += getVertDist(boundary.at(j), dual)/ boundary.size();
		}
		for (int j = 0; j < boundary.size(); j++) {
			var += pow((mu-getVertDist(boundary.at(j), dual)),2) / boundary.size();
		}

		if (var < minVar) {
			minVar = var;
			equidist.ind = candidates.at(i);
			equidist.RG = getVertDist(indices.back(), dual);
		}
	}
	resetDist(dual);
	return equidist;
}

focal_s graphMesh::boundary_maximizer(vector<int> boundary, bool dual) {
	focal_s maximizer;
	vector<int> candidates;
	vector<adjVert>& graph = dual ? dualgraphadjlist : graphadjlist;

	for (int i = 0; i < boundary.size(); i++) {
		resetDist(dual);
		vector<int> indices = LimitedSSSP(boundary.at(i), INFINITY, dual);
		candidates.push_back(indices.back());
	}
	float maxR = INFINITY;
	for (int i = 0; i < candidates.size(); i++) {
		resetDist(dual);
		vector<int> indices = LimitedSSSP(candidates.at(i), INFINITY, dual);
		float R = 0;
		for (int j = 0; j < boundary.size(); j++) {
			float Rtemp = getVertDist(boundary.at(j), dual);
			if (Rtemp > R) { R = Rtemp; }
		}
		if (R < maxR) {
			maxR = R;
			maximizer.ind = candidates.at(i);
			maximizer.RG = getVertDist(indices.back(), dual);
		}
	}
	resetDist(dual);
	return maximizer;
}

focal_s graphMesh::graphFocal(bool dual,bool createD) {
	focal_s focal;
	focal.RG = INFINITY;
	focal.ind = 0;
	Radiuses.resize(orig_vertex_size);
	if (createD) {
		focal.D.resize(orig_vertex_size);
	}

	vector<adjVert>& graph = dual ? dualgraphadjlist : graphadjlist;
	for (int i = 0; i < orig_vertex_size; i++) {
		focal.D.at(i).resize(orig_vertex_size);
		resetDist(dual);
		vector<int> indices = LimitedSSSP(i,INFINITY, dual);
		float R_max = getVertDist(indices.back(),dual);
		Radiuses.at(i) = R_max;
		if (R_max < focal.RG) {
			focal.ind = i; focal.RG = R_max;
			focal.path = getPath(indices.back(),dual);
		}
		if ((createD)&&(i<orig_vertex_size)) {
			for (int j = 0; j < graphadjlist.size(); j++) {
				focal.D.at(i).at(j)=getVertDist(j, dual);
			}
		}
	}
	R = focal.RG;
	resetDist(dual);
	return focal;
}

void graphMesh::visualize_distance_tree(bool dual) {
	vector<adjVert>& graph = dual ? dualgraphadjlist : graphadjlist;
	XYZCloud::Ptr cloud = dual ? DualCloud : Cloud;
	int v1(0), v2(0);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("3D Viewer 1"));
	viewer2->createViewPort(0.5, 0.0, 1.0, 1.0, v1);
	viewer2->createViewPort(0.0, 0.0, 0.5, 1.0, v2);
	string s = "mesh";
	viewer2->addPolygonMesh(*basemesh, s, v1);
	//viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, s,v1);
	string s1 = "cloud";
	viewer2->addPointCloud(cloud, s1, v2);
	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, s1, v1);
	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 122, s1, v1);

	//viewer2->setBackgroundColor(255, 255, 255, v1);
	//viewer2->setBackgroundColor(255, 255, 255, v2);

	for (int i = 0; i < graph.size(); i++) {
		float intensity = (getVertDist(i, dual)/R)* 255;
		if (intensity >255) { intensity = 0; }
		int src = getVertParent(i, dual);
		float r = hcmap[static_cast<int>(floor(intensity))][0];
		float g = hcmap[static_cast<int>(floor(intensity))][1];
		float b = hcmap[static_cast<int>(floor(intensity))][2];
		std::stringstream ss("line");
		std::stringstream ss2("lineo");
		ss << (10 * i);
		ss2 << (10 * i);

		viewer2->addLine(cloud->at(src), cloud->at(i), r, g, b, ss.str(), v2);
		viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, ss.str(), v2);
		viewer2->addLine(cloud->at(src), cloud->at(i), r, g, b, ss2.str(), v1);
		viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, ss2.str(), v1);

	}
	viewer2->resetCamera();
	while (!viewer2->wasStopped())
	{
		viewer2->spinOnce();
	}

}

bool save_focal(focal_s f, string out_file) {
	ofstream file;
	file.open(out_file);
	file << f.ind << endl << f.RG << endl << f.RE << endl;
	file.close();
	return true;
}

bool load_focal(focal_s &f, string in_file) {
	ifstream vec_file(in_file);
	string line;
	int i = 0;
	if (getline(vec_file, line)) {
		std::istringstream stm(line);
		stm >> f.ind;
		if (getline(vec_file, line)) {
			std::istringstream stm1(line);
			stm1 >> f.RG;
			return true;
		}
	}
	return false;
}

void graphMesh::visualize_dual_mesh() {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("3D Viewer 1"));
	string s = "mesh";
	int v1(0), v2(0);
	viewer2->createViewPort(0.5, 0.0, 1.0, 1.0, v1);
	viewer2->createViewPort(0.0, 0.0, 0.5, 1.0, v2);

	viewer2->addPolygonMesh(*basemesh, s,v1);
	//viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, s,v1);
	XYZRGBCloud::Ptr cloud(new XYZRGBCloud);
	fromPCLPointCloud2(basemesh->cloud, *cloud);
	viewer2->addPointCloud(cloud,"s",v2);
	viewer2->setBackgroundColor(255, 255, 255,v1);
	viewer2->setBackgroundColor(255, 255, 255,v2);
	viewer2->initCameraParameters();

	for (int i = 0; i < dualgraphadjlist.size(); i++) {
		adjVert V = dualgraphadjlist.at(i);
		//building edges between dual vertices
		for (int j = 0; j < V.getEdgeNum(); j++) {
			std::stringstream ss("line");
			std::stringstream ss2("lineo");
			ss << (10*i+j);
			ss2 << (10 * i + j);
			wEdge E = V.getEdge(j);
			viewer2->addLine(DualCloud->at(E.source), DualCloud->at(E.dest), 0, 0, 0, ss.str(),v2);
			viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, ss.str(),v2);
			viewer2->addLine(DualCloud->at(E.source), DualCloud->at(E.dest), 0, 0, 0, ss2.str(),v1);
			viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, ss2.str(),v1);
		}
	}			
	viewer2->addText("Original Cloud && Dual Mesh", 60, 10, 18, 0, 0, 0, "DDIS_1", v2);

	viewer2->addText("Original Mesh", 60, 10, 18, 0, 0, 0, "DDIS_2", v1);
	viewer2->resetCamera();
	while (!viewer2->wasStopped())
	{
		viewer2->spinOnce();
	}
}

