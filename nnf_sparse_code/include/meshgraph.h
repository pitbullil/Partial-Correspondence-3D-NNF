#ifndef MESHGRAPH_H
#define MESHGRAPH_H

#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <assert.h> 
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/io.h>
#include <pcl/conversions.h>
#include <cloud_io_visl.hpp>
#include <cloud_visualization_visl.hpp>
#include <proj_namespaces.hpp>
#include <Eigen/Sparse>

using namespace std;
using namespace dis3;

struct graphpath_s {
	vector<int> chain;
	vector<float> distance;
};

class wEdge {
	
public:
	wEdge(int src, int dst, float weight);
	wEdge(int src, int dst, float weight,int face, bool temp);
	float wt;
	int face1=-1, face2=-1;
	int source,dest;
	bool used = false;
	bool operator <(const wEdge &E);
	bool isboundary();
};

class adjVert {
private:
	int id;
	vector<wEdge> adjlist;
	bool mark;
	float dist;
	int parent;
public:
	inline int getID() { return id; }
	adjVert(int i);
	void addEdge(wEdge edge);
	bool hasEdge(int dest);
	int getEdgeNum();
	wEdge getEdge(int i);
	float getDist() const;
	void resetDist();
	bool visit(adjVert& S, wEdge& E);
	void setSource();
	bool operator<(adjVert V);
	int getParent() const;
};

struct focal_s {
	int ind;
	float RG;
	float RE;
	graphpath_s path;
	vector<vector<float>> D;//a matrix containing pairwise distances
};

class graphMesh {
private:
	vector<adjVert> graphadjlist;
	vector<adjVert> dualgraphadjlist;
	vector<pcl::Vertices>* faces;
	vector<float> Radiuses;
	pcl::PolygonMesh::Ptr basemesh;
	Eigen::SparseMatrix<int> adjmat;
	Eigen::SparseMatrix<int> adjmatD;
	Eigen::SparseMatrix<int> dualadjmat;
	Eigen::SparseMatrix<int> dualadjmatD;
	float meanedge;
	uint32_t orig_vertex_size;
	XYZCloud::Ptr Cloud;
	XYZCloud::Ptr DualCloud;
public:
	graphMesh();//do not use - will result in error
	graphMesh(pcl::PolygonMesh::Ptr mesh, bool mixed_mesh = false);//gets a mesh and constructs an adjacancy list from it
	graphMesh(string filename);//WARNING: defunct gets path to a file containing an adjacancy list mesh and reads it
	graphMesh(const graphMesh& G);
	void mesh2graph(pcl::PolygonMesh::Ptr mesh, bool mixed_mesh=false);
	bool savegraphMesh(string filename, bool dual=false);//saves adjacency list representation of mesh into file
	vector<int> QuasiLimitedSSSP(int Sind, float maxR = INFINITY);//runs BFS based approximation to Shortest Path algorithm, which stops when all new vertices are farther then maxR
	vector<int> LimitedSSSP(int Sind, float maxR = INFINITY, bool dual = false, int N = MAXINT);
	void resetDist(bool dual = false);//resets distances to each vertex on the graph.
	float getVertDist(int i,bool dual = false);
	int getVertParent(int i,bool dual = false);
	focal_s boundary_maximizer(vector<int> boundary, bool dual = false);
	focal_s boundary_equidist(vector<int> boundary, bool dual = false);
	focal_s boundary_middist(vector<int> boundary, bool dual = false);
	vector<int> BFSSample(int rate, int src, float dist = INFINITY, bool maj=true, bool dual = true);
	vector<int> dualtoindexes(vector<int> dualindexes);
	focal_s graphFocal(bool dual = false,bool createD=false);
	graphpath_s getPath(int i, bool dual = false);
	XYZCloud::Ptr dualcloud();
	void make_second_order_dual();
	void visualize_dual_mesh();
	void visualize_distance_tree(bool dual = false);
	float R;
	float getRadius(int i);
};


class VCompare {
public:
	bool operator()(const adjVert &a,const adjVert & b) {
		return (a.getDist()>b.getDist());
	}
};

bool save_focal(focal_s f, string out_file);

bool load_focal(focal_s &f, string in_file);
	
#endif