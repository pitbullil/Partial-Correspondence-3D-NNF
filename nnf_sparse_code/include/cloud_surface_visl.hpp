#pragma once
#ifndef CLOUD_SURFACE_H
#define CLOUD_SURFACE_H
#include <proj_namespaces.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/common/common.h>
#include <cloud_features_visl.hpp>
#include <pcl/features/from_meshes.h>
#include <pcl/features/boundary.h>

using namespace dis3;

struct boundary_count_s {
	int ind, count = 0;
};

struct boundary_label_s {
	std::vector<int> ind, label;
};


void FastTriangulation(XYZCloud::Ptr cloud, NormalCloud::Ptr Normals, pcl::PolygonMesh::Ptr mesh, float searchRadius = 0.025, float Mu = 2.5, int NN = 100);

void MLS(XYZCloud::Ptr cloud, NormalCloud::Ptr Normals, XYZCloud::Ptr cloudout,
	float searchR = 0.01, float upsamplingR = 0.005, float upsamplingStep = 0.003);


void MLS_func(XYZCloud::Ptr cloud, XYZCloud::Ptr out, float searchR = 0.01, float upsamplingR = 0.005, float upsamplingStep = 0.003);

void PoissonReconstruction(XYZCloud::Ptr cloud, NormalCloud::Ptr Normals, pcl::PolygonMesh::Ptr mesh, int depth = 10);

void clean_mesh(pcl::PolygonMesh::Ptr mesh, XYZCloud::Ptr cloud, float resolution);

pcl::PolygonMesh::Ptr ReconstructSurface(XYZCloud::Ptr cloud, NormalCloud::Ptr Normals, surface_recon_params_s params);

boundary_label_s MeshBoundary(vector<pcl::Vertices>& in, int size, bool longest_only);

/** \ brief returns the area of a traingle polygon
    \ param poly a structure contating indexes of the polygon cloud points
	\ param cloud a point cloud corresponding to the polygon
*/
float PolygonArea(pcl::Vertices poly, XYZCloud::Ptr cloud);

/** \ brief returns the area of a triangulated mesh
\ param triangulated mesh
\ param cloud a point cloud corresponding to the mesh
*/
float MeshArea(pcl::PolygonMesh::Ptr mesh, XYZCloud::Ptr cloud);

void RemoveBoundary(XYZCloud::Ptr cloud, vector<pcl::Vertices>& poly, cloud_param_s& p);

pcl::PolygonMesh::Ptr cut_mesh(pcl::PolygonMesh::Ptr inmesh, vector<barycentric_polygon_s> gt);

pcl::PolygonMesh::Ptr cut_mesh1(pcl::PolygonMesh::Ptr inmesh, vector<barycentric_polygon_s> gt);

pcl::PolygonMesh::Ptr cut_mesh_from_points(pcl::PolygonMesh::Ptr inmesh, vector<ind_r_s>& scene_P_D);

vector<float> DistSumFunctional(vector<vector<float>> D);

vector<uint32_t> geodesicMaximasupression(vector<float>& F, vector<uint32_t>& maximas, vector<vector<float>>& D, float diam, float thresh = 0.03);

vector<vector<uint32_t>> list_neighbors(vector<pcl::Vertices>& poly, int size, int neighbors = 1);

vector<uint32_t> meshMaximas(vector<float>& F, vector<vector<uint32_t>>& neigbor_list);

void surface_sampling(vector<uint32_t>& seeds, vector<vector<float>>& D, float diam, float thresh = 0.05);
#endif


