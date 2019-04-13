#pragma once
#ifndef CLOUD_FEATURES_H
#define CLOUD_FEATURES_H
#include <iostream>
#include <ostream>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/shot.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/features/rops_estimation.h>
#include <proj_namespaces.hpp>
#include <cloud_io_visl.hpp>
#include <cloud_analysis_visl.hpp>
#include <cloud_rops_visl.h>
#include <nnf_five.h>
#include "cloud_features_visl.tpp"
#include <hks.h>
#include <customfeatures.h>


using namespace dis3;

typedef pcl::PointCloud<ROPS> ROPSCloud;

void computeNormals(XYZCloud::Ptr pointsCloud,
	XYZCloud::Ptr keyPoints,
	NormalCloud::Ptr normals,
	float model_resolution,
	bool Autodefault = false,
	float multiplier = dis3::RADIUS_SEARCH_MULTIPLIER);

void computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr pointsCloud,
	pcl::PointCloud<pcl::Normal>::Ptr normals,
	float searchRadius,
	bool Autodefault = false);

void computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr pointsCloud,
	pcl::PointCloud<pcl::Normal>::Ptr normals,
	int NN = 10,
	bool Autodefault = false);

void computeROPSFeatures(
	XYZCloud::Ptr pointsCloud,
	XYZCloud::Ptr keyPoints,
	//pcl::PointIndicesPtr indices,
	vector <pcl::Vertices> triangles,
	ROPSCloud& rops,
	float support_radius= 0.0285f);

void computeFPFHFeatures(
	XYZCloud::Ptr pointsCloud,
	XYZCloud::Ptr keyPoints,
	NormalCloud::Ptr normals,
	FPFHCloud& fpfhs ,
	 float searchRadius);

void computePFHFeatures(
	XYZCloud::Ptr pointsCloud,
	XYZCloud::Ptr keyPoints,
	pcl::PointCloud<N>::Ptr normals,
	pcl::PointCloud<PFH>& pfhs,
	float searchRadius);

void computeSFPFHFeatures(
	XYZCloud::Ptr pointsCloud,
	XYZCloud::Ptr keyPoints,
	NormalCloud::Ptr normals,
	pcl::SFPFHCloud& sfpfhs,
	Eigen::MatrixXf* D,
	float searchRadius);

void computeGFPFHFeatures(
	XYZCloud::Ptr pointsCloud,
	XYZCloud::Ptr keyPoints,
	NormalCloud::Ptr normals,
	pcl::GFPFHCloud& gfpfhs,
	Eigen::MatrixXf* D,
	float searchRadius);

void computeGPFHFeatures(
	XYZCloud::Ptr pointsCloud,
	XYZCloud::Ptr keyPoints,
	pcl::PointCloud<N>::Ptr normals,
	pcl::GPFHCloud& gpfhs,
	Eigen::MatrixXf* D,
	float searchRadius);


void computeSHOTFeatures(
	XYZCloud::Ptr pointsCloud,
	XYZCloud::Ptr keyPoints,
	NormalCloud::Ptr normals,
	SHOTCloud& shot_desc,
	 float searchRadius
	);

template<class PointT> pcl::PointCloud<PointT> downsample(pcl::PointCloud<PointT> cloud, float resultion_in, float resultion_out);

void computeNNFs(SHOTCloud::Ptr t,
	SHOTCloud::Ptr q,
	pcl::PointCloud<pcl::PointXYZL>::Ptr nnf);

template<class PointT> void computeNNFChiSquare(pcl::PointCloud<PointT>& t,
	pcl::PointCloud<PointT>& q,
	pcl::PointCloud<pcl::PointXYZL>::Ptr nnf,
	float RejectThreshold=1);

template<class PointT, int X> void computeNNFChiSquare(pcl::PointCloud<PointT>& t,
	pcl::PointCloud<PointT>& q,
	pcl::PointCloud<dis3::NNFX<X>>& nnf,
	float RejectThreshold=1);

template<class PointT, int X> void computeNNFChiSquare(pcl::PointCloud<PointT>& t,
	pcl::PointCloud<PointT>& q,
	pcl::PointCloud<dis3::NNFX<X>>& nnf,
	float RejectThreshold) {
	printSection("computing NNF using Chi Square");
	pcl::KdTreeFLANN<PointT, flann::ChiSquareDistance<float>> Feature_kdtree;
	pcl::PointCloud<PointT>::Ptr t1(new pcl::PointCloud<PointT>());
	copyPointCloud(t, *t1);
	Feature_kdtree.setInputCloud(t1);
	vector<int> neigh_indices;
	vector<float> neigh_sqr_dists;
	//Computing nearest neighbor field
	for (size_t i = 0; i < q.size(); ++i)
	{
		if (!pcl_isfinite(q.at(i).histogram[0])) //skipping NaNs
		{
			continue;
		}
		int found_neighs = Feature_kdtree.nearestKSearch(q.at(i), X+1, neigh_indices, neigh_sqr_dists);
		for (int j = 0; j < dis3::NNFX<X>::labelSize(); j++) {
			nnf.at(i).labels[j] = MAXUINT32;
			if (neigh_sqr_dists[j] / neigh_sqr_dists[j+1] < RejectThreshold) {
				nnf.at(i).labels[j] = neigh_indices[j];
				nnf.at(i).distance[j] = neigh_sqr_dists[j];
			}
		}
	}
}

template<class PointT> void computeNNFChiSquare(pcl::PointCloud<PointT>& t,
	pcl::PointCloud<PointT>& q,
	pcl::PointCloud<pcl::PointXYZL>::Ptr nnf,
	float RejectThreshold) {
	printSection("computing NNF using Chi Square");
	pcl::KdTreeFLANN<PointT, flann::ChiSquareDistance<float>> Feature_kdtree;
	pcl::PointCloud<PointT>::Ptr t1(new pcl::PointCloud<PointT>());
	copyPointCloud(t, *t1);
	Feature_kdtree.setInputCloud(t1);
	vector<int> neigh_indices;
	vector<float> neigh_sqr_dists;
	//Computing nearest neighbor field
	for (size_t i = 0; i < q.size(); ++i)
	{
		if (!pcl_isfinite(q.at(i).histogram[0])) //skipping NaNs
		{
			continue;
		}
		int found_neighs = Feature_kdtree.nearestKSearch(q.at(i), 2, neigh_indices, neigh_sqr_dists);
		if (found_neighs == 2) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
		{
			nnf->points[i].label = MAXUINT32;
			if (neigh_sqr_dists[0] / neigh_sqr_dists[1] < RejectThreshold) {
				nnf->points[i].label = neigh_indices[0];
			}
		}
	}
}

template<class PointT> void computeNNFL2(pcl::PointCloud<PointT>& t,
	pcl::PointCloud<PointT>& q,
	pcl::PointCloud<pcl::PointXYZL>::Ptr nnf,
	float RejectThreshold = 1);

template<class PointT> void computeNNFL2(pcl::PointCloud<PointT>& t,
	pcl::PointCloud<PointT>& q,
	pcl::PointCloud<pcl::PointXYZL>::Ptr nnf,
	float RejectThreshold) {
	printSection("computing NNF using L2 distance");
	pcl::KdTreeFLANN<PointT> Feature_kdtree;
	pcl::PointCloud<PointT>::Ptr t1(new pcl::PointCloud<PointT>());
	copyPointCloud(t, *t1);
	cout << "1\n";
	Feature_kdtree.setInputCloud(t1);
	cout << "2\n";

	vector<int> neigh_indices;
	vector<float> neigh_sqr_dists;
	//Computing nearest neighbor field
	for (size_t i = 0; i < q.size(); ++i)
	{
		if (!pcl_isfinite(q.at(i).descriptor[0])) //skipping NaNs
		{
			continue;
		}
		int found_neighs = Feature_kdtree.nearestKSearch(q.at(i), 2, neigh_indices, neigh_sqr_dists);
		if (found_neighs == 2) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
		{
			nnf->points[i].label = MAXUINT32;
			if (neigh_sqr_dists[0] / neigh_sqr_dists[1] < RejectThreshold) {
				nnf->points[i].label = neigh_indices[0];
			}
		}
	}
	cout << "3\n";
}

template<class PointT, int X> void computeNNFL2(pcl::PointCloud<PointT>& t,
	pcl::PointCloud<PointT>& q,
	pcl::PointCloud<dis3::NNFX<X>>& nnf,
	float RejectThreshold) {
	printSection("computing NNF using L2 distance");
	pcl::KdTreeFLANN<PointT> Feature_kdtree;
	pcl::PointCloud<PointT>::Ptr t1(new pcl::PointCloud<PointT>());
	copyPointCloud(t, *t1);
	cout << "1\n";
	Feature_kdtree.setInputCloud(t1);
	cout << "2\n";

	vector<int> neigh_indices;
	vector<float> neigh_sqr_dists;
	//Computing nearest neighbor field
	for (size_t i = 0; i < q.size(); ++i)
	{
		if (!pcl_isfinite(q.at(i).descriptor[0])) //skipping NaNs
		{
			continue;
		}
		int found_neighs = Feature_kdtree.nearestKSearch(q.at(i), X, neigh_indices, neigh_sqr_dists);
		for (int j = 0; j < dis3::NNFX<X>::labelSize(); j++) {
			nnf.at(i).labels[j] = neigh_indices[j];
			nnf.at(i).distance[j] = neigh_sqr_dists[j];
		}
	}
	cout << "3\n";
}

template <typename PointT, typename PointNT> inline void
computeApproximateMeshNormals(const pcl::PointCloud<PointT>& cloud, const std::vector<pcl::Vertices>& polygons, pcl::PointCloud<PointNT>& normals);

template <typename PointT, typename PointNT> inline void
computeApproximateMeshNormals(const pcl::PointCloud<PointT>& cloud, const std::vector<pcl::Vertices>& polygons, pcl::PointCloud<PointNT>& normals)
{
	int nr_points = static_cast<int>(cloud.points.size());
	int nr_polygons = static_cast<int>(polygons.size());

	normals.header = cloud.header;
	normals.width = cloud.width;
	normals.height = cloud.height;
	normals.points.resize(nr_points);

	for (int i = 0; i < nr_points; ++i)
		normals.points[i].getNormalVector3fMap() = Eigen::Vector3f::Zero();

	// NOTE: for efficiency the weight is computed implicitly by using the
	// cross product, this causes inaccurate normals for meshes containing
	// non-triangle polygons (quads or other types)
	for (int i = 0; i < nr_polygons; ++i)
	{
		const int nr_points_polygon = (int)polygons[i].vertices.size();
		if (nr_points_polygon < 3) continue;

		// compute normal for triangle
		Eigen::Vector3f vec_a_b = cloud.points[polygons[i].vertices[0]].getVector3fMap() - cloud.points[polygons[i].vertices[1]].getVector3fMap();
		Eigen::Vector3f vec_a_c = cloud.points[polygons[i].vertices[0]].getVector3fMap() - cloud.points[polygons[i].vertices[2]].getVector3fMap();
		Eigen::Vector3f normal = vec_a_b.cross(vec_a_c);
		//pcl::flipNormalTowardsViewpoint(cloud.points[polygons[i].vertices[0]], 0.0f, 0.0f, 0.0f, normal(0), normal(1), normal(2));

		// add normal to all points in polygon
		for (int j = 0; j < nr_points_polygon; ++j)
			normals.points[polygons[i].vertices[j]].getNormalVector3fMap() += normal;
	}

	for (int i = 0; i < nr_points; ++i)
	{
		normals.points[i].getNormalVector3fMap().normalize();
		//pcl::flipNormalTowardsViewpoint(cloud.points[i], 0.0f, 0.0f, 0.0f, normals.points[i].normal_x, normals.points[i].normal_y, normals.points[i].normal_z);
	}
}

#endif
