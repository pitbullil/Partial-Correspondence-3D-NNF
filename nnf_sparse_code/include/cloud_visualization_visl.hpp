#pragma once
#ifndef CLOUD_VISUALIZATION_H
#define CLOUD_VISUALIZATION_H

#include <proj_namespaces.hpp>
#include <misc_utils.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <nnf_five.h>
#include "colormaps.hpp"

using namespace dis3;
bool color_point(const pcl::PointXYZI point_in, pcl::PointXYZRGB& point);
bool color_pointi(const pcl::PointXYZI point_in, pcl::PointXYZRGB& point);

bool color_point(const pcl::PointXYZL point_in, pcl::PointXYZRGB& point);
bool color_pointi(const pcl::PointXYZL point_in, pcl::PointXYZRGB& point);

template<int X> bool color_point(const dis3::NNFX<X> point_in, pcl::PointXYZRGB& point) {
	point.r = lcmap[point_in.labels[0] % 1000][0];
	point.g = lcmap[point_in.labels[0] % 1000][1];
	point.b = lcmap[point_in.labels[0] % 1000][2];
	return true;
}

template<int X> bool color_pointi(const dis3::NNFX<X> point_in, pcl::PointXYZRGB& point) { return false; };

template <typename PointT>
pcl::PointCloud<pcl::PointXYZRGB>::Ptr visualizeCloud(typename pcl::PointCloud<PointT>::Ptr& cloud ,int colorset=0) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::vector<PointT, Eigen::aligned_allocator<PointT>>::iterator it = cloud->points.begin();
	int i = 0;
	for (it = cloud->points.begin(), i = 0; it.operator!=(cloud->points.end()); it++, i++) {
		bool valid;
		pcl::PointXYZRGB point;
		point.x = cloud->at(i).x;
		point.y = cloud->at(i).y;
		point.z = cloud->at(i).z;
		
		if (colorset == 1) {
			valid =color_pointi(cloud->at(i), point);
		}
		else {
			valid = color_point(cloud->at(i), point);
		}
		if (valid) rgb_point_cloud_ptr->points.push_back(point);

	}
	//rgb_point_cloud_ptr->width = (int)cloud->points.size();
	//rgb_point_cloud_ptr->height = 1;

	return rgb_point_cloud_ptr;
}

template <typename PointT>
pcl::PointCloud<pcl::PointXYZRGB>::Ptr visualizeNNF(pcl::PointCloud<PointT>& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr basecloud, int colorset = 0) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::vector<PointT, Eigen::aligned_allocator<PointT>>::iterator it = cloud.points.begin();
	int i = 0;
	for (it = cloud.points.begin(), i = 0; it.operator!=(cloud.points.end()); it++, i++) {
		bool valid;
		pcl::PointXYZRGB point;
		point.x = basecloud->at(i).x;
		point.y = basecloud->at(i).y;
		point.z = basecloud->at(i).z;

		if (colorset == 1) {
			valid = color_pointi(cloud.at(i), point);
		}
		else {
			valid = color_point(cloud.at(i), point);
		}
		if (valid) rgb_point_cloud_ptr->points.push_back(point);

	}
	//rgb_point_cloud_ptr->width = (int)cloud->points.size();
	//rgb_point_cloud_ptr->height = 1;

	return rgb_point_cloud_ptr;
}

void visualize_keyPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints2,
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointsCloud1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointsCloud2,
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointsCloud2_transformed,
	pcl::CorrespondencesPtr correpsondences,
	pcl::CorrespondencesPtr ransacCorrepsondences);

void gt_visualization(XYZCloud::Ptr gt_cloud,
	vector<vector<int>> gt,
	pcl::PolygonMeshPtr gt_mesh,
	pcl::PolygonMeshPtr cutmesh,
	XYZCloud::Ptr cut_cloud,
	vector<int> points);

void nnf_visualization(XYZCloud::Ptr scene_cloud,
	pcl::PointCloud<NNFX<5>>::Ptr nnf5,
	pcl::PolygonMeshPtr scenemesh,
	pcl::PolygonMeshPtr cutmesh,
	XYZCloud::Ptr cut_cloud,
	vector<int> points);

void normal_visualization(XYZCloud::Ptr cloud, pcl::PolygonMeshPtr mesh, NormalCloud::Ptr normals, vector<int> points);
#endif

