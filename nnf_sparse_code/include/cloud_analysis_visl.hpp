#pragma once
#ifndef CLOUD_ANALYSIS_H
#define CLOUD_ANALYSIS_H

#include <iostream>
#include <fstream>
#include <meshgraph.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/correspondence_estimation.h>
#include <proj_namespaces.hpp>

using namespace dis3; 

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;

#ifdef MESHGRAPH_H
void integral_mesh(XYZICloud::Ptr input, graphMesh graph, XYZICloud::Ptr output, float R);
#endif

XYZCloud::Ptr filterpointsonsurface(XYZCloud::Ptr sourcePointCloud,
	XYZCloud::Ptr surfacePointCloud,
	float threshold);

void findGeoMatches(XYZCloud::Ptr cloud1, XYZCloud::Ptr keyPoints1, XYZCloud::Ptr keyPoints2,  pcl::CorrespondencesPtr correspondences, double thresholdMultiplier , double modelResolution, double y2xRatio);

void findResolution(XYZCloud::Ptr cloud, cloud_statistics_s& S, std::string filename = "bollocks", bool save = true);

float findMSE(XYZCloud::Ptr keyPoints1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints2_transformed,
	pcl::CorrespondencesPtr goodCorrespondences);

void pointCloudCut(XYZCloud::Ptr sourcePointCloud,
	XYZCloud::Ptr targetPointCloud,
	int startingAngle,
	int endAngle);

std::vector<int> appxFarthestPoints(CloudT::Ptr cloud);

std::vector<int> Region_Growth(XYZCloud::Ptr cloud, vector<int> init_region, float radius);
#endif