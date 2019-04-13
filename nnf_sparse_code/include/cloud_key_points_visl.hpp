#pragma once
#ifndef CLOUD_KEYPOINTS_H
#define CLOUD_KEYPOINTS_H
#include <iostream>
#include <string.h>
#include <cloud_io_visl.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
#include <proj_namespaces.hpp>

namespace keypoints_visl {

	const float RADIUS_SEARCH_MULTIPLIER = 15;
	const float NON_MAX_RADIUS_MULTIPLIER = 10;
}

using namespace std;
using namespace dis3;
void computeHarris(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointsCloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &keyPoints,
	double modelResolution, float radius, float radiusSearch, bool nonMaxSuppression, bool refine);


void computeISS(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointsCloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &keyPoints,
	double modelResolution,
	float salient_radius,
	float non_max_radius,
	float gamma_21,
	float gamma_32,
	int min_neighbors,
	int threads,
	bool border);

void computeSIFT(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointsCloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &keyPoints,
	double modelResolution,
	float minScale,
	int nOctaves,
	int scalesPerOctave,
	float minContrast,
	float searchRadius);

int computeKeyPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr pointsCloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints,
	KeypointTypes Method,
	float modelResolution, bool border = false, bool autodefault = false);

void computeNARF(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointsCloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &keyPoints,
	double modelResolution, float searchRadius,
	float angularResolution, float supportSize,
	float noiseLevel, float minRange, float borderSize);
#endif

