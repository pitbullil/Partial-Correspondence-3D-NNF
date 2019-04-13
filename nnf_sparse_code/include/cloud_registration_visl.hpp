
#pragma once

#ifndef CLOUD_REGISTRATION_V_H
#define CLOUD_REGISTRATION_V_H

#include <proj_namespaces.hpp>
#include <cloud_visualization_visl.hpp>
#include <cloud_io_visl.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <cloud_features_visl.hpp>

#include <vector>

using namespace dis3;
namespace registration_visl {
	const float RANSAC_THRESHOLD_MULTIPLIER = 15;
const float	MATCHING_ERROR_TO_FEATURE_NORM_RATIO = 0.3;
const float ICP_THRESHOLD_MULTIPLIER = 4;
}

void transformPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut,
	Eigen::Matrix4f &transform);

void corrrespondencesToPoints(pcl::CorrespondencesPtr correspondences,
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints2,
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints1Matches,
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints2Matches
);

void ransac(const pcl::CorrespondencesPtr &all_correspondences,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr &keypoints_src,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr &keypoints_tgt,
	pcl::Correspondences &remaining_correspondences,
	int maxIters, float threshold);

void matchKeyPointsFeatures(
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints2,
	pcl::PointCloud<pcl::SHOT352>::Ptr shot1,
	pcl::PointCloud<pcl::SHOT352>::Ptr shot2,
	float model_resolution,
	pcl::CorrespondencesPtr all_correspondences);


void matchKeyPointsFeatures(
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints2,
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr  fpfhs1,
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr  fpfhs2,
	float model_resolution,
	pcl::CorrespondencesPtr all_correspondences
);


void findGeoMatches(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints2,
	pcl::CorrespondencesPtr correspondences,
	float threshold);


void calcTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints2,
	pcl::CorrespondencesPtr correspondences,
	Eigen::Matrix4f &transform);

void filterMatchesByFeaturesDist(pcl::PointCloud<pcl::SHOT352>& features1,
	pcl::PointCloud<pcl::SHOT352>& features2,
	pcl::CorrespondencesPtr correspondences,
	pcl::CorrespondencesPtr filteredCorrespondences,
	float thresholdPrecentage
);

void filterMatchesByFeaturesDist(pcl::PointCloud<pcl::FPFHSignature33>& features1,
	pcl::PointCloud<pcl::FPFHSignature33>& features2,
	pcl::CorrespondencesPtr correspondences,
	pcl::CorrespondencesPtr filteredCorrespondences,
	float thresholdPrecentage);



template <class PointT> void mymatchKeyPointsFeatures(
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints2,
	pcl::PointCloud<PointT>&  F1,
	pcl::PointCloud<PointT>&  F2,
	float modelResolution,
	pcl::CorrespondencesPtr correspondences)
{
	pcl::PointCloud<PointT>::Ptr F1p(new pcl::PointCloud<PointT>()), F2p(new pcl::PointCloud<PointT>());
	copyPointCloud(F1, *F1p);	copyPointCloud(F2, *F2p);
	pcl::registration::CorrespondenceEstimation<PointT, PointT> estimator;
	estimator.setInputSource(F1p);
	estimator.setInputTarget(F2p);
	estimator.determineReciprocalCorrespondences(*correspondences);
}

template<> inline void mymatchKeyPointsFeatures(
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints2,
	pcl::PointCloud<ROPS>&  F1,
	pcl::PointCloud<ROPS>&  F2,
	float modelResolution,
	pcl::CorrespondencesPtr correspondences)
{
	pcl::KdTreeFLANN<ROPS> tree;
	pcl::PointCloud<ROPS>::Ptr F1p(new pcl::PointCloud<ROPS>()), F2p(new pcl::PointCloud<ROPS>());
	tree.setInputCloud(F1p);
	XYZLCloud::Ptr temp(new XYZLCloud);
	copyPointCloud(*keyPoints1, *temp);
	computeNNFL2(F1, F2, temp);
	vector<int> indices; vector<float> sceneDistances;
	for (int i = 0; i < F2.size(); i++) {
		tree.nearestKSearch(F2.at(i), 1, indices, sceneDistances);
		if (temp->at(indices.at(0)).label == i) {
			pcl::Correspondence c(i, indices.at(0), sceneDistances.at(0));
			correspondences->push_back(c);
		}
	}
}

float get_icp_transform(XYZCloud::Ptr srcxyz, XYZCloud::Ptr tgtxyz, Eigen::Matrix4f& final_transform);

#endif