#pragma once

#ifndef CLOUD_REGISTRATION_H
#define CLOUD_REGISTRATION_H
#include <iostream>
#include <cloud_features_visl.hpp>
#include <cloud_registration_visl.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <proj_namespaces.hpp>
using namespace std;
using namespace Eigen;
using namespace dis3;

template<class CloudT1Pointer, class CloudT2Pointer> void seperate_clouds(CloudT1Pointer srcxyz, CloudT2Pointer tgtxyz, XYZCloud::Ptr src2xyz, XYZCloud::Ptr tgt2xyz, Vector4f n, float Radius, float multiplier);

/*  align a template cloud to it's (guessed) match in a scene cloud
srcxyz - the temoplate cloud
tgtxyz - the scene cloud part we want to align to
Radius - the radius to be translated after alignment */
template<class CloudT1Pointer, class CloudT2Pointer> void seperate_clouds(CloudT1Pointer srcxyz, CloudT2Pointer tgtxyz, XYZCloud::Ptr src2xyz, XYZCloud::Ptr tgt2xyz, Vector4f n, float Radius, float multiplier) {
	Vector4f centroid;
	Matrix4f transform_1 = Matrix4f::Identity();
	compute3DCentroid(*tgtxyz, centroid);
	Vector4f q = centroid;
	float cos = n.dot(q);
	n(3) = 0;
	n.normalize();
	n = n * multiplier * Radius;
	if (cos < 0) { //make sure final translation is away from the model center
		n = -n;
	}

	transform_1(0, 3) = n(0);
	transform_1(1, 3) = n(1);
	transform_1(2, 3) = n(2);
	transformPointCloud(*srcxyz, *srcxyz, transform_1);
	transformPointCloud(*src2xyz, *src2xyz, transform_1);

}

template<class CloudT1Pointer, class CloudT2Pointer> void seperate_clouds(CloudT1Pointer srcxyz, CloudT2Pointer tgtxyz, Vector4f n, float Radius, float multiplier) {
	Vector4f centroid;
	Matrix4f transform_1 = Matrix4f::Identity();
	compute3DCentroid(*tgtxyz, centroid);
	Vector4f q = centroid;
	float cos = n.dot(q);
	n(3) = 0;
	n.normalize();
	n = n * multiplier * Radius;
	if (cos < 0) { //make sure final translation is away from the model center
		n = -n;
	}

	transform_1(0, 3) = n(0);
	transform_1(1, 3) = n(1);
	transform_1(2, 3) = n(2);
	transformPointCloud(*srcxyz, *srcxyz, transform_1);

}

/*  Distance a cloud from another away from scene center of mass and along the a specified direction
srcxyz - the temoplate cloud
tgtxyz - the scene cloud part we want to align to
n - shift direction
Radius - the radius to be translated after alignment 
Multiplier - the number of radiuses we want to distance the clouds by*/
void seperate_clouds(XYZCloud::Ptr srcxyz, XYZCloud::Ptr tgtxyz, Eigen::Vector4f n, float Radius, float multiplier = 5);
void seperate_clouds(XYZLCloud::Ptr srcxyz, XYZCloud::Ptr tgtxyz, Eigen::Vector4f n, float Radius, float multiplier = 5);

void seperate_clouds(XYZCloud::Ptr srcxyz, XYZCloud::Ptr tgtxyz, XYZCloud::Ptr src2xyz, XYZCloud::Ptr tgt2xyz, Vector4f n, float Radius, float multiplier);

/*void mirrorCloud(XYZCloud::Ptr tgtxyz, Eigen::Vector3f n);*/
float align_clouds_icp(XYZCloud::Ptr srcxyz, XYZCloud::Ptr tgtxyz, Matrix4f& final_transform);
Matrix4f initial_alignment(XYZCloud::Ptr T_cloud, XYZCloud::Ptr scene_cloud, SHOTCloud::Ptr T_SHOT_cloud, SHOTCloud::Ptr scene_SHOT_cloud, pcl::CorrespondencesPtr BBS);
Matrix4f initial_alignment(XYZCloud::Ptr T_cloud, XYZCloud::Ptr scene_cloud, FPFHCloud::Ptr T_FPFH_cloud, FPFHCloud::Ptr scene_FPFH_cloud, pcl::CorrespondencesPtr BBS);
Matrix4f initial_alignment2(XYZCloud::Ptr T_cloud, XYZCloud::Ptr scene_cloud, FPFHCloud::Ptr T_FPFH_cloud, FPFHCloud::Ptr scene_FPFH_cloud);
float align_clouds_icp_n(XYZNCloud::Ptr srcxyz, XYZNCloud::Ptr tgtxyz, Matrix4f& final_transform);
vector<int> project_cloud(XYZCloud::Ptr source, XYZCloud::Ptr destination, XYZCloud::Ptr resultCloud);

#endif