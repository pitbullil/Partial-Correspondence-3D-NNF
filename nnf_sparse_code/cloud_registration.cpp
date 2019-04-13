#include <cloud_registeration.hpp>

using namespace std;
using namespace pcl;
using namespace Eigen;

/*template<class CloudT1Pointer, class CloudT2Pointer> void seperate_clouds(CloudT1Pointer srcxyz, CloudT2Pointer tgtxyz, XYZCloud::Ptr src2xyz, XYZCloud::Ptr tgt2xyz, Vector4f n, float Radius, float multiplier) {
	Vector4f centroid;
	Matrix4f transform_1 = Matrix4f::Identity();
	compute3DCentroid(*tgtxyz, centroid);
	Vector4f q = centroid;
	float cos = n.dot(q);
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

}*/


float align_clouds_icp(XYZCloud::Ptr srcxyz, XYZCloud::Ptr tgtxyz, Matrix4f& final_transform) {
	Matrix4f transform_1 = Matrix4f::Identity();
	IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud(srcxyz);
	icp.setInputTarget(tgtxyz);
	final_transform = icp.getFinalTransformation();
	icp.align(*srcxyz);
	return icp.getFitnessScore();
}

float align_clouds_icp_n(XYZNCloud::Ptr srcxyz, XYZNCloud::Ptr tgtxyz, Matrix4f& final_transform) {
	Matrix4f transform_1 = Matrix4f::Identity();
	IterativeClosestPointWithNormals<PointNormal, PointNormal> icp;
	icp.setInputCloud(srcxyz);
	icp.setInputTarget(tgtxyz);
	final_transform = icp.getFinalTransformation();
	icp.align(*srcxyz);
	return icp.getFitnessScore();
}

/*void seperate_clouds(XYZCloud::Ptr srcxyz, XYZCloud::Ptr tgtxyz, Vector4f n, float Radius, float multiplier) {
	Vector4f centroid;
	Matrix4f transform_1 = Matrix4f::Identity();
	compute3DCentroid(*tgtxyz, centroid);
	Vector4f q = centroid;
	float cos = n.dot(q);
	n.normalize();
	n = n * multiplier * Radius;
	if (cos < 0) { //make sure final translation is away from the model center
		n = -n;
	}

	transform_1(0, 3) = n(0);
	transform_1(1, 3) = n(1);
	transform_1(2, 3) = n(2);
	transformPointCloud(*srcxyz, *srcxyz, transform_1);

}*/

/*void seperate_clouds(XYZCloud::Ptr srcxyz, XYZCloud::Ptr tgtxyz, XYZCloud::Ptr src2xyz, XYZCloud::Ptr tgt2xyz, Vector4f n, float Radius, float multiplier) {
	Vector4f centroid;
	Matrix4f transform_1 = Matrix4f::Identity();
	compute3DCentroid(*tgtxyz, centroid);
	Vector4f q = centroid;
	float cos = n.dot(q);
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
}*/

Matrix4f initial_alignment(XYZCloud::Ptr T_cloud, XYZCloud::Ptr scene_cloud, FPFHCloud::Ptr T_FPFH_cloud, FPFHCloud::Ptr scene_FPFH_cloud, CorrespondencesPtr BBS) {
	SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
	float reject_dist = 0;
	float min_sample_distance = 0;
	cout << "2\n";
	for (int i=0; i < BBS->size(); i++) {
		if (BBS->at(i).distance > reject_dist) { reject_dist = BBS->at(i).distance; }
		if (BBS->at(i).distance < min_sample_distance) { min_sample_distance = BBS->at(i).distance; }
	}
	sac_ia_.setInputCloud(T_cloud);
	sac_ia_.setSourceFeatures(T_FPFH_cloud);
	sac_ia_.setInputTarget(scene_cloud);
	sac_ia_.setTargetFeatures(scene_FPFH_cloud);
	sac_ia_.setMinSampleDistance(min_sample_distance);
	sac_ia_.setMaxCorrespondenceDistance(11);
	cout << "3\n";
	
	sac_ia_.align(*T_cloud);
	return sac_ia_.getFinalTransformation();
}

Matrix4f initial_alignment2(XYZCloud::Ptr T_cloud, XYZCloud::Ptr scene_cloud, FPFHCloud::Ptr T_FPFH_cloud, FPFHCloud::Ptr scene_FPFH_cloud) {
	Matrix4f transform;
	CorrespondencesPtr correspondences(new Correspondences);
	pcl::registration::CorrespondenceEstimation<FPFH, FPFH> cest;
	cest.setInputSource(T_FPFH_cloud);
	cest.setInputTarget(scene_FPFH_cloud);
	cest.determineCorrespondences(*correspondences);
	CorrespondencesPtr corr_filtered(new Correspondences);
	pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZ> rejector;
	rejector.setInputSource(T_cloud);
	rejector.setInputTarget(scene_cloud);
	rejector.setInlierThreshold(2.5);
	rejector.setMaximumIterations(1000000);
	rejector.setRefineModel(false);
	rejector.setInputCorrespondences(correspondences);
	rejector.getCorrespondences(*corr_filtered);
	transform = initial_alignment(T_cloud, scene_cloud, T_FPFH_cloud, scene_FPFH_cloud, corr_filtered);
	//pcl::registration::TransformationEstimationSVD<PointXYZ, PointXYZ> trans_est;
	//trans_est.estimateRigidTransformation(*T_cloud, *scene_cloud, *corr_filtered, transform);
	return transform;

}

Matrix4f initial_alignment(XYZCloud::Ptr T_cloud, XYZCloud::Ptr scene_cloud, SHOTCloud::Ptr T_SHOT_cloud, SHOTCloud::Ptr scene_SHOT_cloud, CorrespondencesPtr BBS) {
	CorrespondencesPtr filteredCorrespondences;
	Matrix4f transform;
	filterMatchesByFeaturesDist(*T_SHOT_cloud, *scene_SHOT_cloud, BBS, filteredCorrespondences,0.3);
	calcTransform(T_cloud, scene_cloud, filteredCorrespondences, transform);
	return transform;
}

/*void mirrorCloud(XYZCloud::Ptr tgtxyz, Eigen::Vector3f n){
	Eigen::Vector4f centroid;
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	n.normalize();
	compute3DCentroid(*tgtxyz, centroid);
	float d = -n(0)*centroid(0)-n(1)*centroid(1) - n(2)*centroid(2);
	for (int j = 0; j < tgtxyz->size(); ++j) {
		Eigen::Vector3f P = tgtxyz->at(j).getVector3fMap();
		float D = (n(0)*P(0) + n(1)*P(1) + n(2)*P(2) + d);
		n = 2 * D*n;
		P = P - n;
		tgtxyz->points[j].x = P(0);		tgtxyz->points[j].y = P(1); 		tgtxyz->points[j].z = P(2);
	}
}*/

/*void ransac(const CorrespondencesPtr &correspondences,
	const PointCloud<PointXYZ>::Ptr &keypoints_src,
	const PointCloud<PointXYZ>::Ptr &keypoints_tgt,
	Correspondences &remaining_correspondences,
	int maxIters, float threshold)
{
	
	pcl::CorrespondenceRejectorSampleConsensus<PointXYZ> ransacRej;
	ransacRej.setInputSource(keypoints_src);
	ransacRej.setInputTarget(keypoints_tgt);
	ransacRej.setInputCorrespondences(correspondences);
	ransacRej.setMaximumIterations(maxIters);
	ransacRej.setInlierThreshold(threshold);
	ransacRej.getCorrespondences(remaining_correspondences);
}*/
vector<int> project_cloud(XYZCloud::Ptr source, XYZCloud::Ptr destination, XYZCloud::Ptr resultCloud) {
	vector<int> res;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(destination);
	vector<int> indices;
	vector<float> dist;
	for (int i = 0; i < source->size(); i++) {
		PointXYZ p1 = source->at(i);
		kdtree.nearestKSearch(source->at(i), 1, indices, dist);
		res.push_back(indices[0]);
		resultCloud->push_back(destination->at(indices[0]));
	}
	return res;
}