#include <cloud_registration_visl.hpp>

POINT_CLOUD_REGISTER_POINT_STRUCT(Histogram<153>,
	(float[153], histogram, histogram)
	);

using namespace pcl;
using namespace pcl::registration;

template<class Vec>
float calcNorm(Vec vec,int size) {
	float dist(0);
	for (int j = 0; j < size; j++)
		dist += pow(vec[j], 2);
	dist = sqrt(dist);
	return dist;
}

// Find the closest point in cloud 2 (if exist) in an elipsoid for each point in cloud 1 ; if ratio of resolution between x and y axes is different then 1- insert the value
void findGeoMatches(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints1, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints2, 
	pcl::CorrespondencesPtr correspondences, 
	float threshold) {

	std::vector<int> pointIdx(1);
	std::vector<float> squaredDistance(1);
	pcl::Correspondence corr;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	float localResolution = 0;
	correspondences->clear();
	for (int i = 0; i < keyPoints1->size(); i++) {

		
		kdtree.setInputCloud(keyPoints2);
		kdtree.nearestKSearch(keyPoints1->at(i), 1, pointIdx, squaredDistance);
		if (sqrt(squaredDistance[0])<threshold) {
			corr.index_query = i;
			corr.index_match = pointIdx[0];
			correspondences->push_back(corr);
		}
	}
}

void corrrespondencesToPoints(CorrespondencesPtr correspondences,
	PointCloud<PointXYZ>::Ptr keyPoints1,
	PointCloud<PointXYZ>::Ptr keyPoints2,
	PointCloud<PointXYZ>::Ptr keyPoints1Matches,
	PointCloud<PointXYZ>::Ptr keyPoints2Matches
	)
{
	for (int i = 0; i < correspondences->size(); i++) {
		keyPoints1Matches->push_back(keyPoints1->points[correspondences->at(i).index_query]);
		keyPoints2Matches->push_back(keyPoints2->points[correspondences->at(i).index_match]);
	}
}

void filterMatchesByFeaturesDist(PointCloud<FPFHSignature33>& features1, 
	PointCloud<FPFHSignature33>& features2, 
	CorrespondencesPtr correspondences,
	CorrespondencesPtr filteredCorrespondences,
	float thresholdPrecentage=0.3) {

	int descriptorSize = features1.points[1].descriptorSize();
	float dist, feature1Norm, feature2Norm, threshold;
	std::vector<float> distVector;

	for(int i = 0; i < correspondences->size();i++) {
		dist = 0;
	//	std::cout << i<<std::endl;
		for (int j = 0; j < descriptorSize; j++) 
			distVector.push_back(features1.points[correspondences->at(i).index_query].histogram[j] - features2.points[correspondences->at(i).index_match].histogram[j]);

		dist = calcNorm(distVector,descriptorSize);
		feature1Norm = calcNorm(features1.points[correspondences->at(i).index_query].histogram, descriptorSize);
		feature2Norm = calcNorm(features2.points[correspondences->at(i).index_match].histogram, descriptorSize);
		threshold = feature1Norm > feature2Norm ? feature2Norm : feature1Norm;
		threshold = threshold* thresholdPrecentage;
		if (threshold > dist)
			filteredCorrespondences->push_back(correspondences->at(i));
	}
}

	void filterMatchesByFeaturesDist(PointCloud<SHOT352>& features1,
		PointCloud<SHOT352>& features2,
		CorrespondencesPtr correspondences,
		CorrespondencesPtr filteredCorrespondences,
		float thresholdPrecentage =0.3
		){
		int descriptorSize = features1.points[1].descriptorSize();
		float error, feature1Norm, feature2Norm,threshold;
		std::vector<float> distVector;
		int i = 0;
		for (int i = 0; i < correspondences->size(); i++) {
			error = 0;
			for (int j = 0; j < descriptorSize; j++)
				distVector.push_back(features1.points[correspondences->at(i).index_query].descriptor[j] - features2.points[correspondences->at(i).index_match].descriptor[j]);

			error = calcNorm(distVector, descriptorSize);
			feature1Norm = calcNorm(features1.points[correspondences->at(i).index_query].descriptor, descriptorSize);
			feature2Norm = calcNorm(features2.points[correspondences->at(i).index_match].descriptor, descriptorSize);
			threshold = feature1Norm > feature2Norm ? feature2Norm : feature1Norm;
			threshold = threshold*thresholdPrecentage;
			if (threshold > error)
				filteredCorrespondences->push_back(correspondences->at(i));
		}
}


void ransac(const CorrespondencesPtr &correspondences,
	const PointCloud<PointXYZ>::Ptr &keypoints_src,
	const PointCloud<PointXYZ>::Ptr &keypoints_tgt,
	Correspondences &remaining_correspondences,
	int maxIters,float threshold)
{

	CorrespondenceRejectorSampleConsensus<PointXYZ> ransacRej;
	ransacRej.setInputSource(keypoints_src);
	ransacRej.setInputTarget(keypoints_tgt);
	ransacRej.setInputCorrespondences(correspondences);
    ransacRej.setMaximumIterations(maxIters);
	ransacRej.setInlierThreshold(threshold);
	ransacRej.getCorrespondences(remaining_correspondences);
}

void calcTransform(PointCloud<PointXYZ>::Ptr keyPoints1,
	PointCloud<PointXYZ>::Ptr keyPoints2,
	CorrespondencesPtr good_correspondences,
	Eigen::Matrix4f &transform) {
	TransformationEstimationSVD<PointXYZ, PointXYZ> trans_est;
	trans_est.estimateRigidTransformation(*keyPoints1, *keyPoints2, *good_correspondences, transform);
}


void matchKeyPointsFeatures(
	PointCloud<PointXYZ>::Ptr keyPoints1,
	PointCloud<PointXYZ>::Ptr keyPoints2,
	PointCloud<FPFHSignature33>::Ptr  fpfhs1,
	PointCloud<FPFHSignature33>::Ptr  fpfhs2,
	float modelResolution,
	CorrespondencesPtr correspondences)
{
	CorrespondenceEstimation<FPFHSignature33, FPFHSignature33> estimator;
	estimator.setInputSource(fpfhs1);
	estimator.setInputTarget(fpfhs2);
	estimator.determineReciprocalCorrespondences(*correspondences);
}

void matchKeyPointsFeatures(
	PointCloud<PointXYZ>::Ptr keyPoints1,
	PointCloud<PointXYZ>::Ptr keyPoints2,
	PointCloud<SHOT352>::Ptr shot1,
	PointCloud<SHOT352>::Ptr shot2,
	float modelResolution,
	CorrespondencesPtr correspondences)
{
	//PointCloud<PointXYZ>::Ptr keyPoints2_transformed(new PointCloud<PointXYZ>);
	CorrespondenceEstimation<SHOT352, SHOT352> estimator;
	estimator.setInputSource(shot1);
	estimator.setInputTarget(shot2);
	estimator.determineReciprocalCorrespondences(*correspondences);
}

float get_icp_transform(XYZCloud::Ptr srcxyz, XYZCloud::Ptr tgtxyz, Eigen::Matrix4f& final_transform) {
	XYZCloud::Ptr tmp(new XYZCloud());
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud(srcxyz);
	icp.setInputTarget(tgtxyz);
	icp.setMaximumIterations(100000);
	icp.align(*tmp);
	final_transform = icp.getFinalTransformation();
	print4x4Matrix(final_transform.cast<float>());
	cout << "blah";
	return icp.getFitnessScore();
}