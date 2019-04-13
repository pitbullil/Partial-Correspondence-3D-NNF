#include <cloud_features_visl.hpp>
#define PCL_NO_PRECOMPILE
using namespace pcl;
void computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr pointsCloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints,
	pcl::PointCloud<pcl::Normal>::Ptr normals,
	float modelResolution,
	bool Autodefault,
	float multiplier)
{
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation(8);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);

	float searchRadius = getUserValue("search radiuses", modelResolution*multiplier, Autodefault);

    normal_estimation.setInputCloud (keyPoints);
	normal_estimation.setSearchMethod (kdtree);
	normal_estimation.setSearchSurface(pointsCloud);
	normal_estimation.setRadiusSearch (searchRadius);
	normal_estimation.compute (*normals);
}

void computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr pointsCloud,
	pcl::PointCloud<pcl::Normal>::Ptr normals,
	float searchRadius,
	bool Autodefault)
{
	cout << "I'm here!";
	pcl::search::KdTree<pcl::PointXYZ>::Ptr search(new pcl::search::KdTree<pcl::PointXYZ>());
	NormalEstimation<PointXYZ, Normal> ne;
	ne.setInputCloud(pointsCloud);
	ne.setSearchMethod(search);
	ne.setRadiusSearch(searchRadius);
	//Eigen::Vector4f centroid;
	//compute3DCentroid(*pointsCloud, centroid);
	//ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
	ne.compute(*normals);
	/*for (size_t i = 0; i < normals->size(); ++i)
	{
		normals->points[i].normal_x *= -1;
		normals->points[i].normal_y *= -1;
		normals->points[i].normal_z *= -1;
	}*/
}

void computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr pointsCloud,
	pcl::PointCloud<pcl::Normal>::Ptr normals,
	int NN,
	bool Autodefault) {
	NormalEstimationOMP<PointXYZ, Normal> ne;
	ne.setNumberOfThreads(7);
	ne.setInputCloud(pointsCloud);
	ne.setKSearch(NN);
	Eigen::Vector4f centroid;
	compute3DCentroid(*pointsCloud, centroid);
	ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
	ne.compute(*normals);
	for (size_t i = 0; i < normals->size(); ++i)
	{
		normals->points[i].normal_x *= -1;
		normals->points[i].normal_y *= -1;
		normals->points[i].normal_z *= -1;
	}
}


void computeROPSFeatures(
	XYZCloud::Ptr pointsCloud,
	XYZCloud::Ptr keyPoints,
	//PointIndicesPtr indices,
	vector<Vertices> triangles,
	ROPSCloud& rops,
	float support_radius) {

	unsigned int number_of_partition_bins = 5;
	unsigned int number_of_rotations = 3;
	search::KdTree<PointXYZ>::Ptr search_method(new search::KdTree<PointXYZ>);
	search_method->setInputCloud(pointsCloud);

	ROPSEstimation <PointXYZ, Histogram<135>> feature_estimator;
	feature_estimator.setSearchMethod(search_method);
	feature_estimator.setSearchSurface(pointsCloud);
	feature_estimator.setInputCloud(keyPoints);
	//feature_estimator.setIndices(indices);
	feature_estimator.setTriangles(triangles);
	feature_estimator.setRadiusSearch(support_radius);
	feature_estimator.setNumberOfPartitionBins(number_of_partition_bins);
	feature_estimator.setNumberOfRotations(number_of_rotations);
	feature_estimator.setSupportRadius(support_radius);

	pcl::PointCloud<Histogram<135>>::Ptr histograms(new pcl::PointCloud <Histogram<135>>());
	feature_estimator.compute(*histograms);
	rops.resize(histograms->size());
	for (int i = 0; i < histograms->size(); i++) {
		for (int j = 0; j < 135; j++) {
			rops.points[i].descriptor[j] = histograms->at(i).histogram[j];
		}
	}
}

/***************************** spin images ***********************************/

void computeSIFeatures(
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints,
	pcl::PointCloud<pcl::Normal>::Ptr normals,
	pcl::PointCloud<pcl::Histogram<153>>::Ptr spin_images,
	double modelResolution
)
{
	std::string input = "";
	std::string::size_type sz;

	int min_neigh (8);
	std::cout<<"Please Enter min_neigh argument: "<<std::endl;
	getline(cin, input);
	if (!input.empty()){
		min_neigh=std::stoi(input,&sz);
	}
	
		int image_width (3);
	std::cout<<"Please Enter image_width argument: "<<std::endl;
	getline(cin, input);
	if (!input.empty()){
		image_width=std::stoi(input,&sz);
	}
	
	double support_angle (0.05);
	std::cout<<"Please Enter support_angle argument: "<<std::endl;
	getline(cin, input);
	if (!input.empty()){
		support_angle=std::stod(input,&sz);
	}

	float radius_search (modelResolution*dis3::RADIUS_SEARCH_MULTIPLIER);
	std::cout<<"Please Enter radius_search argument: "<<std::endl;
	getline(cin, input);
	if (!input.empty()){
		radius_search=std::stof(input,&sz);
	}
// Setup spin image computation

	std::cout <<"Setting kdtree"<<std::endl;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> > spin_image_descriptor( image_width, support_angle, min_neigh );
	
	std::cout <<"Setting radius search"<<std::endl;
	spin_image_descriptor.setRadiusSearch (radius_search);

	//For pointCloud1:
	spin_image_descriptor.setInputCloud (keyPoints);
	spin_image_descriptor.setInputNormals (normals);
	spin_image_descriptor.setSearchMethod (kdtree);
	std::cout <<"Computing SI for cloud :"<<std::endl;
	spin_image_descriptor.compute (*spin_images);
	std::cout<<"Number of SI features:"<<spin_images->size()<<std::endl;
	// Write to file:
	

}

/*****************************Fast Point features histogram ***********************************/
void computeFPFHFeatures(
	XYZCloud::Ptr pointsCloud,
	XYZCloud::Ptr keyPoints,
	NormalCloud::Ptr normals,
	FPFHCloud& fpfhs,
	float searchRadius)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud (keyPoints);
    fpfh_est.setInputNormals (normals);
	fpfh_est.setSearchSurface(pointsCloud);
	fpfh_est.setSearchMethod(kdtree);
    fpfh_est.setRadiusSearch (searchRadius);
    fpfh_est.compute (fpfhs);

}

void computeSFPFHFeatures(
	XYZCloud::Ptr pointsCloud,
	XYZCloud::Ptr keyPoints,
	NormalCloud::Ptr normals,
	SFPFHCloud& sfpfhs,
	Eigen::MatrixXf* D,
	float searchRadius) {
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	SFPFHEstimation<pcl::PointXYZ, pcl::Normal,SFPFH> fpfh_est;
	fpfh_est.setInputCloud(keyPoints);
	fpfh_est.setInputNormals(normals);
	fpfh_est.setSearchSurface(pointsCloud);
	fpfh_est.setSearchMethod(kdtree);
	fpfh_est.setRadiusSearch(searchRadius);
	fpfh_est.setDistances(D);
	fpfh_est.compute(sfpfhs);

}

void computeGFPFHFeatures(
	XYZCloud::Ptr pointsCloud,
	XYZCloud::Ptr keyPoints,
	NormalCloud::Ptr normals,
	GFPFHCloud& sfpfhs,
	Eigen::MatrixXf* D,
	float searchRadius) {
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	GFPFHEstimation<pcl::PointXYZ, pcl::Normal,GFPFH> fpfh_est;
	fpfh_est.setInputCloud(keyPoints);
	fpfh_est.setInputNormals(normals);
	fpfh_est.setSearchSurface(pointsCloud);
	fpfh_est.setSearchMethod(kdtree);
	fpfh_est.setRadiusSearch(searchRadius);
	fpfh_est.setDistances(D);
	fpfh_est.compute(sfpfhs);

}

/*****************************Point features histogram ***********************************/
void computePFHFeatures(
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointsCloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints,
	pcl::PointCloud<pcl::Normal>::Ptr normals,
	pcl::PointCloud<pcl::PFHSignature125>& pfhs,
	float searchRadius)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_est;
	pfh_est.setInputCloud(keyPoints);
	pfh_est.setInputNormals(normals);
	pfh_est.setRadiusSearch(searchRadius);
	pfh_est.setSearchSurface(pointsCloud);
	pfh_est.setSearchMethod(kdtree);
	pfh_est.compute(pfhs);
}

void computeGPFHFeatures(
	XYZCloud::Ptr pointsCloud,
	XYZCloud::Ptr keyPoints,
	NormalCloud::Ptr normals,
	GPFHCloud& sfpfhs,
	Eigen::MatrixXf* D,
	float searchRadius) {
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	GPFHEstimation<pcl::PointXYZ, pcl::Normal, GPFH> fpfh_est;
	fpfh_est.setInputCloud(keyPoints);
	fpfh_est.setInputNormals(normals);
	fpfh_est.setSearchSurface(pointsCloud);
	fpfh_est.setSearchMethod(kdtree);
	fpfh_est.setRadiusSearch(searchRadius);
	fpfh_est.setDistances(D);
	fpfh_est.compute(sfpfhs);

}

/***************************** SHOT features ***********************************/
void computeSHOTFeatures(
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointsCloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints,
	pcl::PointCloud<pcl::Normal>::Ptr normals,
	pcl::PointCloud<pcl::SHOT352>& shot_desc,
	float searchRadius
	) {

	pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
	shot.setInputCloud(keyPoints);
	shot.setSearchSurface(pointsCloud);
	shot.setInputNormals(normals);
	shot.setRadiusSearch(searchRadius);
	shot.compute(shot_desc);
}

/*void calculateFeatures(XYZCloud::Ptr pointsCloud,NormalCloud::Ptr N, FeatureCloud& F) {
	string Type = F.ReturnType();
	if (Type.compare(0, 4, "fpfh") == 0) {
		FPFH1Cloud& fpfh = dynamic_cast<FPFH1Cloud&>(F);
		computeFPFHFeatures(pointsCloud, pointsCloud, N, fpfh.descriptor, 0.2);
	}
	else if (Type.compare(0, 3, "pfh") == 0) {
		PFH1Cloud& pfh = dynamic_cast<PFH1Cloud&>(F);
	}
}

void FPFH1Cloud::compute() {
	computeFPFHFeatures(basecloud, keypoints, N, descriptor, p_f.searchRadius);
}

void PFH1Cloud::compute() {
	computePFHFeatures(basecloud, keypoints, N, descriptor, p_f.searchRadius);
}*/