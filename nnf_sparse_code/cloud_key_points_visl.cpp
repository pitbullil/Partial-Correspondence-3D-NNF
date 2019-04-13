#include <cloud_key_points_visl.hpp>
/// Function for computing key points:



int computeKeyPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr pointsCloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints,
	KeypointTypes Method,
	float modelResolution, bool border, bool autodefault){

	// Call detectors:
	if (Method == HARRIS){

		float radius = getUserValue("radius", modelResolution*keypoints_visl::RADIUS_SEARCH_MULTIPLIER); //For normals calculation:
		float radiusSearch = getUserValue("search radius", modelResolution*keypoints_visl::RADIUS_SEARCH_MULTIPLIER);
		bool nonMaxSuppression = getUserValue("non max supression", true);
		bool refine = getUserValue("refine", false);

		computeHarris(pointsCloud, keyPoints, modelResolution, radius, radiusSearch, nonMaxSuppression, refine);

	}else if (Method == ISS) {

		float gamma21 = 0.975; //For normals calculation:
		float gamma32 = 0.975;
		int minNeighbors = 5;
		int threads = 4;
		float salientRadius = modelResolution;
		float nonMaxRadius     = modelResolution*0.66666;
		if (!(autodefault)) {
			gamma21 = getUserValue("gamma 2-1", 0.975f); //For normals calculation:
			gamma32 = getUserValue("gamma 3-2", 0.975f);
			minNeighbors = getUserValue("minimum neighbors", 5);

			border = getUserValue("border", false);
			salientRadius = getUserValue("salient radius", keypoints_visl::RADIUS_SEARCH_MULTIPLIER * modelResolution);
			nonMaxRadius = getUserValue("non max radius", keypoints_visl::NON_MAX_RADIUS_MULTIPLIER * modelResolution);
		}

		computeISS(pointsCloud,keyPoints,modelResolution,salientRadius,nonMaxRadius,gamma21,gamma32,minNeighbors,threads,border);

	}else if (Method == SIFT) {

		float minScale = getUserValue("minimum scale", modelResolution * 2); 
		int  nOctaves= getUserValue("number of octaves", 8);
		int scalesPerOctave = getUserValue("scales per octave", 8);
		float minContrast = getUserValue("minimum contrast", modelResolution / 10);
		float searchRadius = getUserValue("search radius", keypoints_visl::RADIUS_SEARCH_MULTIPLIER*modelResolution);

		computeSIFT(pointsCloud,keyPoints,modelResolution,minScale,nOctaves,scalesPerOctave,minContrast,searchRadius);

	}else if (Method == NARF) {

		float searchRadius = getUserValue("search radius", modelResolution*keypoints_visl::RADIUS_SEARCH_MULTIPLIER);
		float angularResolution = getUserValue("angular resolution", 0.5f);
		float supportSize = getUserValue("support size", modelResolution * 5);
		float noiseLevel = getUserValue("noise level", 0.0f);
		float minRange = getUserValue("minimum range",0.0f);
		float borderSize = getUserValue("border size", 0.1f);

		computeNARF(pointsCloud,keyPoints,modelResolution,searchRadius,angularResolution,supportSize,noiseLevel,minRange,borderSize);

	}else {
		std::cerr<<"keypoints method name is not legal, please enter: sift, harris or iss"<<std::endl;
		exit(0);
	}
	return 0;
}

///////////////////// Harris //////////////////////////////////////////////////////

void computeHarris(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointsCloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &keyPoints, 
	double modelResolution, float radius,float radiusSearch,bool nonMaxSuppression,bool refine)
{

	pcl::PointCloud<pcl::PointXYZI>::Ptr inKeyPoints (new pcl::PointCloud<pcl::PointXYZI>());
	pcl::HarrisKeypoint3D<pcl::PointXYZ,pcl::PointXYZI> detector; 

	detector.setNonMaxSupression (nonMaxSuppression); 
	detector.setRadius (radius); 
	detector.setRefine(refine);
	detector.setRadiusSearch (radiusSearch); 
	detector.setInputCloud(pointsCloud); 

	cout<<"Starting Harris computation"<<endl;
	detector.compute(*inKeyPoints); 

	keyPoints->resize(inKeyPoints->size());
	for(size_t i = 0; i<inKeyPoints->points.size(); ++i)
	{
		keyPoints->points[i].x = inKeyPoints->points[i].x;
		keyPoints->points[i].y = inKeyPoints->points[i].y;
		keyPoints->points[i].z = inKeyPoints->points[i].z;
	}
}


///////////////////// ISS //////////////////////////////////////////////////////

void computeISS(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointsCloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &keyPoints,
	double modelResolution,
	float salient_radius,
	float non_max_radius,
	float gamma_21, 
	float gamma_32,
	int min_neighbors, 
	int threads,
	bool border)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;

	// Compute keypoints
	iss_detector.setSearchMethod (tree);
	iss_detector.setSalientRadius (salient_radius);
	iss_detector.setNonMaxRadius (non_max_radius);
	iss_detector.setThreshold21 (gamma_21);
	iss_detector.setThreshold32 (gamma_32);
	iss_detector.setMinNeighbors (min_neighbors);
	iss_detector.setNumberOfThreads (threads);
	iss_detector.setInputCloud (pointsCloud);
	if (border){
		double iss_normal_radius_ = 10* modelResolution;
		double iss_border_radius_ = 10* modelResolution;
		std::cout<<"Border radius:"<<iss_border_radius_<<std::endl;
		iss_detector.setNormalRadius (iss_normal_radius_);
		iss_detector.setBorderRadius (iss_border_radius_);
	}
	iss_detector.compute (*keyPoints);
}

///////////////////// SIFT //////////////////////////////////////////////////////

void computeSIFT(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointsCloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &keyPoints, 
	double modelResolution,
	float minScale,
	int nOctaves,
	int scalesPerOctave,
	float minContrast,
	float searchRadius)
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
	pcl::PointCloud<pcl::PointWithScale> result;
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());

	// Estimate Normals :
	std::cout << "Calculating normals" << std::endl;
	ne.setInputCloud(pointsCloud);
	ne.setSearchMethod(tree_n);
	ne.setRadiusSearch(modelResolution*keypoints_visl::RADIUS_SEARCH_MULTIPLIER);
	ne.compute(*cloud_normals);
	
	// Copy the xyz info from pointsCloud and add it to cloud_normals as the xyz field in PointNormals estimation is zero

	for(size_t i = 0; i<cloud_normals->points.size(); ++i)
	{
		cloud_normals->points[i].x = pointsCloud->points[i].x;
		cloud_normals->points[i].y = pointsCloud->points[i].y;
		cloud_normals->points[i].z = pointsCloud->points[i].z;
	}

	
	std::cout<<"Calculating key points"<<std::endl;
	sift.setSearchMethod(tree);
	sift.setRadiusSearch(searchRadius);
	sift.setScales(minScale, nOctaves, scalesPerOctave);
	sift.setMinimumContrast(minContrast);
	sift.setInputCloud(cloud_normals);
	sift.compute(result);

	//Convert into standard point cloud:
	copyPointCloud(result, *keyPoints);
}

/******************************  NARF  ***************************************/

void computeNARF(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointsCloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &keyPoints, 
	double modelResolution,float searchRadius,
	float angularResolution, float supportSize,
	float noiseLevel,float minRange,float borderSize){

	
		pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
		pcl::PointCloud<pcl::PointXYZ>& point_cloud = *pointsCloud;
		Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
		boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
		pcl::RangeImage& range_image = *range_image_ptr; 

		//Create range image from cloud point:
		scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                               point_cloud.sensor_origin_[1],
                                                               point_cloud.sensor_origin_[2])) *
                        Eigen::Affine3f (point_cloud.sensor_orientation_);

		range_image.createFromPointCloud (point_cloud, angularResolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                   scene_sensor_pose, coordinate_frame, noiseLevel, minRange, borderSize);

		// -----Extract NARF keypoints-----
		pcl::RangeImageBorderExtractor range_image_border_extractor;
		pcl::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor);
		narf_keypoint_detector.setRangeImage (&range_image);
		narf_keypoint_detector.getParameters ().support_size = supportSize;

		pcl::PointCloud<int> keypoint_indices;
			narf_keypoint_detector.compute (keypoint_indices);
		// Copy results to key points:
		keyPoints->resize (keypoint_indices.points.size ());
		for (size_t i=0; i<keypoint_indices.points.size (); ++i)
			keyPoints->points[i].getVector3fMap () = range_image.points[keypoint_indices.points[i]].getVector3fMap ();



}