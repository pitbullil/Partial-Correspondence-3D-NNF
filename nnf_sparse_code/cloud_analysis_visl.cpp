#include <cloud_analysis_visl.hpp>

using namespace std;
using namespace pcl;
using namespace dis3;

#ifdef MESHGRAPH_H
void integral_mesh(XYZICloud::Ptr input, graphMesh graph, XYZICloud::Ptr output, float R) {
	for (int i = 0; i < input->size(); i++) {
		float intensity = 0;
		vector<int> indices = graph.LimitedSSSP(i, R);
		float N = indices.size();
		for (int j = 0; j < N; j++) {
			intensity += input->at(indices[j]).intensity;
		}
		output->at(i).intensity = intensity / N;
	}
}
#endif

void saveResolution(string filename, cloud_statistics_s& S) {
	ofstream resolution_file;
	resolution_file.open(filename);
	resolution_file << S.MeanResolution << std::endl << S.Range <<std::endl << S.MaxResolution;
	resolution_file.close();
}
void loadResolution(string filename, cloud_statistics_s& S) {
	string line;
	ifstream resolution_file(filename);
	string::size_type sz;
	if (resolution_file.is_open())
	{
		getline(resolution_file, line);
		S.MeanResolution = stof(line, &sz);
		getline(resolution_file, line);
		S.Range = stof(line, &sz);
		getline(resolution_file, line);
		S.MaxResolution = stof(line, &sz);
		resolution_file.close();
	}
	else cout << "Unable to open file";
}

float findMSE(XYZCloud::Ptr keyPoints1,
	XYZCloud::Ptr keyPoints2_transformed,
	CorrespondencesPtr goodCorrespondences)
{
	float MSE = 0;
	for (int i = 0; i < goodCorrespondences->size(); i++) {
		MSE += pow(keyPoints2_transformed->points[goodCorrespondences->at(i).index_match].x - keyPoints1->points[goodCorrespondences->at(i).index_query].x, 2) +
			pow(keyPoints2_transformed->points[goodCorrespondences->at(i).index_match].y - keyPoints1->points[goodCorrespondences->at(i).index_query].y, 2) +
			pow(keyPoints2_transformed->points[goodCorrespondences->at(i).index_match].z - keyPoints1->points[goodCorrespondences->at(i).index_query].z, 2);
		//	cout << MSE << endl;

	}
	MSE = MSE / goodCorrespondences->size();
	return MSE;
}

void findResolution(XYZCloud::Ptr cloud, cloud_statistics_s& S, string filename, bool save)
/*This computes the mean resolution of a point cloud
INPUT:
cloud - the point cloud we find a resolution for
filename - A file containing normals if one exists, if not - the normals will be saved to this file

OUTPUTS:
meanResolution - the clouds mean distance between points
meanRange - the mean distance of a point to (0,0,0) in the cloud 
*/
{
	if (ifstream(filename)) {
		cout << "Loading resolution from file" << endl;
		loadResolution(filename, S);
		
	}else{
	

		int count=1,  numberOfSamples(1000);
		bool random = false;
		KdTreeFLANN<pcl::PointXYZ> kdtree;
		PointCloud<PointXYZ>::iterator it = cloud->begin();
		PointXYZ point;
		vector<int> pointIdxSearch(1);
		vector<float> pointSquaredDistance(1);
		srand(clock());
		kdtree.setInputCloud(cloud);
		S.MaxResolution = 0;
		S.MeanResolution = 0;
		S.Range = 0;
		float R;
		while (it!=cloud->end()) {

			point.x = it->x;
			point.y = it->y;
			point.z = it->z;
			
			if (kdtree.nearestKSearch(point, 2, pointIdxSearch, pointSquaredDistance) > 0) {
				R = sqrt(pointSquaredDistance[1]);
				if (R>S.MaxResolution) {
					S.MaxResolution = R;
				}
				S.MeanResolution += R;
				S.Range += sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
				count++;
			}

			it++;
		}
		S.MeanResolution = S.MeanResolution / count;
		S.Range = S.Range / count;
		if (save) {
			saveResolution(filename, S);
		}
	}
	
}

XYZCloud::Ptr filterpointsonsurface(XYZCloud::Ptr sourcePointCloud,
	XYZCloud::Ptr surfacePointCloud,
	float threshold) {
	XYZCloud::Ptr result(new XYZCloud);
	KdTreeFLANN<XYZ> kdtree;
	kdtree.setInputCloud(surfacePointCloud);
	XYZCloud::iterator it = sourcePointCloud->begin();
	XYZ point;
	vector<int> pointIdxSearch(1);
	vector<float> pointSquaredDistance(1);
	srand(clock());
	while (it != sourcePointCloud->end()) {
		point.x = it->x;
		point.y = it->y;
		point.z = it->z;
		if (kdtree.nearestKSearch(point, 1, pointIdxSearch, pointSquaredDistance) > 0) {
			if (pointSquaredDistance[0] < threshold) {
				result->push_back(point);
			}
		}
		it++;
	}
	return result;
}

void pointCloudCut(XYZCloud::Ptr sourcePointCloud,
	XYZCloud::Ptr targetPointCloud,
	int startingAngle,
	int endAngle) {
	const float PI(3.14159265359);
	float pointAngle;
	for (int i = 0; i < sourcePointCloud->size(); i++) {
		pointAngle = atan2(sourcePointCloud->points[i].y, sourcePointCloud->points[i].x) * 180 / PI;
		if ((pointAngle > startingAngle) && (pointAngle < endAngle))
			targetPointCloud->push_back(sourcePointCloud->points[i]);
	}
}

vector<int> appxFarthestPoints(CloudT::Ptr cloud) {
	float R_max = 0;
	vector<Vertices> polygons;
	pcl::ConvexHull<PointT> hull_calculator;
	CloudT::Ptr hull(new CloudT);
	hull_calculator.setInputCloud(cloud);
	hull_calculator.reconstruct(*hull, polygons);
	vector<int> indices;
	for (int i = 0; i < polygons.size(); i++) {
		indices.push_back(polygons.at(i).vertices.at(0));
	}
	vector<int> p(2);
	for (int i = 0; i < indices.size(); i++) {
		for (int j = 0; j < indices.size(); j++) {
			float R = (cloud->at(i).getVector3fMap() - cloud->at(j).getVector3fMap()).norm();
			if (R > R_max) {
				p.at(0) = i; p.at(1) = j; R_max = R;
			}
		}
	}
	return p;
}

pcl::PointIndicesPtr extractNNindices(XYZCloud::Ptr keypoints, XYZCloud::Ptr target, int K) {
	pcl::PointIndicesPtr indices = boost::shared_ptr <pcl::PointIndices>(new pcl::PointIndices());
	KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(target);
	XYZCloud::iterator it = keypoints->begin();
	XYZ point;
	vector<int> pointIdxSearch(1);
	vector<float> pointSquaredDistance(1);
	srand(clock());
	while (it != keypoints->end()) {
		point.x = it->x;
		point.y = it->y;
		point.z = it->z;
		if (kdtree.nearestKSearch(point, K, pointIdxSearch, pointSquaredDistance) > 0) {
			for (int i = 0; i < K; i++) {
				indices->indices.push_back(pointIdxSearch[K]);
			}
		}
		it++;
	}
	return indices;
}

std::vector<int> Region_Growth(XYZCloud::Ptr cloud, vector<int> init_region, float radius) {
	vector<bool> restemp(cloud->size());
	vector<int> result;
	KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	vector<int> pointIdxSearch;
	vector<float> pointSquaredDistance;

	for (int i = 0; i < init_region.size(); i++) {
		restemp.at(init_region.at(i)) = true;
		kdtree.radiusSearch(init_region.at(i), radius, pointIdxSearch, pointSquaredDistance);
		for (int j = 0; j < pointIdxSearch.size(); j++) {
			restemp.at(pointIdxSearch.at(j)) = true;
		}
	}
	for (int i = 0; i < restemp.size(); i++) {
		if (restemp.at(i)) result.push_back(i);
	}
	return result;
}

