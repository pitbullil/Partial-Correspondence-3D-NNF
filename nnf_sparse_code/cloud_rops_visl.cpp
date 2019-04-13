
#include <cloud_surface_visl.hpp>

using namespace pcl;
using namespace std;

void FastTriangulation(XYZCloud::Ptr cloud, NormalCloud::Ptr Normals, PolygonMesh::Ptr mesh, float searchRadius, float Mu, int NN) {
	PointCloud<PointNormal>::Ptr cloud_with_normals(new PointCloud<PointNormal>);
	// Concatenate the XYZ and normal fields*
	concatenateFields(*cloud, *Normals, *cloud_with_normals);

	// Create search tree*
	search::KdTree<PointNormal>::Ptr tree2(new search::KdTree<PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	GreedyProjectionTriangulation<PointNormal> gp3;
	PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(searchRadius);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(NN);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(*mesh);

}

void MLS(XYZCloud::Ptr cloud, XYZCloud::Ptr out, float searchR, float upsamplingR, float upsamplingStep) {
	MovingLeastSquares<PointXYZ, PointXYZ> mls;
	mls.setInputCloud(cloud);
	mls.setSearchRadius(0.01);
	mls.setPolynomialFit(true);
	mls.setPolynomialOrder(2);
	mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
	mls.setUpsamplingRadius(0.005);
	mls.setUpsamplingStepSize(0.003);
	PointCloud<PointXYZ>::Ptr cloud_smoothed(new PointCloud<PointXYZ>());
	mls.process(*out);
}

void clean_mesh(PolygonMesh::Ptr mesh, XYZCloud::Ptr cloud, float resolution) {

	XYZCloud::Ptr meshcloud(new XYZCloud()),newcloud(new XYZCloud());
	fromPCLPointCloud2(mesh->cloud, *meshcloud);
	vector <pcl::Vertices> triangles;
	search::KdTree<PointXYZ>::Ptr kdtree(new search::KdTree<PointXYZ>);
	kdtree->setInputCloud(cloud);
	float R;
	vector<int> map(meshcloud->size());
	vector<int> pointIdxSearch(1);
	vector<float> pointSquaredDistance(1);

	for (int i = 0; i < meshcloud->size(); i++) {
		PointXYZ point = meshcloud->at(i);
		kdtree->nearestKSearch(point, 1, pointIdxSearch, pointSquaredDistance);
		if (i == 117919) {
			cout << "NN dist:" << pointSquaredDistance[0] << endl;
		}
		if (sqrt(pointSquaredDistance[0]) < resolution) { //pointsbelonging on the cloud
			newcloud->push_back(point);
			map.at(i) = newcloud->size() - 1;
		}
		else { //points to delete
			//cout << "Bad Vertex:" << i << endl;
			map.at(i) = -1;
		}
	}
	toPCLPointCloud2(*newcloud, mesh->cloud);
	for (int i = 0; i < mesh->polygons.size(); i++) {
		Vertices v = mesh->polygons.at(i);
		int v0 = v.vertices.at(0);
		int v1 = v.vertices.at(1);
		int v2 = v.vertices.at(2);

		if (!((map.at(v0) == -1) || (map.at(v1) == -1) || (map.at(v2) == -1))) {
			v.vertices.at(0) = map.at(v0);
			v.vertices.at(1) = map.at(v1);
			v.vertices.at(2) = map.at(v2);
			triangles.push_back(v);
		}
	}
	cout << "original triangles" << mesh->polygons.size();

	cout << "cropped triangles" << triangles.size();
	mesh->polygons = triangles;
}

void PoissonReconstruction(XYZCloud::Ptr cloud, NormalCloud::Ptr Normals, PolygonMesh::Ptr mesh, int depth) {
	PointCloud<PointNormal>::Ptr cloud_with_normals(new PointCloud<PointNormal>);
	concatenateFields(*cloud, *Normals, *cloud_with_normals);
	Poisson<PointNormal> poisson;
	poisson.setDepth(depth);
	poisson.setInputCloud(cloud_with_normals);
	poisson.reconstruct(*mesh);
}