#include <cloud_visualization_visl.hpp>

using namespace dis3;
using namespace pcl;

bool color_point(const pcl::PointXYZI point_in, pcl::PointXYZRGB& point) {
	if (point_in.intensity == INFINITY) return false;
	point.r = hcmap[static_cast<int>(floor(point_in.intensity))][0];
	point.g = hcmap[static_cast<int>(floor(point_in.intensity))][1];
	point.b = hcmap[static_cast<int>(floor(point_in.intensity))][2];
	return true;
}

bool color_pointi(const pcl::PointXYZI point_in, pcl::PointXYZRGB& point) {
	if (point_in.intensity == INFINITY) return false;
	point.r = icmap[static_cast<int>(floor(point_in.intensity))][0];
	point.g = icmap[static_cast<int>(floor(point_in.intensity))][1];
	point.b = icmap[static_cast<int>(floor(point_in.intensity))][2];
	return true;

}

bool color_point(const pcl::PointXYZL point_in, pcl::PointXYZRGB& point) {
	point.r = lcmap[point_in.label % 1000][0];
	point.g = lcmap[point_in.label % 1000][1];
	point.b = lcmap[point_in.label % 1000][2];
	return true;
}

bool color_pointi(const pcl::PointXYZL point_in, pcl::PointXYZRGB& point) { return false; };

void visualize_keyPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints2,
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointsCloud1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointsCloud2,
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointsCloud2_transformed,
	pcl::CorrespondencesPtr correpsondences,
	pcl::CorrespondencesPtr ransacCorrepsondences)
{
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler1(keyPoints1, 0, 255, 0);
	viewer.addPointCloud(keyPoints1, keypoints_color_handler1, "keypoints1");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler2(keyPoints2, 0, 255, 255);
	viewer.addPointCloud(keyPoints2, keypoints_color_handler2, "keypoints2");


	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler1(pointsCloud1, 0, 255, 0);
	viewer.addPointCloud(pointsCloud1, cloud_color_handler1, "cloud1");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler2(pointsCloud2, 0, 255, 255);
	viewer.addPointCloud(pointsCloud2, cloud_color_handler2, "cloud2");
	double r, g, b;
	std::stringstream ss_line;
	// add matches before ransac:
	for (int i = 0; i < correpsondences->size(); i++) {
		ss_line << "correspondence_line" << i;
		viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(keyPoints1->points[correpsondences->at(i).index_query], keyPoints2->points[correpsondences->at(i).index_match], 255, 255, 0, ss_line.str());
	}
	// Add ransac matches:
	if (!ransacCorrepsondences->empty()) {
		for (int i = 0; i < ransacCorrepsondences->size(); i++) {
			ss_line << "ransac_correspondence_line" << i;
			r = 0;//double(rand() % 256) / 255;
			g = 255;//double(rand() % 256) / 255;
			b = 0;//double(rand() % 256) / 255;
			viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(keyPoints1->points[ransacCorrepsondences->at(i).index_query], keyPoints2->points[ransacCorrepsondences->at(i).index_match], r, g, b, ss_line.str());
		}
	}

	/*if (!pointsCloud2_transformed->empty()) {
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler3(pointsCloud2_transformed, 255, 255, 255);
		viewer.addPointCloud(pointsCloud2_transformed, cloud_color_handler2, "cloud2_transformed");
	}*/

	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints1");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints2");
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
}

/*void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
	void* viewer_void)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	if (event.getKeySym() == "s" && event.keyDown())
	{
		std::cout << "s was pressed => saving snapshot" << std::endl;

		char str[512];
		for (unsigned int i = 0; i < text_id; ++i)
		{
			sprintf(str, "text#%03d", i);
			viewer->removeShape(str);
		}
		text_id = 0;
	}
}*/

void normal_visualization(XYZCloud::Ptr cloud ,PolygonMeshPtr mesh, NormalCloud::Ptr normals, vector<int> points) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPolygonMesh(*mesh, "mesh");
	viewer->addPointCloudNormals<XYZ, N>(cloud, normals, 10, 0.1, "normals");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}

}

void gt_visualization(XYZCloud::Ptr gt_cloud,
	vector<vector<int>> gt,
	PolygonMeshPtr gt_mesh,
	PolygonMeshPtr cutmesh,
	XYZCloud::Ptr cut_cloud,
	vector<int> points) {
	Eigen::Vector4f centroid; Eigen::Vector4f farthest_point;
	XYZCloud::Ptr cloud_cloud(new XYZCloud());
	XYZCloud::Ptr orig_cloud(new XYZCloud());

	fromPCLPointCloud2(gt_mesh->cloud, *cloud_cloud);
	fromPCLPointCloud2(gt_mesh->cloud, *orig_cloud);

	pcl::compute3DCentroid(*cloud_cloud, centroid);
	pcl::getMaxDistance(*cloud_cloud, centroid, farthest_point);
	Eigen::Vector3f radi = (farthest_point - centroid).head<3>();
	XYZCloud::Ptr pcacloud(new XYZCloud()), cut2_cloud(new XYZCloud());
	copyPointCloud(*gt_cloud, *pcacloud);
	copyPointCloud(*cut_cloud, *cut2_cloud);

	pcl::PCA<XYZ> pca(*cloud_cloud);
	Eigen::Matrix3f xyz_coords = pca.getEigenVectors();
	Eigen::Vector3f ax(xyz_coords(0), xyz_coords(1), xyz_coords(2));
	Eigen::Vector3f ax1(xyz_coords(3), xyz_coords(4), xyz_coords(5));
	Eigen::Vector3f ax2(xyz_coords(6), xyz_coords(7), xyz_coords(8));
	string input = "";
	float c1 = 1 / sqrt(3);	float c2 = 1 / sqrt(3); float c3 = 1 / sqrt(3);
	pcl::demeanPointCloud(*pcacloud, centroid, *pcacloud);
	pcl::demeanPointCloud(*cloud_cloud, centroid, *cloud_cloud);

	pcl::compute3DCentroid(*cut_cloud, centroid);
	pcl::demeanPointCloud(*cut2_cloud, centroid, *cut2_cloud);

	cout << "choose ratio of primary ax \n";
	string::size_type sz;
	std::getline(cin, input);
	if (check_num(input)) {
		c1 = stof(input, &sz);
	}

	cout << "choose ratio of secondary ax \n";
	std::getline(cin, input);
	if (check_num(input)) {
		c2 = stof(input, &sz);
	}

	cout << "choose ratio of minor ax \n";
	std::getline(cin, input);
	if (check_num(input)) {
		c3 = stof(input, &sz);
	}

	Eigen::Vector3f n_1 = c1*ax.normalized() + c2*ax1.normalized() + c3*ax2.normalized();
	n_1.normalize();
	Eigen::Vector4f n(n_1(0), n_1(1), n_1(2), 0);

	float multiplier = 2;
	cout << "choose Number of scene radiuses from Center \n";
	std::getline(cin, input);
	if (check_num(input)) {
		multiplier = stof(input, &sz);
	}

	Eigen::Vector4f ntrans = n * multiplier * radi.norm();
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	if (multiplier != 0) {
		transform_1(0, 3) = ntrans(0);
		transform_1(1, 3) = ntrans(1);
		transform_1(2, 3) = ntrans(2);
	}
	XYZCloud cloud;
	transformPointCloud(*cut2_cloud, *cut2_cloud, transform_1);
	XYZRGBCloud::Ptr shownCorrs(new XYZRGBCloud());
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("3D Viewer 1"));
	viewer2->setBackgroundColor(255, 255, 255);
	viewer2->initCameraParameters();
	toPCLPointCloud2(*cloud_cloud, gt_mesh->cloud);
	toPCLPointCloud2(*cut2_cloud, cutmesh->cloud);
	viewer2->addPolygonMesh(*cutmesh, "temp1");
	viewer2->addPolygonMesh(*gt_mesh, "scn1");
	float template_point_size = 5;
	float point_size_corrs = 5;
	int k = 0;
	bool alter = false;
	for (int i = 0; i < points.size(); i++) {
		pcl::PointXYZRGB  p_tgt;
		copyPoint(cut2_cloud->at(points.at(i)), p_tgt);
		shownCorrs->push_back(p_tgt);
		for (int j = 0; j < gt.at(points.at(i)).size(); j++) {
			pcl::PointXYZRGB  p_src;
			copyPoint(pcacloud->at(gt.at(points.at(i)).at(j)), p_src);
			int r = 0;				int g = 0;    int b = 0;

			std::stringstream ss("line");
			ss << k;
			std::stringstream sss("spheresource");
			sss << k;
			std::stringstream ssss("spheretarget");
			ssss << k++;
			if (alter)
			{
				viewer2->addLine(p_src, p_tgt, r, g, b, sss.str());
				viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, sss.str());
			}
			else
			{
				viewer2->addLine(p_src, p_tgt, r, g, b, ss.str());
				viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, ss.str());
			}
			alter = !alter;
		}
	}
	string corrs = "Corrs";
	viewer2->addPointCloud(shownCorrs, corrs);
	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, template_point_size, corrs);

	//savePointCloudToFile(shownCorrs, "snapshot.ply");
	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_corrs, corrs);
	viewer2->setWindowBorders(true);
	viewer2->resetCamera();
	while (!viewer2->wasStopped())
	{
		viewer2->spinOnce();
	}
	toPCLPointCloud2(*orig_cloud, gt_mesh->cloud);
	toPCLPointCloud2(*cut_cloud, cutmesh->cloud);

};

void nnf_visualization(XYZCloud::Ptr scene_cloud,
	pcl::PointCloud<NNFX<5>>::Ptr nnf5,
	pcl::PolygonMeshPtr scenemesh,
	pcl::PolygonMeshPtr cutmesh,
	XYZCloud::Ptr cut_cloud,
	vector<int> points) {
	vector<vector<vector<int>>> TtoQnnf(cut_cloud->size());
	for (int i = 0; i < cut_cloud->size(); i++) {
		TtoQnnf.at(i).resize(5);
	}
	for (int i = 0; i < nnf5->size(); i++) {
		for (int j = 0; j < 5; j++) {
			TtoQnnf.at(nnf5->at(i).labels[j]).at(j).push_back(i);
		}
	}

	Eigen::Vector4f centroid; Eigen::Vector4f farthest_point;
	pcl::compute3DCentroid(*scene_cloud, centroid);
	pcl::getMaxDistance(*scene_cloud, centroid, farthest_point);
	Eigen::Vector3f radi = (farthest_point - centroid).head<3>();
	XYZCloud::Ptr pcacloud(new XYZCloud()), cut2_cloud(new XYZCloud());
	copyPointCloud(*scene_cloud, *pcacloud);
	copyPointCloud(*cut_cloud, *cut2_cloud);

	pcl::PCA<XYZ> pca(*pcacloud);
	Eigen::Matrix3f xyz_coords = pca.getEigenVectors();
	Eigen::Vector3f ax(xyz_coords(0), xyz_coords(1), xyz_coords(2));
	Eigen::Vector3f ax1(xyz_coords(3), xyz_coords(4), xyz_coords(5));
	Eigen::Vector3f ax2(xyz_coords(6), xyz_coords(7), xyz_coords(8));
	string input = "";
	float c1 = 1 / sqrt(3);	float c2 = 1 / sqrt(3); float c3 = 1 / sqrt(3);

	pcl::compute3DCentroid(*cut_cloud, centroid);
	pcl::demeanPointCloud(*cut2_cloud, centroid, *cut2_cloud);
	pcl::demeanPointCloud(*pcacloud, centroid, *pcacloud);

	cout << "choose ratio of primary ax \n";
	string::size_type sz;
	std::getline(cin, input);
	if (check_num(input)) {
		c1 = stof(input, &sz);
	}

	cout << "choose ratio of secondary ax \n";
	std::getline(cin, input);
	if (check_num(input)) {
		c2 = stof(input, &sz);
	}

	cout << "choose ratio of minor ax \n";
	std::getline(cin, input);
	if (check_num(input)) {
		c3 = stof(input, &sz);
	}

	Eigen::Vector3f n_1 = c1*ax.normalized() + c2*ax1.normalized() + c3*ax2.normalized();
	n_1.normalize();
	Eigen::Vector4f n(n_1(0), n_1(1), n_1(2), 0);

	float multiplier = 2;
	cout << "choose Number of scene radiuses from Center \n";
	std::getline(cin, input);
	if (check_num(input)) {
		multiplier = stof(input, &sz);
	}

	Eigen::Vector4f ntrans = n * multiplier * radi.norm();
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	if (multiplier != 0) {
		transform_1(0, 3) = ntrans(0);
		transform_1(1, 3) = ntrans(1);
		transform_1(2, 3) = ntrans(2);
	}
	XYZCloud cloud;
	transformPointCloud(*cut2_cloud, *cut2_cloud, transform_1);
	XYZRGBCloud::Ptr shownCorrs(new XYZRGBCloud());
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("3D Viewer 1"));
	viewer2->setBackgroundColor(255, 255, 255);
	viewer2->initCameraParameters();
	toPCLPointCloud2(*pcacloud, scenemesh->cloud);
	toPCLPointCloud2(*cut2_cloud, cutmesh->cloud);
	viewer2->addPolygonMesh(*cutmesh, "temp1");
	viewer2->addPolygonMesh(*scenemesh, "scn1");
	float template_point_size = 10;
	float point_size_corrs = 10;
	int k = 0;
	bool alter = false;
	for (int i = 0; i < points.size(); i++) {
		pcl::PointXYZRGB  p_tgt;
		copyPoint(cut2_cloud->at(points.at(i)), p_tgt);
		shownCorrs->push_back(p_tgt);
		for (int j = 0; j < 5; j++) {
			for (int k = 0; k < TtoQnnf.at(points.at(i)).at(j).size(); k++) {
				pcl::PointXYZRGB  p_src;
				int r = 255;				int g = 0;    int b = 0;
				if (j == 1) { b = 255; };
				if (j == 2) { g = 255; };
				if (j == 3) { r = 0; b = 255; };
				if (j == 3) { r = 0; g = 255; };

				copyPoint(pcacloud->at(TtoQnnf.at(points.at(i)).at(j).at(k)), p_src);
				p_src.r = r;				p_src.g = g;    				p_src.b = b;

				string ss("line");
				ss = ss+ to_string(i*1000000+j*20000+k);
				string sss("spheresource");
				sss = sss + to_string(i * 1000000 + j * 20000 + k);
				string ssss("spheretarget");
				ssss = ssss + to_string(i * 1000000 + j * 20000 + k);
				shownCorrs->push_back(p_src);

				if (alter)
				{
					viewer2->addLine(p_src, p_tgt, r, g, b, ss);
					viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, ss);
				}
				else
				{
					viewer2->addLine(p_src, p_tgt, r, g, b, ss);
					viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, ss);
				}
				alter = !alter;
			}
		}
	}
	string corrs = "Corrs";
	viewer2->addPointCloud(shownCorrs, corrs);

	//savePointCloudToFile(shownCorrs, "snapshot.ply");
	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_corrs, corrs);
	viewer2->setWindowBorders(true);
	viewer2->resetCamera();
	while (!viewer2->wasStopped())
	{
		viewer2->spinOnce();
	}

};