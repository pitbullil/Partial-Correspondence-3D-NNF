#ifndef QUICKSORT_H
#define QUICKSORT_H

#include <string>
#include <iostream>
#include <fstream>
#include <algorithm> 
#include <direct.h>
#include <omp.h> 
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/pca.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/conversions.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/geometry.h>

using namespace std;
using namespace pcl;

struct curvaturesortnode {
	float curvature;
	int32_t origin;
	bool operator > (const curvaturesortnode& str) const
	{
		return (curvature > str.curvature);
	}

};

vector<curvaturesortnode> create_curvature_array(PointCloud<Normal>::Ptr NormalCloud);

 void sortCloudsbyCurvature(PointCloud<Normal>::Ptr NormalCloud, PointCloud<PointXYZ>::Ptr SrcCloud, PointCloud<PointXYZ>::Ptr Tcloud);

#endif