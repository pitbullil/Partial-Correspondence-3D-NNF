#include <pcl/point_types.h>
#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>


template<class PointT> pcl::PointCloud<PointT> downsample(pcl::PointCloud<PointT> cloud, float resultion_out) {
	pcl::PointCloud<PointT>::Ptr sampled_cloud(new pcl::PointCloud<PointT>);
	  pcl::VoxelGrid<pcl::PCLPointCloud<PointT>> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (resultion_out, resultion_out, resultion_out);
	sor.filter (*sampled_cloud);

	return *sampled_cloud;
};