#ifndef VOL_IOU_H
#define VOL_IOU_H
#include <cmath>
#include <algorithm>
#include <pcl/filters/crop_hull.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/centroid.h>
#include <pcl/common/impl/common.hpp>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

typedef PointXYZ PointT;
typedef PointCloud<PointT> CloudT;

const static double default_alpha = 1e3f;

static void
saveCloud(std::string const& filename, CloudT const& cloud);

static CloudT::Ptr
calculateHull(std::vector<pcl::Vertices>& polygons, int& dim, double& volume, CloudT::Ptr cloud, double alpha = default_alpha);

static void
cropToHull(CloudT::Ptr output, CloudT::Ptr input, CloudT::Ptr hull_cloud, std::vector<pcl::Vertices> const& polygons, double& volume, int dim);

double ConvexIoU(CloudT::Ptr& gt_cloud, CloudT::Ptr& result_cloud, double& gtV, double& resultV, double& IntersectionV);

double SphereIoU(CloudT::Ptr& gt_cloud, CloudT::Ptr& result_cloud, double& V_GT, double& V_Result, double& V_Intersection, double& R_Result, double& RGT, double& d);

const float PI(3.14159265359);

#endif