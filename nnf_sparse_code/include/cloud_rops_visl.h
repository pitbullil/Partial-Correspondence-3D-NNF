
#define PCL_NO_PRECOMPILE
#ifndef ROPS_H
#define ROPS_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
namespace dis3 {
	struct ROPS;
}

PCL_EXPORTS std::ostream& operator << (std::ostream& os, const dis3::ROPS& p);

struct dis3::ROPS
{
	float descriptor[135];
	static int descriptorSize() { return 135; }

	friend std::ostream& operator << (std::ostream& os, const ROPS& p);
};

POINT_CLOUD_REGISTER_POINT_STRUCT(dis3::ROPS,
(float[135], descriptor, rops)
);

#endif
