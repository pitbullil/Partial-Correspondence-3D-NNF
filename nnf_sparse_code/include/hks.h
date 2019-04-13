#define PCL_NO_PRECOMPILE
#ifndef HKS_H
#define HKS_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
namespace dis3 {
	struct HKS;
	struct SIHKS;
	struct SCHKS;
	struct LBEIGS;


}

PCL_EXPORTS std::ostream& operator << (std::ostream& os, const dis3::HKS& p);
PCL_EXPORTS std::ostream& operator << (std::ostream& os, const dis3::SIHKS& p);
PCL_EXPORTS std::ostream& operator << (std::ostream& os, const dis3::SCHKS& p);
PCL_EXPORTS std::ostream& operator << (std::ostream& os, const dis3::LBEIGS& p);

struct dis3::HKS
{
	float descriptor[23];
	static int descriptorSize() { return 23; }

	friend std::ostream& operator << (std::ostream& os, const HKS& p);
};


struct dis3::SIHKS
{
	float descriptor[19];
	static int descriptorSize() { return 19; }

	friend std::ostream& operator << (std::ostream& os, const SIHKS& p);
};

struct dis3::SCHKS
{
	float descriptor[23];
	static int descriptorSize() { return 23; }
	friend std::ostream& operator << (std::ostream& os, const SCHKS& p);
};

struct dis3::LBEIGS
{
	float descriptor[90];
	static int descriptorSize() { return 90; }
	friend std::ostream& operator << (std::ostream& os, const LBEIGS& p);
};


POINT_CLOUD_REGISTER_POINT_STRUCT(dis3::HKS,
(float[23], descriptor, hks)
);

POINT_CLOUD_REGISTER_POINT_STRUCT(dis3::SIHKS,
(float[19], descriptor, sihks)
);

POINT_CLOUD_REGISTER_POINT_STRUCT(dis3::SCHKS,
(float[23], descriptor, schks)
);

POINT_CLOUD_REGISTER_POINT_STRUCT(dis3::LBEIGS,
(float[90], descriptor, lbeigs)
);



#endif
