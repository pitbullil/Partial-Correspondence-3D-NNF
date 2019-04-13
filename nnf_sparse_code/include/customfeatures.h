#define PCL_NO_PRECOMPILE
#ifndef CUSTOM_FEATS_H
#define CUSTOM_FEATS_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/feature.h>
#include <set>
#include <sfpfh.h>
#include <gfpfh.h>
#include <gpfh.h>
namespace pcl
{
	struct SFPFHSignature33;
	typedef SFPFHSignature33 SFPFH;
	typedef PointCloud<SFPFH> SFPFHCloud;

	struct GFPFHSignature33;
	typedef GFPFHSignature33 GFPFH;
	typedef PointCloud<GFPFH> GFPFHCloud;

	struct GPFHSignature125;
	typedef GPFHSignature125 GPFH;
	typedef PointCloud<GPFH> GPFHCloud;

}

PCL_EXPORTS std::ostream& operator << (std::ostream& os, const pcl::GPFHSignature125& p);
struct pcl::GPFHSignature125
{
	float histogram[125];
	static int descriptorSize() { return 125; }

	friend std::ostream& operator << (std::ostream& os, const pcl::GPFHSignature125& p);
};

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::GPFHSignature125,
(float[125], histogram, sfpfh)
);

	PCL_EXPORTS std::ostream& operator << (std::ostream& os, const pcl::SFPFHSignature33& p);
	struct pcl::SFPFHSignature33
	{
		float histogram[33];
		static int descriptorSize() { return 33; }

		friend std::ostream& operator << (std::ostream& os, const pcl::SFPFHSignature33& p);
	};
	
	POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::SFPFHSignature33,
		(float[33], histogram, sfpfh)
	);

	PCL_EXPORTS std::ostream& operator << (std::ostream& os, const pcl::GFPFHSignature33& p);
	struct pcl::GFPFHSignature33
	{
		float histogram[33];
		static int descriptorSize() { return 33; }

		friend std::ostream& operator << (std::ostream& os, const pcl::GFPFHSignature33& p);
	};

	POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::GFPFHSignature33,
		(float[33], histogram, gfpfh)
	);

#endif
