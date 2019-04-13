#define PCL_NO_PRECOMPILE
#ifndef NNF5_H
#define NNF5_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
namespace dis3 {
	template<int X> struct NNFX;
	template<int X> struct HistogramInt;
}

PCL_EXPORTS std::ostream& operator << (std::ostream& os, const dis3::NNFX<5>& p);
PCL_EXPORTS std::ostream& operator << (std::ostream& os, const dis3::NNFX<1>& p);
PCL_EXPORTS std::ostream& operator << (std::ostream& os, const dis3::HistogramInt<5>& p);

template<int X> struct dis3::NNFX
{
	int labels[X];
	float distance[X];

	static int labelSize() { return X; }

	friend std::ostream& operator << (std::ostream& os, const NNFX<X>& p);
};

template<int X> struct dis3::HistogramInt
{
	int counter[X];
	static int labelSize() { return X; }

	friend std::ostream& operator << (std::ostream& os, const HistogramInt<X>& p);
};


POINT_CLOUD_REGISTER_POINT_STRUCT(dis3::NNFX<5>,
	(int[5], labels, labels)
	(float[5], distance, distance)
);

POINT_CLOUD_REGISTER_POINT_STRUCT(dis3::NNFX<1>,
(int[1], labels, labels)
(float[1], distance, distance)
);

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::Histogram<5>,
(float[5], histogram, distance)
);

POINT_CLOUD_REGISTER_POINT_STRUCT(dis3::HistogramInt<5>,
(int[5], counter, counter)
);


#endif
