#ifndef UTIL_UTIL_H_
#define UTIL_UTIL_H_

#include <pcl/point_types.h>

namespace vision_3d
{
	namespace util
	{
		void downSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);
	}
}

#endif
