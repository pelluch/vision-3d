#ifndef UTIL_UTIL_H_
#define UTIL_UTIL_H_

#include <pcl17/point_types.h>

namespace vision_3d
{
	namespace util
	{
		void downSample(pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr cloud, pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr output);
	}
}

#endif
