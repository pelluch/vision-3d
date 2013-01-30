#ifndef FEATURES_SURFLET_H_
#define FEATURES_SURFLET_H_

#include <features/normal.h>
#include <pcl/features/fpfh.h>
#include <pcl/point_types.h>

namespace vision_3d
{
	class SurfletExtractor
	{
		public:
			SurfletExtractor();
			void extractSurflets(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs);
	};
}

#endif