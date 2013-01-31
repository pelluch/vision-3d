#ifndef FEATURES_SURFLET_H_
#define FEATURES_SURFLET_H_

#include <features/normal.h>
#include <pcl17/features/fpfh.h>
#include <pcl17/point_types.h>

namespace vision_3d
{
	class SurfletExtractor
	{
		public:
			SurfletExtractor();
			void extractSurflets(pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr cloud, pcl17::PointCloud<pcl17::Normal>::Ptr normals, pcl17::PointCloud<pcl17::FPFHSignature33>::Ptr fpfhs);
	};
}

#endif