#ifndef FEATURES_NORMAL_H_
#define FEATURES_NORMAL_H_

#include <pcl17/point_types.h>
#include <pcl17/features/normal_3d.h>

namespace vision_3d
{
	class NormalEstimator
	{
		public:
			NormalEstimator();
			void estimateNormals(pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr cloud, pcl17::PointCloud<pcl17::Normal>::Ptr normals);
			void setRadiusSearch(float radius);
			void setNeighbourSearch(int knn);
		private:
			float radius_;
			int knn_;
			bool isRadiusSearch_;
			bool isNeighbourSearch_;
	};
}

#endif