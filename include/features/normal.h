#ifndef FEATURES_NORMAL_H_
#define FEATURES_NORMAL_H_

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

namespace vision_3d
{
	class NormalEstimator
	{
		public:
			NormalEstimator();
			void estimateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);
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