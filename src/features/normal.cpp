#include <features/normal.h>

namespace vision_3d
{
	vision_3d::NormalEstimator::NormalEstimator()
	{
		isRadiusSearch_ = true;
		radius_ = 0.03f;
		knn_ = 0;
		isNeighbourSearch_ = false;
	}

	void vision_3d::NormalEstimator::estimateNormals(pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr cloud, pcl17::PointCloud<pcl17::Normal>::Ptr normals)
	{
		pcl17::NormalEstimation<pcl17::PointXYZRGB, pcl17::Normal> ne;
		ne.setInputCloud (cloud);
		pcl17::search::KdTree<pcl17::PointXYZRGB>::Ptr tree (new pcl17::search::KdTree<pcl17::PointXYZRGB> ());
	  	ne.setSearchMethod (tree);
	  	ne.compute(*normals);
	}

	void vision_3d::NormalEstimator::setRadiusSearch(float radius)
	{
		isRadiusSearch_ = true;
		isNeighbourSearch_ = false;
		radius_ = radius;
	}

	void vision_3d::NormalEstimator::setNeighbourSearch(int knn)
	{
		isRadiusSearch_ = false;
		isNeighbourSearch_ = true;
		knn_ = knn;
	}
}