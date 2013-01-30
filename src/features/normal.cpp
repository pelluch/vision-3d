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

	void vision_3d::NormalEstimator::estimateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
	{
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
		ne.setInputCloud (cloud);
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
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