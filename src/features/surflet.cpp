#include <features/surflet.h>

namespace vision_3d
{

	vision_3d::SurfletExtractor::SurfletExtractor()
	{

	}

	void vision_3d::SurfletExtractor::extractSurflets(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs)
	{
		pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
  		fpfh.setInputCloud (cloud);
  		fpfh.setInputNormals (normals);

  		// Create an empty kdtree representation, and pass it to the FPFH estimation object.
 		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  		fpfh.setSearchMethod (tree);

  		// Use all neighbors in a sphere of radius 5cm
		// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
		fpfh.setRadiusSearch (0.1);

		// Compute the features
		fpfh.compute (*fpfhs);
	}
}
