#include <features/surflet.h>

namespace vision_3d
{

	vision_3d::SurfletExtractor::SurfletExtractor()
	{

	}

	void vision_3d::SurfletExtractor::extractSurflets(pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr cloud, pcl17::PointCloud<pcl17::Normal>::Ptr normals, pcl17::PointCloud<pcl17::FPFHSignature33>::Ptr fpfhs)
	{
		pcl17::FPFHEstimation<pcl17::PointXYZRGB, pcl17::Normal, pcl17::FPFHSignature33> fpfh;
  		fpfh.setInputCloud (cloud);
  		fpfh.setInputNormals (normals);

  		// Create an empty kdtree representation, and pass it to the FPFH estimation object.
 		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  		pcl17::search::KdTree<pcl17::PointXYZRGB>::Ptr tree (new pcl17::search::KdTree<pcl17::PointXYZRGB>);
  		fpfh.setSearchMethod (tree);

  		// Use all neighbors in a sphere of radius 5cm
		// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
		fpfh.setRadiusSearch (0.1);

		// Compute the features
		fpfh.compute (*fpfhs);
	}
}
