#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <pcl/visualization/pcl_visualizer.h>

namespace vision_3d
{
	void visualizePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud);
	void visualizePointCloudNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud, pcl::PointCloud<pcl::Normal>::Ptr normals);
}

#endif