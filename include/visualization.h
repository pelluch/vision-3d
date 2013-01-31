#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <pcl17/visualization/pcl_visualizer.h>

namespace vision_3d
{
	void visualizePointCloud(pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr pcl17Cloud);
	void visualizePointCloudNormals(pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr pcl17Cloud, pcl17::PointCloud<pcl17::Normal>::Ptr normals);
}

#endif