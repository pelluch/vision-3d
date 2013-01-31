#include <util/util.h>
#include <pcl17/filters/voxel_grid.h>

namespace vision_3d
{
	namespace util
	{
		void downSample(pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr cloud, pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr output)
		{
			 pcl17::VoxelGrid<pcl17::PointXYZRGB> ds;
			 ds.setInputCloud(cloud);
			 ds.setLeafSize(0.005f, 0.005f, 0.005f);
			 ds.filter(*output);
		}
	}
}