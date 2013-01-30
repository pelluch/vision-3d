#include <util/util.h>
#include <pcl/filters/voxel_grid.h>

namespace vision_3d
{
	namespace util
	{
		void downSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
		{
			 pcl::VoxelGrid<pcl::PointXYZRGB> ds;
			 ds.setInputCloud(cloud);
			 ds.setLeafSize(0.005f, 0.005f, 0.005f);
			 ds.filter(*output);
		}
	}
}