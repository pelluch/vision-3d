#include <visualization.h>
#include <ros/ros.h>
#include <boost/thread/thread.hpp>

namespace vision_3d
{
	void visualizePointCloud(pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr pclCloud)
	{
		boost::shared_ptr<pcl17::visualization::PCLVisualizer> viewer (new pcl17::visualization::PCLVisualizer ("3D Viewer"));
		viewer->setBackgroundColor (0, 0, 0);
		pcl17::visualization::PointCloudColorHandlerRGBField<pcl17::PointXYZRGB> rgb(pclCloud);
		viewer->addPointCloud<pcl17::PointXYZRGB> (pclCloud, rgb, "sample cloud");
		viewer->setPointCloudRenderingProperties (pcl17::visualization::PCL17_VISUALIZER_POINT_SIZE, 5, "sample cloud");
  		//viewer->addPointCloudNormals<pcl17::PointXYZRGB, pcl17::Normal> (pclCloud, normals, 10, 0.05, "normals");
		viewer->addCoordinateSystem (1.0);
		viewer->initCameraParameters ();
		ROS_INFO("Viewer ready.");
		while (!viewer->wasStopped ())
		{
			viewer->spinOnce (100);
			boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		}
	}

	void visualizePointCloudNormals(pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr pclCloud, pcl17::PointCloud<pcl17::Normal>::Ptr normals)
	{
		boost::shared_ptr<pcl17::visualization::PCLVisualizer> viewer (new pcl17::visualization::PCLVisualizer ("3D Viewer"));
		viewer->setBackgroundColor (0, 0, 0);
		pcl17::visualization::PointCloudColorHandlerRGBField<pcl17::PointXYZRGB> rgb(pclCloud);
		viewer->addPointCloud<pcl17::PointXYZRGB> (pclCloud, rgb, "sample cloud");
		viewer->setPointCloudRenderingProperties (pcl17::visualization::PCL17_VISUALIZER_POINT_SIZE, 5, "sample cloud");
		viewer->addPointCloudNormals<pcl17::PointXYZRGB, pcl17::Normal> (pclCloud, normals, 10, 0.05, "normals");
		viewer->addCoordinateSystem (1.0);
		viewer->initCameraParameters ();
		ROS_INFO("Viewer ready.");
		while (!viewer->wasStopped ())
		{
			viewer->spinOnce (100);
			boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		}
	}
}