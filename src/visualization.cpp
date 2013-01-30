#include <visualization.h>
#include <ros/ros.h>
#include <boost/thread/thread.hpp>

namespace vision_3d
{
	void visualizePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud)
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
		viewer->setBackgroundColor (0, 0, 0);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pclCloud);
		viewer->addPointCloud<pcl::PointXYZRGB> (pclCloud, rgb, "sample cloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud");
  		//viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (pclCloud, normals, 10, 0.05, "normals");
		viewer->addCoordinateSystem (1.0);
		viewer->initCameraParameters ();
		ROS_INFO("Viewer ready.");
		while (!viewer->wasStopped ())
		{
			viewer->spinOnce (100);
			boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		}
	}

	void visualizePointCloudNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
		viewer->setBackgroundColor (0, 0, 0);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pclCloud);
		viewer->addPointCloud<pcl::PointXYZRGB> (pclCloud, rgb, "sample cloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud");
		viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (pclCloud, normals, 10, 0.05, "normals");
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