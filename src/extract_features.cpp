#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

int numCallbacks;

void estimateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
		// Create the normal estimation class, and pass the input dataset to it
	  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	  ne.setInputCloud (cloud);
    ROS_INFO("Set input cloud in normal estimator");
	  // Create an empty kdtree representation, and pass it to the normal estimation object.
	  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	  ne.setSearchMethod (tree);

	  // Use all neighbors in a sphere of radius 3cm
  	ne.setRadiusSearch (0.03);
    ROS_INFO("Computing normals...");
    ne.compute(*normals);

}

void XYZRGBCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
	   ROS_INFO("Received XYZRGB point cloud");
	   //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  	 pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
     pcl::fromROSMsg(*cloud, *pclCloud);
     estimateNormals(pclCloud, normals);
     //std::stringstream fileName;
     //fileName << "/home/pablo/Desktop/PCD/" << numCallbacks << ".pcd";
     //pcl::io::savePCDFile(fileName.str(), *normals);
     numCallbacks++;
     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
     viewer->setBackgroundColor (0, 0, 0);
     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pclCloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (pclCloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
   viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (pclCloud, normals, 10, 0.05, "normals");
    viewer->addCoordinateSystem (1.0);
   viewer->initCameraParameters ();
  	//  // Create the FPFH estimation class, and pass the input dataset+normals to it
  	 pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
 	   fpfh.setInputCloud (pclCloud);
  	 fpfh.setInputNormals (normals);

  	// // Create an empty kdtree representation, and pass it to the FPFH estimation object.
 	 //  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

  	 fpfh.setSearchMethod (tree);

  	// // Output datasets
  	 pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

  	// // Use all neighbors in a sphere of radius 5cm
  	// // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  	 fpfh.setRadiusSearch (0.08);

  	// // Compute the features
  	 fpfh.compute (*fpfhs);
     while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "extract_features");
	ros::NodeHandle nh;
  numCallbacks = 0;
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 100, XYZRGBCallback);
  ros::spin();
	return 0;
}