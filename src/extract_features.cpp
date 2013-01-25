#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
//pcl::visualization::CloudViewer * viewer;
#include <boost/thread/thread.hpp>

void callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  


 pcl::PointCloud<pcl::PointXYZRGB> cloud(input->width, input->height);
 pcl::fromROSMsg (*input, cloud);
 //printf ("Cloud: width = %d, height = %d\n", cloud.width, cloud.height);
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr = cloud.makeShared();
 pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
 ne.setInputCloud (cloudPtr);

// Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudPtr);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloudPtr, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloudPtr, cloud_normals, 100, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  while (!viewer->wasStopped ())
  {
	  viewer->spinOnce(100);
	  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "extract_features");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 100, callback);
  //viewer = new pcl::visualization::CloudViewer("Cloud");

  ros::spin();
  //delete viewer;
}