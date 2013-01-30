#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/voxel_grid.h>
#include <visualization.h>
#include <pcl/keypoints/sift_keypoint.h>

int numCallbacks;

//Should be moved to different file
void detectKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints)
{
  pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB> sift_detector;
  sift_detector.setInputCloud(cloud);
  sift_detector.setScales(0.2f, 3, 3);
  sift_detector.compute(*keypoints);
  //sift_detector.setInputCloud(*cloud);
  ROS_INFO("Keypoints detected");

}
void downSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
  pcl::VoxelGrid<pcl::PointXYZRGB> ds;
  ds.setInputCloud(cloud);
  ds.setLeafSize(0.005f, 0.005f, 0.005f);
  ds.filter(*output);
  ROS_INFO("Old cloud size: %d\n New cloud size: %d", cloud->width*cloud->height, output->width*output->height);
}

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
  ROS_INFO("Normals computed..");

}

void XYZRGBCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  ROS_INFO("Received XYZRGB point cloud");
  //Creating PCL format point clouds and normals
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr dsCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(*cloud, *pclCloud);
  detectKeypoints(pclCloud, keypoints);
  //Visualizing the cloud for debugging reasons.
  //visualizePointCloud(pclCloud);
  downSample(pclCloud, dsCloud);
  ROS_INFO("PCL downsampled");
  estimateNormals(dsCloud, normals);

  //Visualizing point cloud normals
  vision_3d::visualizePointCloudNormals(dsCloud, normals);
  numCallbacks++;


  ROS_INFO("Computing FPFHE");

  // Create the FPFH estimation class, and pass the input dataset+normals to it
  pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud (dsCloud);
  fpfh.setInputNormals (normals);

  // Create an empty kdtree representation, and pass it to the FPFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

  fpfh.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

  // Use all neighbors in a sphere of radius 5cm
  // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  fpfh.setRadiusSearch (0.1);

  // Compute the features
  fpfh.compute (*fpfhs);
  ROS_INFO("Finished");
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