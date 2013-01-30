#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/voxel_grid.h>
#include <visualization.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <features/normal.h>
#include <features/surflet.h>
#include <util/util.h>

int numCallbacks;



void XYZRGBCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  ROS_INFO("Received XYZRGB point cloud");
  //Creating PCL format point clouds and normals
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr dsCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(*cloud, *pclCloud);
  //detectKeypoints(pclCloud, keypoints);
  //Visualizing the cloud for debugging reasons.
  //visualizePointCloud(pclCloud);
  vision_3d::util::downSample(pclCloud, dsCloud);
  ROS_INFO("PCL downsampled");

  vision_3d::NormalEstimator estimator;
  estimator.estimateNormals(dsCloud, normals);

  //Visualizing point cloud normals
  vision_3d::visualizePointCloudNormals(dsCloud, normals);
  numCallbacks++;

  ROS_INFO("Computing FPFHE");

  // Output datasets
  vision_3d::SurfletExtractor surflets;
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
  surflets.extractSurflets(dsCloud, normals, fpfhs);

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