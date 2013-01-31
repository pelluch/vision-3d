#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl17/point_types.h>
#include <pcl17/features/fpfh.h>
#include <pcl_ros/point_cloud.h>
#include <pcl17/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl17/filters/voxel_grid.h>
#include <visualization.h>
#include <pcl17/keypoints/sift_keypoint.h>
#include <features/normal.h>
#include <features/surflet.h>
#include <util/util.h>

int numCallbacks;



void XYZRGBCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  ROS_INFO("Received XYZRGB point cloud");
  //Creating pcl17 format point clouds and normals
  pcl17::PointCloud<pcl17::Normal>::Ptr normals (new pcl17::PointCloud<pcl17::Normal> ());
  pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr pcl17Cloud(new pcl17::PointCloud<pcl17::PointXYZRGB>());
  pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr dsCloud(new pcl17::PointCloud<pcl17::PointXYZRGB>());
  pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr keypoints(new pcl17::PointCloud<pcl17::PointXYZRGB>());
  pcl17::fromROSMsg(*cloud, *pcl17Cloud);
  //detectKeypoints(pcl17Cloud, keypoints);
  //Visualizing the cloud for debugging reasons.
  //visualizePointCloud(pcl17Cloud);
  vision_3d::util::downSample(pcl17Cloud, dsCloud);
  ROS_INFO("pcl17 downsampled");

  vision_3d::NormalEstimator estimator;
  estimator.estimateNormals(dsCloud, normals);

  //Visualizing point cloud normals
  vision_3d::visualizePointCloudNormals(dsCloud, normals);
  numCallbacks++;

  ROS_INFO("Computing FPFHE");

  // Output datasets
  vision_3d::SurfletExtractor surflets;
  pcl17::PointCloud<pcl17::FPFHSignature33>::Ptr fpfhs (new pcl17::PointCloud<pcl17::FPFHSignature33> ());
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