#include <pcl17/io/pcd_io.h>
#include <pcl17/point_cloud.h>
#include <pcl17/correspondence.h>
#include <pcl17/features/normal_3d_omp.h>
#include <pcl17/features/shot_omp.h>
#include <pcl17/features/board.h>
#include <pcl17/keypoints/uniform_sampling.h>
#include <pcl17/recognition/cg/hough_3d.h>
#include <pcl17/recognition/cg/geometric_consistency.h>
#include <pcl17/visualization/pcl_visualizer.h>
#include <pcl17/kdtree/kdtree_flann.h>
#include <pcl17/kdtree/impl/kdtree_flann.hpp>
#include <pcl17/common/transforms.h>
#include <pcl17/console/parse.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

void openniCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  //Define necessary point clouds
  pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr model (new pcl17::PointCloud<pcl17::PointXYZRGB> ());
  pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr model_keypoints (new pcl17::PointCloud<pcl17::PointXYZRGB> ());
  pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr scene (new pcl17::PointCloud<pcl17::PointXYZRGB> ());
  pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr scene_keypoints (new pcl17::PointCloud<pcl17::PointXYZRGB> ());
  pcl17::PointCloud<pcl17::Normal>::Ptr model_normals (new pcl17::PointCloud<pcl17::Normal> ());
  pcl17::PointCloud<pcl17::Normal>::Ptr scene_normals (new pcl17::PointCloud<pcl17::Normal> ());
  pcl17::PointCloud<pcl17::SHOT352>::Ptr model_descriptors (new pcl17::PointCloud<pcl17::SHOT352> ());
  pcl17::PointCloud<pcl17::SHOT352>::Ptr scene_descriptors (new pcl17::PointCloud<pcl17::SHOT352> ());

  //Transform received point cloud to PCL type
  pcl17::fromROSMsg(*cloud, *model);

}

int
main(int argc, char ** argv)
{
  ros::Rate r(5);

  ros::init(argc, argv, "detection");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, openniCallback);

  while(ros::ok())
  {
  	ros::spinOnce();
  	r.sleep();
  }
  return 0;
}