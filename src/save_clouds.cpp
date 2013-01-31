#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/point_types.h>
#include <iostream>

int counter = 0;

void 
PointXYZRGBtoXYZI (pcl17::PointXYZRGB&  in,
                 pcl17::PointXYZI&    out)
{
	out.x = in.x; out.y = in.y; out.z = in.z;
	out.intensity = 0.299f * in.r + 0.587f * in.g + 0.114f * in.b;
}

void
  PointCloudXYZRGBtoXYZI (pcl17::PointCloud<pcl17::PointXYZRGB>& in,
                          pcl17::PointCloud<pcl17::PointXYZI>& out)
  {
    out.width   = in.width;
    out.height  = in.height;
    for (size_t i = 0; i < in.points.size (); i++)
    {
      pcl17::PointXYZI p;
      PointXYZRGBtoXYZI (in.points[i], p);
      out.points.push_back (p);
    }
  }

void openniCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
	pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr scene (new pcl17::PointCloud<pcl17::PointXYZRGB> ());
	pcl17::fromROSMsg(*cloud, *scene);
	pcl17::PointCloud<pcl17::PointXYZI>::Ptr intensity (new pcl17::PointCloud<pcl17::PointXYZI> ());
	PointCloudXYZRGBtoXYZI(*scene, *intensity);
	std::stringstream ss;
	ss << "/home/pablo/Desktop/pcd/" << counter << "_XYZRGB.pcd";
	pcl17::io::savePCDFileASCII (ss.str(), *scene);
	ss.str("");
	ss << "/home/pablo/Desktop/pcd/" << counter << "_XYZI.pcd";
	std::cout << "Attempting to save intensity at " << ss.str() << std::endl;
	pcl17::io::savePCDFileASCII (ss.str(), *intensity);
	counter = counter + 1;
}

int
main(int argc, char ** argv)
{
	ros::init(argc, argv, "detection");
	ros::NodeHandle nh;
	ros::Rate r(1);	
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, openniCallback);

	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}