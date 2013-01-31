#include <keypoints/narf.h>
#include <pcl17/io/pcd_io.h>
#include <iostream>

int main(int argc, char ** argv)
{
	pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr point_cloud (new pcl17::PointCloud<pcl17::PointXYZRGB>);
	pcl17::PointCloud<int>::Ptr indices (new pcl17::PointCloud<int>());

	std::string filename = argv[1];
	if (pcl17::io::loadPCDFile (filename, *point_cloud) == -1)
    {
      std::cout << "Was not able to open file \""<<filename<<"\".\n";      
    }
    else
    {
    	vision_3d::NarfExtractor extractor;
    	extractor.extractNarfKeypoints(point_cloud, indices);
    }
    
    return 0;
}