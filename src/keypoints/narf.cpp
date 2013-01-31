#include <keypoints/narf.h>
#include <pcl17/visualization/range_image_visualizer.h>
#include <boost/thread/thread.hpp>

namespace vision_3d
{
	NarfExtractor::NarfExtractor()
	{

	}

	void
	NarfExtractor::extractFromFolder(char * inputFolder, char * outputFolder)
	{

	}

	void 
	NarfExtractor::extractFromPCD(char * inputFile, char * outputFile)
	{

	}

	void
	NarfExtractor::extractNarfKeypoints(pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr cloud_ptr, pcl17::PointCloud<int>::Ptr indices )
	{

		pcl17::RangeImage::CoordinateFrame coordinate_frame = pcl17::RangeImage::CAMERA_FRAME;
		pcl17::PointCloud<pcl17::PointXYZRGB>& cloud = *cloud_ptr;
		//Transform point cloud to Range image
		Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
		scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (cloud.sensor_origin_[0],
			cloud.sensor_origin_[1],
			cloud.sensor_origin_[2])) *
		Eigen::Affine3f (cloud.sensor_orientation_);

		float angular_resolution = 0.5f;
		float noise_level = 0.0;
		float min_range = 0.0f;
		int border_size = 1;
		float support_size = 0.2f;

		boost::shared_ptr<pcl17::RangeImage> range_image_ptr (new pcl17::RangeImage);
		pcl17::RangeImage& range_image = *range_image_ptr;   
		range_image.createFromPointCloud (cloud, angular_resolution, pcl17::deg2rad (360.0f), pcl17::deg2rad (180.0f),
			scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
		//range_image.integrateFarRanges (far_ranges);


		pcl17::RangeImageBorderExtractor range_image_border_extractor;
		pcl17::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor);
		narf_keypoint_detector.setRangeImage (&range_image);
		narf_keypoint_detector.getParameters ().support_size = support_size;
 		//narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
 		//narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;

		narf_keypoint_detector.compute (*indices);
		pcl17::visualization::RangeImageVisualizer range_image_widget ("Range image");
  		range_image_widget.showRangeImage (range_image);
	}
}