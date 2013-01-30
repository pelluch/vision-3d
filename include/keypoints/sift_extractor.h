#ifndef KEYPOINTS_SIFT_EXTRACTOR_H_
#define KEYPOINTS_SIFT_EXTRACTOR_H_

#include <keypoints/keypoint_extractor.h>
#include <pcl/keypoints/sift_keypoint.h>

namespace vision_3d
{
	class SiftExtractor : public KeypointExtractor
	{
		public:
			SiftExtractor();
			void setScales(float min_scale, int nr_octaves, int nr_scales_per_octave);
			void extractFromFolder(char * inputFolder, char * outputFolder);
			void extractFromPCD(char * inputFile, char * outputFile);
			void detectKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints);
			~SiftExtractor();
		private:
			float min_scale_;
			int nr_octaves_;
			int nr_scales_per_octave_;
			pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> * extractor_;
	};
}
#endif