#ifndef KEYPOINTS_SIFT_EXTRACTOR_H_
#define KEYPOINTS_SIFT_EXTRACTOR_H_

#include <keypoints/keypoint_extractor.h>
#include <pcl17/keypoints/sift_keypoint.h>

namespace vision_3d
{
	class SiftExtractor : public KeypointExtractor
	{
		public:
			SiftExtractor();
			void setScales(float min_scale, int nr_octaves, int nr_scales_per_octave);
			void extractFromFolder(char * inputFolder, char * outputFolder);
			void extractFromPCD(char * inputFile, char * outputFile);
			void detectKeypoints(pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr cloud, pcl17::PointCloud<pcl17::PointWithScale>::Ptr keypoints);
			~SiftExtractor();
		private:
			float min_scale_;
			int nr_octaves_;
			int nr_scales_per_octave_;
			pcl17::SIFTKeypoint<pcl17::PointXYZRGB, pcl17::PointWithScale> * extractor_;
	};
}
#endif