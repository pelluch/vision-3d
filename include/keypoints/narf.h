#ifndef KEYPOINTS_NARF_H_
#define KEYPOINTS_NARF_H_

#include <keypoints/keypoint_extractor.h>

#include <pcl17/range_image/range_image.h>
#include <pcl17/features/range_image_border_extractor.h>
#include <pcl17/keypoints/narf_keypoint.h>

namespace vision_3d
{
	class NarfExtractor : public KeypointExtractor
	{
		public:
			NarfExtractor();
			void extractFromFolder(char * inputFolder, char * outputFolder);
			void extractFromPCD(char * inputFile, char * outputFile);
			void extractNarfKeypoints(pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr cloud, pcl17::PointCloud<int>::Ptr indices );
	};
}
#endif