#ifndef KEYPOINTS_KEYPOINT_EXTRACTOR_H_
#define KEYPOINTS_KEYPOINT_EXTRACTOR_H_

#include <pcl17/point_types.h>
#include <pcl17/keypoints/keypoint.h>

class KeypointExtractor
{
	public:
		void extractFromFolder(char * inputFolder, char * outputFolder);
		void extractFromPCD(char * inputFile, char * outputFile);
};

#endif