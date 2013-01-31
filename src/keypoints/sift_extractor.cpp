#include <keypoints/sift_extractor.h>

namespace vision_3d
{
	SiftExtractor::SiftExtractor()
	{
		extractor_ = new pcl17::SIFTKeypoint<pcl17::PointXYZRGB, pcl17::PointWithScale>();
	}

	void 
	SiftExtractor::extractFromFolder(char * inputFolder, char * outputFolder)
	{

	}
	void 
	SiftExtractor::extractFromPCD(char * inputFile, char * outputFile)
	{

	}

	//Should be moved to different file
	void
	SiftExtractor::detectKeypoints(pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr cloud, pcl17::PointCloud<pcl17::PointWithScale>::Ptr keypoints)
	{
	  extractor_->setInputCloud(cloud);
	  extractor_->setScales(min_scale_, nr_octaves_, nr_scales_per_octave_);
	  extractor_->compute(*keypoints);
	}

	void
	SiftExtractor::setScales(float min_scale, int nr_octaves, int nr_scales_per_octave)
	{
		min_scale_ = min_scale;
		nr_octaves_ = nr_octaves;
		nr_scales_per_octave_ = nr_scales_per_octave;
	}

	SiftExtractor::~SiftExtractor()
	{
		delete extractor_;
	}
}