/*
	Author: Hiu Hong Yu
	Orginization: UC Davis AHMCT

	This module contains modified and structured stereo camera calibration source code from
	the below source.

	Citation:
	Learning OpenCV: Computer Vision with the OpenCV Library
	by Gary Bradski and Adrian Kaehler
	Published by O'Reilly Media, October 3, 2008
	Page 446 - 452
	Chapter 12 - Stereo Calibration, Rectification, and Correspondence Code

	OpenCV 2.4.13 - opencv-2.4.13/samples/cpp/stereo_match.cpp


*/
#include "stereo_camera_point_cloud.h"
// #include "stereo_camera_point_cloud.tpp"

/*
	Constructor/Destructor
*/
//constructor
StereoCameraPointCloud::StereoCameraPointCloud(StereoCameraCalibrationParameters* scp, bool calibrate)	{
	sccp = scp;
	scd = new StereoCameraDisparity(sccp, calibrate);
}

// destructor
StereoCameraPointCloud::~StereoCameraPointCloud()	{
	pixelToWorld.clear();
	delete scd;
}

/*
	Getter/Setter
*/
std::map<cv::Point2d, pcl::PointXYZRGB, PointComp> StereoCameraPointCloud::getPixelToWorldMap()	{
	return pixelToWorld;
}

pcl::PointXYZRGB StereoCameraPointCloud::getPoints(int x, int y)	{
	cv::Point2d coordinate(x, y);
	if(pixelToWorld.find(coordinate) != pixelToWorld.end())	{
		return pixelToWorld[coordinate];
	}
	return pcl::PointXYZRGB();
}

pcl::PointXYZRGB StereoCameraPointCloud::getPoints(double x, double y)	{
	// cv::Point2d coordinate(x, y);
	// if(pixelToWorld.find(coordinate) != pixelToWorld.end())	{
	// 	return pixelToWorld[coordinate];
	// }
	return pcl::PointXYZRGB();
}

pcl::PointXYZRGB StereoCameraPointCloud::getPoints(cv::Point2d coordinate)	{
	if(pixelToWorld.find(coordinate) != pixelToWorld.end())	{
		return pixelToWorld[coordinate];
	}
	return pcl::PointXYZRGB();
}

/*
	Non-Template Class Methods
*/
bool StereoCameraPointCloud::init(cv::Size imageSize, StereoCameraParameters* bmp)	{
	return scd->disparityInit(imageSize, bmp);
}

// World Coordinate Point Cloud Generator
void StereoCameraPointCloud::generateWorldPCLPointCloud(cv::Mat image1, cv::Mat image2, double minZ, double maxZ, int disparityType,
														cv::Mat& filteredImage1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc,
														REPROJECTION_MODE rMode)	{
	if(disparityType != CV_8U && disparityType != CV_8UC1 && 
	   disparityType != CV_16U && disparityType != CV_16UC1)	{
		std::cerr << "Only CV_8U and CV_16U supported\n";
		return;
	}

	cv::Size imageSize = image1.size();

	// Computed Disparity Map
	if(scd->computeDisparityMap(image1, image2, imageSize, TRIPLE))	{	
		// Get Computed Disparity Map in 8bit unsigned format
		cv::Mat disparityMap = scd->getNormDisparityMap(disparityType);
		// Reproject Point Cloud according to the type
		if(disparityType == CV_8U || disparityType == CV_8UC1)	{
			// Reproject Disparity Map in 3D world coordinate
			filteredImage1 = reprojectionWorld<uint8_t>(image1, disparityMap, pc, minZ, maxZ, STANDARD, rMode);
		}
		else if(disparityType == CV_16U || disparityType == CV_16UC1)	{
			// Reproject Disparity Map in 3D world coordinate
			filteredImage1 = reprojectionWorld<uint16_t>(image1, disparityMap, pc, minZ, maxZ, STANDARD, rMode);
		}
	}
}

void StereoCameraPointCloud::generateWorldPCLPointCloud(cv::Mat image1, cv::Mat image2, double minZ, double maxZ,
														int disparityType,
														pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc,
														REPROJECTION_MODE rMode)	{
	if(disparityType != CV_8U && disparityType != CV_8UC1 && 
	   disparityType != CV_16U && disparityType != CV_16UC1)	{
		std::cerr << "Only CV_8U and CV_16U supported\n";
		return;
	}

	cv::Size imageSize = image1.size();

	// Computed Disparity Map
	if(scd->computeDisparityMap(image1, image2, imageSize, TRIPLE))	{	
		// Get Computed Disparity Map in 8bit unsigned format
		cv::Mat disparityMap = scd->getNormDisparityMap(disparityType);
		// Reproject Point Cloud according to the type
		if(disparityType == CV_8U || disparityType == CV_8UC1)	{
			// Reproject Disparity Map in 3D world coordinate
			reprojectionWorld<uint8_t>(image1, disparityMap, pc, minZ, maxZ, STANDARD);
		}
		else if(disparityType == CV_16U || disparityType == CV_16UC1)	{
			// Reproject Disparity Map in 3D world coordinate
			reprojectionWorld<uint16_t>(image1, disparityMap, pc, minZ, maxZ, STANDARD);
		}
	}
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr StereoCameraPointCloud::generateWorldPCLPointCloud(cv::Mat image1, cv::Mat image2, 
																						  double minZ, double maxZ, 
																						  int disparityType,
																						  REPROJECTION_MODE rMode)	{
	if(disparityType != CV_8U && disparityType != CV_8UC1 && 
	   disparityType != CV_16U && disparityType != CV_16UC1)	{
		std::cerr << "Only CV_8U and CV_16U supported\n";
		return NULL;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr stereoPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cv::Size imageSize = image1.size();

	// Computed Disparity Map
	if(scd->computeDisparityMap(image1, image2, imageSize, TRIPLE))	{	
		// Get Computed Disparity Map in 8bit unsigned format
		cv::Mat disparityMap = scd->getNormDisparityMap(disparityType);
		// Reproject Point Cloud according to the type
		if(disparityType == CV_8U || disparityType == CV_8UC1)	{
			// Reproject Disparity Map in 3D world coordinate
			reprojectionWorld<uint8_t>(image1, disparityMap, stereoPointCloud, minZ, maxZ, STANDARD);
		}
		else if(disparityType == CV_16U || disparityType == CV_16UC1)	{
			// Reproject Disparity Map in 3D world coordinate
			reprojectionWorld<uint16_t>(image1, disparityMap, stereoPointCloud, minZ, maxZ, STANDARD);
		}
	}
	
	std::cout <<stereoPointCloud->size() << "\n";
	return stereoPointCloud;
}


// Pixel Coordinate Point Cloud Generator
void StereoCameraPointCloud::generateDepthPCLPointCloud(cv::Mat image1, cv::Mat image2, double minZ, double maxZ, int disparityType,
														cv::Mat& filteredImage1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc,
														REPROJECTION_MODE rMode)	{
	if(disparityType != CV_8U && disparityType != CV_8UC1 && 
	   disparityType != CV_16U && disparityType != CV_16UC1)	{
		std::cerr << "Only CV_8U and CV_16U supported\n";
		return;
	}

	cv::Size imageSize = image1.size();

	// Computed Disparity Map
	if(scd->computeDisparityMap(image1, image2, imageSize, TRIPLE))	{	
		// Get Computed Disparity Map in 8bit unsigned format
		cv::Mat disparityMap = scd->getNormDisparityMap(disparityType);
		// Reproject Point Cloud according to the type
		if(disparityType == CV_8U || disparityType == CV_8UC1)	{
			// Reproject Disparity Map in 3D world coordinate
			filteredImage1 = reprojectionDepth<uint8_t>(image1, disparityMap, pc,minZ, maxZ, rMode);
		}
		else if(disparityType == CV_16U || disparityType == CV_16UC1)	{
			// Reproject Disparity Map in 3D world coordinate
			filteredImage1 = reprojectionDepth<uint16_t>(image1, disparityMap, pc, minZ, maxZ, rMode);
		}
	}
}

void StereoCameraPointCloud::generateDepthPCLPointCloud(cv::Mat image1, cv::Mat image2, double minZ, double maxZ, 
														int disparityType,
														pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc,
														REPROJECTION_MODE rMode)	{
	if(disparityType != CV_8U && disparityType != CV_8UC1 && 
	   disparityType != CV_16U && disparityType != CV_16UC1)	{
		std::cerr << "Only CV_8U and CV_16U supported\n";
		return;
	}

	cv::Size imageSize = image1.size();

	if(scd->computeDisparityMap(image1, image2, imageSize, TRIPLE))	{	
		// Get Computed Disparity Map in 8bit unsigned format
		cv::Mat disparityMap = scd->getNormDisparityMap(disparityType);
		// Reproject Disparity Map in Pixel coordinate
		if(disparityType == CV_8U || disparityType == CV_8UC1)	{
			// Reproject Disparity Map in 3D world coordinate
			if(rMode == RMODE_2D)	{
				// reprojectionDepth2D<uint8_t>(image1, disparityMap, pc, maxZ);
			}
			else	{
				reprojectionDepth<uint8_t>(image1, disparityMap, pc, minZ, maxZ);
			}
		}
		else if(disparityType == CV_16U || disparityType == CV_16UC1)	{
			// Reproject Disparity Map in 3D world coordinate
			if(rMode == RMODE_2D)	{
				// reprojectionDepth2D<uint16_t>(image1, disparityMap, pc, maxZ);
			}
			else	{
				reprojectionDepth<uint16_t>(image1, disparityMap, pc, minZ, maxZ);
			}
			
		}
	}
}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr StereoCameraPointCloud::generateDepthPCLPointCloud(cv::Mat image1, cv::Mat image2, 
																						  double minZ, double maxZ, 
																						  int disparityType,
																						  REPROJECTION_MODE rMode)	{
	if(disparityType != CV_8U && disparityType != CV_8UC1 && 
	   disparityType != CV_16U && disparityType != CV_16UC1)	{
		std::cerr << "Only CV_8U and CV_16U supported\n";
		return NULL;
	}

	cv::Size imageSize = image1.size();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr stereoPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	// Computed Disparity Map
	if(scd->computeDisparityMap(image1, image2, imageSize, TRIPLE))	{	
		// Get Computed Disparity Map in 8bit unsigned format
		cv::Mat disparityMap = scd->getNormDisparityMap(disparityType);
		// Reproject Disparity Map in Pixel coordinate
		if(disparityType == CV_8U || disparityType == CV_8UC1)	{
			// Reproject Disparity Map in 3D world coordinate
			if(rMode == RMODE_2D)	{
				// reprojectionDepth2D<uint8_t>(image1, disparityMap, stereoPointCloud, maxZ);
			}
			else	{
				reprojectionDepth<uint8_t>(image1, disparityMap, stereoPointCloud, minZ, maxZ);
			}
		}
		else if(disparityType == CV_16U || disparityType == CV_16UC1)	{
			// Reproject Disparity Map in 3D world coordinate
			if(rMode == RMODE_2D)	{
				// reprojectionDepth2D<uint16_t>(image1, disparityMap, stereoPointCloud, maxZ);
			}
			else	{
				reprojectionDepth<uint16_t>(image1, disparityMap, stereoPointCloud, minZ, maxZ);
			}
			
		}
	}
	
	return stereoPointCloud;
}

// Pixel and World Coordinate Point Cloud Generator
void StereoCameraPointCloud::generateWorldDepthPCLPointCloud(cv::Mat image1, cv::Mat image2, double minZ, double maxZ, 
															 int disparityType, cv::Mat& filteredImage1, 
															 pcl::PointCloud<pcl::PointXYZRGB>::Ptr worldPC,
															 pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthPC,
															 REPROJECTION_MODE rMode)	{
	if(disparityType != CV_8U && disparityType != CV_8UC1 && 
	   disparityType != CV_16U && disparityType != CV_16UC1)	{
		std::cerr << "Only CV_8U and CV_16U supported\n";
		return;
	}

	cv::Size imageSize = image1.size();

	// Computed Disparity Map
	if(scd->computeDisparityMap(image1, image2, imageSize, TRIPLE))	{	
		// Get Computed Disparity Map in 8bit unsigned format
		cv::Mat disparityMap = scd->getNormDisparityMap(disparityType);
		// Reproject Point Cloud according to the type
		if(disparityType == CV_8U || disparityType == CV_8UC1)	{
			// Reproject Disparity Map in 3D world coordinate
			filteredImage1 = reprojectionWorldDepth<uint8_t>(image1, disparityMap, worldPC, depthPC, minZ, maxZ, rMode);
		}
		else if(disparityType == CV_16U || disparityType == CV_16UC1)	{
			// Reproject Disparity Map in 3D world coordinate
			filteredImage1 = reprojectionWorldDepth<uint16_t>(image1, disparityMap, worldPC, depthPC, minZ, maxZ, rMode);
		}
	}
}

void StereoCameraPointCloud::pointScaler(int disparityType, double& zScaleFactor, double& disparityScaleFactor, double& zOffset)	{
	if(disparityType == CV_8U || disparityType == CV_8UC1)	{
		zScaleFactor = 1.0;
		zOffset = 0.0;
		disparityScaleFactor = 1.0;
	}
	else if(disparityType == CV_16U || disparityType == CV_16UC1)	{
		zScaleFactor = 16.0;
		zOffset = 0.0;
		disparityScaleFactor = 1.0 / 16.0;
	}
}