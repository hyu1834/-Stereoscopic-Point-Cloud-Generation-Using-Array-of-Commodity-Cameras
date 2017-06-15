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
#ifndef STEREO_CAMERA_POINT_CLOUD_H
	#define STEREO_CAMERA_POINT_CLOUD_H
// Standard Libraries
// #include <iostream>
// #include <fstream>
#include <map>
#include <string>
#include <cassert>
#include <limits> 

// 3rd Parties Libraries

//OpenCV
#include <cv.h>
#include <highgui.h>
#include "opencv2/core/core.hpp"
//PCL
#include <pcl/common/common_headers.h>

// Local Libraries
#include "debugging.h"
#include "opencv_utils.h"
#include "stereo_camera_utils.h"
#include "stereo_camera_parameters.h"
#include "stereo_camera_calibration_parameters.h"
#include "stereo_camera_sgbm_parameters.h"
#include "stereo_camera_disparity.h"
#include "point_cloud_utils.h"

//MACRO
enum REPROJECTION_METHODS {
	OPENCV,
	STANDARD,
	PIXEL,
	CUSTOM
};

enum REPROJECTION_MODE	{
	RMODE_2D, 
	RMODE_3D
};

class StereoCameraPointCloud	{
	private:
		std::map<cv::Point2d, pcl::PointXYZRGB, PointComp> pixelToWorld;
		// double FOV;
		// double cameraDistance;

		// Stereo Calibration Parameters
		StereoCameraCalibrationParameters* sccp;

		// Disparity Map
		StereoCameraDisparity* scd;

	protected:
	public:
		/*
			Constructor/Destructor
		*/
		StereoCameraPointCloud(StereoCameraCalibrationParameters* scp, bool calibrate);
		~StereoCameraPointCloud();

		/*
			Getter/Setter
		*/
		std::map<cv::Point2d, pcl::PointXYZRGB, PointComp> getPixelToWorldMap();
		pcl::PointXYZRGB getPoints(int x, int y);
		pcl::PointXYZRGB getPoints(double x, double y);
		pcl::PointXYZRGB getPoints(cv::Point2d coordinate);

		/*
			Non-Template Class Methods
		*/
		// World Coordinate Point Cloud Generator
		bool init(cv::Size imageSize, StereoCameraParameters* bmp);

		void generateWorldPCLPointCloud(cv::Mat image1, cv::Mat image2, double minZ, double maxZ, int disparityType,
										cv::Mat& filteredImage1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc,
										REPROJECTION_MODE rMode = RMODE_3D);
		void generateWorldPCLPointCloud(cv::Mat image1, cv::Mat image2, double minZ, double maxZ, int disparityType,
										pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc,
										REPROJECTION_MODE rMode = RMODE_3D);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr generateWorldPCLPointCloud(cv::Mat image1, cv::Mat image2, 
																	 	  double minZ, double maxZ, int disparityType,
																		  REPROJECTION_MODE rMode = RMODE_3D);
		
		// Pixel Coordinate Point Cloud Generator
		void generateDepthPCLPointCloud(cv::Mat image1, cv::Mat image2, double minZ, double maxZ, int disparityType,
										cv::Mat& filteredImage1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc,
										REPROJECTION_MODE rMode = RMODE_3D);
		void generateDepthPCLPointCloud(cv::Mat image1, cv::Mat image2, double minZ, double maxZ, int disparityType,
										pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc,
										REPROJECTION_MODE rMode = RMODE_3D);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr generateDepthPCLPointCloud(cv::Mat image1, cv::Mat image2, 
																		  double minZ, double maxZ, 
																		  int disparityType, 
																		  REPROJECTION_MODE rMode = RMODE_3D);

		// Pixel and World Coordinate Point Cloud Generator
		void generateWorldDepthPCLPointCloud(cv::Mat image1, cv::Mat image2, double minZ, double maxZ, int disparityType,
											 cv::Mat& filteredImage1, 
											 pcl::PointCloud<pcl::PointXYZRGB>::Ptr worldPC,
											 pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthPC,
											 REPROJECTION_MODE rMode);

		void pointScaler(int disparityType, double& zScaleFactor, double& disparityScaleFactor, double& zOffset);
		
		/*
			Template Class Methods
		*/
		template <typename T>
		void reprojectionWorld(const cv::Mat bgrImage, cv::Mat dispImage, 
							   pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointCloud, double minZ,
							   double maxZ, cv::Mat& filteredBGRImage, REPROJECTION_METHODS method = OPENCV,
							   REPROJECTION_MODE rMode = RMODE_3D)	{
			// Check if disparity type is allowed
			int disparityType = getCVMatType(dispImage);
			if(disparityType != CV_8U && disparityType != CV_8UC1 && 
			   disparityType != CV_16U && disparityType != CV_16UC1)	{
				std::cerr << "Only CV_8U && CV_16U are supported\n";
				return bgrImage;
			}

			// Local variables
			pcl::PointXYZRGB point;
			filteredBGRImage = bgrImage.clone();
			double disparity, zScaleFactor, zOffset, disparityScaleFactor;
			// Reprojection Variables
			double cx, cy, baseline, focal_length, deltaCx;

			// Get point scaler
			pointScaler(disparityType, zScaleFactor, disparityScaleFactor, zOffset);

			// Get Reprojection Constants
			cx = -sccp->qMatrix.at<double>(0, 3);
			cy = -sccp->qMatrix.at<double>(1, 3);
			baseline = -sccp->translationVector.at<double>(0, 0);
			focal_length = sccp->qMatrix.at<double>(2, 3);
			deltaCx = sccp->qMatrix.at<double>(3, 3) * baseline; 

			// Solve the 3D Reproject according to the Methods
			if(method == OPENCV)	{
				cv::Mat_<double> pointVec(4,1);
				dispImage = dispImage * disparityScaleFactor;
				for(int y = 0; y < dispImage.rows; y++)	{
					for(int x = 0; x < dispImage.cols; x++)	{
						disparity = static_cast<double>(dispImage.at<T>(y, x));

						if(disparity == 0 || disparity == std::numeric_limits<T>::max())	{
							filteredBGRImage.at<cv::Vec3b>(y, x)[0] = 0;
							filteredBGRImage.at<cv::Vec3b>(y, x)[1] = 0;
							filteredBGRImage.at<cv::Vec3b>(y, x)[2] = 0;
							continue;
						}

						/*
							Reproject points to 3D world coordinate by the following equations
							[X Y Z W]^T = [x y disparity(x,y) 1]^T
							_3dImage(x,y) = (X/W, Y/W, Z/W)

						*/
						//Construct point vector
						pointVec(0) = x;
						pointVec(1) = y;
						pointVec(2) = disparity;
						pointVec(3) = 1;
						// 3D Projection
						pointVec = sccp->qMatrix * pointVec;
						pointVec /= pointVec(3);
						// if the Z is > than max Z, we will ignore it
						if(abs(pointVec(2)) < minZ || abs(pointVec(2)) > maxZ)	{
						// if(abs(pointVec(2)) > maxZ || abs(pointVec(2)) < minZ)	{
							filteredBGRImage.at<cv::Vec3b>(y, x)[0] = 0;
							filteredBGRImage.at<cv::Vec3b>(y, x)[1] = 0;
							filteredBGRImage.at<cv::Vec3b>(y, x)[2] = 0;
							continue;
						}
						//Get RGB info
						point.r = bgrImage.at<uint8_t>(y, 3 * x + 2);
						point.g = bgrImage.at<uint8_t>(y, 3 * x + 1);
						point.b = bgrImage.at<uint8_t>(y, 3 * x);
						// Get XYZ coordinate
						point.x = pointVec(0);
						point.y = pointVec(1);
						if(rMode == RMODE_2D)	{
							point.z = 0.0;
						}
						else	{
							point.z = pointVec(2) * zScaleFactor + zOffset;
						}
						
						// Add points to PCL PointXYZRGB Container
						pcl::PointXYZRGB worldPoint = rotation3D<pcl::PointXYZRGB>(point, 180, X_AXIS);
						pclPointCloud->push_back(worldPoint);
						// Add points to pixel to world map
						// pixelToWorld[cv::Point2d(x, y)] = worldPoint;
					}
				}
			}
			else if(method == STANDARD)	{
				double pointZ;
				for(int y = 0; y < dispImage.rows; y++)	{
					for(int x = 0; x < dispImage.cols; x++)	{
						disparity = static_cast<double>(dispImage.at<T>(y, x)) * disparityScaleFactor;
						
						if(disparity == 0)	{
							filteredBGRImage.at<cv::Vec3b>(y, x)[0] = 0;
							filteredBGRImage.at<cv::Vec3b>(y, x)[1] = 0;
							filteredBGRImage.at<cv::Vec3b>(y, x)[2] = 0;
							continue;
						}

						/*
							Reproject points to 3D world coordinate by the following equations
							X = Z * (x - cx) / f
							Y = Z * (y - cy) / f
							Z = f * baseline / disparity
						*/

						pointZ = (focal_length * baseline / (disparity + deltaCx)) * zScaleFactor + zOffset;
						// if the Z is > than max Z, we will ignore it
						if(abs(pointZ) < minZ || abs(pointZ) > maxZ)	{
						// if(abs(pointZ) > maxZ)	{
							filteredBGRImage.at<cv::Vec3b>(y, x)[0] = 0;
							filteredBGRImage.at<cv::Vec3b>(y, x)[1] = 0;
							filteredBGRImage.at<cv::Vec3b>(y, x)[2] = 0;
							continue;
						}

						//Get RGB info
						point.r = bgrImage.at<uint8_t>(y, 3 * x + 2);
						point.g = bgrImage.at<uint8_t>(y, 3 * x + 1);
						point.b = bgrImage.at<uint8_t>(y, 3 * x);
						// Get XY coordinate
						point.x = pointZ * (x - cx) / focal_length;
						point.y = pointZ * (y - cy) / focal_length;
						if(rMode == RMODE_2D)	{
							point.z = 0.0;
						}
						else	{
							point.z = pointZ;
						}
						// Add points to PCL PointXYZRGB Container
						pcl::PointXYZRGB worldPoint = rotation3D<pcl::PointXYZRGB>(point, 180, X_AXIS);
						pclPointCloud->push_back(worldPoint);
						// Add points to pixel to world map
						pixelToWorld.insert(std::pair<cv::Point2d, pcl::PointXYZRGB>(cv::Point2d(x, y), worldPoint));
					}
				}
			}
		}

		template <typename T>
		cv::Mat reprojectionWorld(const cv::Mat bgrImage, cv::Mat dispImage, 
								  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointCloud, double minZ, 
								  double maxZ, REPROJECTION_METHODS method = OPENCV,
								  REPROJECTION_MODE rMode = RMODE_3D)	{
			// Check if disparity type is allowed
			int disparityType = getCVMatType(dispImage);
			if(disparityType != CV_8U && disparityType != CV_8UC1 && 
			   disparityType != CV_16U && disparityType != CV_16UC1)	{
				std::cerr << "Only CV_8U && CV_16U are supported\n";
				return bgrImage;
			}

			// Local variables
			pcl::PointXYZRGB point;
			cv::Mat filteredBGRImage = bgrImage.clone();
			double disparity, zScaleFactor, zOffset, disparityScaleFactor;
			// Reprojection Variables
			double cx, cy, baseline, focal_length, deltaCx;

			// Get point scaler
			pointScaler(disparityType, zScaleFactor, disparityScaleFactor, zOffset);

			// Get Reprojection Constants
			cx = -sccp->qMatrix.at<double>(0, 3);
			cy = -sccp->qMatrix.at<double>(1, 3);
			baseline = -sccp->translationVector.at<double>(0, 0);
			focal_length = sccp->qMatrix.at<double>(2, 3);
			deltaCx = sccp->qMatrix.at<double>(3, 3) * baseline; 

			// Solve the 3D Reproject according to the Methods
			if(method == OPENCV)	{
				cv::Mat_<double> pointVec(4,1);
				dispImage = dispImage * disparityScaleFactor;
				for(int y = 0; y < dispImage.rows; y++)	{
					for(int x = 0; x < dispImage.cols; x++)	{
						disparity = static_cast<double>(dispImage.at<T>(y, x));

						if(disparity == 0 || disparity == std::numeric_limits<T>::max())	{
							filteredBGRImage.at<cv::Vec3b>(y, x)[0] = 0;
							filteredBGRImage.at<cv::Vec3b>(y, x)[1] = 0;
							filteredBGRImage.at<cv::Vec3b>(y, x)[2] = 0;
							continue;
						}

						/*
							Reproject points to 3D world coordinate by the following equations
							[X Y Z W]^T = [x y disparity(x,y) 1]^T
							_3dImage(x,y) = (X/W, Y/W, Z/W)

						*/
						//Construct point vector
						pointVec(0) = x;
						pointVec(1) = y;
						pointVec(2) = disparity;
						pointVec(3) = 1;
						// 3D Projection
						pointVec = sccp->qMatrix * pointVec;
						pointVec /= pointVec(3);
						// if the Z is > than max Z, we will ignore it
						if(abs(pointVec(2)) < minZ || abs(pointVec(2)) > maxZ)	{
						// if(abs(pointVec(2)) > maxZ)	{
							filteredBGRImage.at<cv::Vec3b>(y, x)[0] = 0;
							filteredBGRImage.at<cv::Vec3b>(y, x)[1] = 0;
							filteredBGRImage.at<cv::Vec3b>(y, x)[2] = 0;
							continue;
						}
						//Get RGB info
						point.r = bgrImage.at<uint8_t>(y, 3 * x + 2);
						point.g = bgrImage.at<uint8_t>(y, 3 * x + 1);
						point.b = bgrImage.at<uint8_t>(y, 3 * x);
						// Get XYZ coordinate
						point.x = pointVec(0);
						point.y = pointVec(1);
						if(rMode == RMODE_2D)	{
							point.z = 0.0;
						}
						else	{
							point.z = pointVec(2) * zScaleFactor + zOffset;
						}
						
						// Add points to PCL PointXYZRGB Container
						pcl::PointXYZRGB worldPoint = rotation3D<pcl::PointXYZRGB>(point, 180, X_AXIS);
						pclPointCloud->push_back(worldPoint);
						// Add points to pixel to world map
						// pixelToWorld[cv::Point2d(x, y)] = worldPoint;
					}
				}
			}
			else if(method == STANDARD)	{
				double pointZ;
				for(int y = 0; y < dispImage.rows; y++)	{
					for(int x = 0; x < dispImage.cols; x++)	{
						disparity = static_cast<double>(dispImage.at<T>(y, x)) * disparityScaleFactor;
						
						if(disparity == 0)	{
							filteredBGRImage.at<cv::Vec3b>(y, x)[0] = 0;
							filteredBGRImage.at<cv::Vec3b>(y, x)[1] = 0;
							filteredBGRImage.at<cv::Vec3b>(y, x)[2] = 0;
							continue;
						}

						/*
							Reproject points to 3D world coordinate by the following equations
							X = Z * (x - cx) / f
							Y = Z * (y - cy) / f
							Z = f * baseline / disparity
						*/

						pointZ = (focal_length * baseline / (disparity + deltaCx)) * zScaleFactor + zOffset;
						// if the Z is > than max Z, we will ignore it
						if(abs(pointZ) < minZ || abs(pointZ) > maxZ)	{
						// if(abs(pointZ) > maxZ)	{
							filteredBGRImage.at<cv::Vec3b>(y, x)[0] = 0;
							filteredBGRImage.at<cv::Vec3b>(y, x)[1] = 0;
							filteredBGRImage.at<cv::Vec3b>(y, x)[2] = 0;
							continue;
						}

						//Get RGB info
						point.r = bgrImage.at<uint8_t>(y, 3 * x + 2);
						point.g = bgrImage.at<uint8_t>(y, 3 * x + 1);
						point.b = bgrImage.at<uint8_t>(y, 3 * x);
						// Get XY coordinate
						point.x = pointZ * (x - cx) / focal_length;
						point.y = pointZ * (y - cy) / focal_length;
						if(rMode == RMODE_2D)	{
							point.z = 0.0;
						}
						else	{
							point.z = pointZ;
						}
						// Add points to PCL PointXYZRGB Container
						pcl::PointXYZRGB worldPoint = rotation3D<pcl::PointXYZRGB>(point, 180, X_AXIS);
						pclPointCloud->push_back(worldPoint);
						// Add points to pixel to world map
						pixelToWorld.insert(std::pair<cv::Point2d, pcl::PointXYZRGB>(cv::Point2d(x, y), worldPoint));
					}
				}
			}
			return filteredBGRImage;
		}


		template <typename T>
		void reprojectionDepth(const cv::Mat bgrImage, cv::Mat dispImage, 
							   pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointCloud, double minZ, 
							   double maxZ, cv::Mat& filteredBGRImage, REPROJECTION_MODE rMode = RMODE_3D)	{
			// Check if disparity type is allowed
			int disparityType = getCVMatType(dispImage);
			if(disparityType != CV_8U && disparityType != CV_8UC1 && 
			   disparityType != CV_16U && disparityType != CV_16UC1)	{
				std::cerr << "Only CV_8U && CV_16U supported\n";
				return bgrImage;
			}

			// Local variables
			pcl::PointXYZRGB point;
			filteredBGRImage = bgrImage.clone();
			double disparity, zScaleFactor, zOffset, disparityScaleFactor;
			// Reprojection Variables
			double cx, cy, baseline, focal_length, deltaCx;

			// Get point scaler
			pointScaler(disparityType, zScaleFactor, disparityScaleFactor, zOffset);

			// Get Reprojection Constants
			cx = -sccp->qMatrix.at<double>(0, 3);
			cy = -sccp->qMatrix.at<double>(1, 3);
			baseline = -sccp->translationVector.at<double>(0, 0);
			focal_length = sccp->qMatrix.at<double>(2, 3);
			deltaCx = sccp->qMatrix.at<double>(3, 3) * baseline; 

			
			double z;
			for(int y = 0; y < dispImage.rows; y++)	{
				for(int x = 0; x < dispImage.cols; x++)	{
					disparity = static_cast<double>(dispImage.at<T>(y, x)) * disparityScaleFactor;				
					if(disparity == 0)	{
						filteredBGRImage.at<cv::Vec3b>(y, x)[0] = 0;
						filteredBGRImage.at<cv::Vec3b>(y, x)[1] = 0;
						filteredBGRImage.at<cv::Vec3b>(y, x)[2] = 0;
						continue;
					}

					/*
						Reproject points to 3D world coordinate by the following equations
						X = Z * (x - cx) / f
						Y = Z * (y - cy) / f
						Z = f * baseline / disparity
					*/

					z = (focal_length * baseline / (disparity + deltaCx)) * zScaleFactor + zOffset;
					// if the Z is > than max Z, we will ignore it
					if(abs(z) < minZ || abs(z) > maxZ)	{
					// if(abs(z) > maxZ)	{
						filteredBGRImage.at<cv::Vec3b>(y, x)[0] = 0;
						filteredBGRImage.at<cv::Vec3b>(y, x)[1] = 0;
						filteredBGRImage.at<cv::Vec3b>(y, x)[2] = 0;
						continue;
					}

					//Get RGB info
					point.r = bgrImage.at<uint8_t>(y, 3 * x + 2);
					point.g = bgrImage.at<uint8_t>(y, 3 * x + 1);
					point.b = bgrImage.at<uint8_t>(y, 3 * x);
					// Get XY coordinate
					point.x = x;
					point.y = -y;
					if(rMode == RMODE_2D)	{
						point.z = 0.0;
					}
					else	{
						point.z = -z;
					}

					// Add point to PCL PointXYZRGB Container
					pclPointCloud->push_back(point);
				}
			}
		}

		template <typename T>
		cv::Mat reprojectionDepth(const cv::Mat bgrImage, cv::Mat dispImage, 
									pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointCloud, double minZ, 
									double maxZ, REPROJECTION_MODE rMode = RMODE_3D)	{
			// Check if disparity type is allowed
			int disparityType = getCVMatType(dispImage);
			if(disparityType != CV_8U && disparityType != CV_8UC1 && 
			   disparityType != CV_16U && disparityType != CV_16UC1)	{
				std::cerr << "Only CV_8U && CV_16U supported\n";
				return bgrImage;
			}

			// Local variables
			pcl::PointXYZRGB point;
			cv::Mat filteredBGRImage = bgrImage.clone();
			double disparity, zScaleFactor, zOffset, disparityScaleFactor;
			// Reprojection Variables
			double cx, cy, baseline, focal_length, deltaCx;

			// Get point scaler
			pointScaler(disparityType, zScaleFactor, disparityScaleFactor, zOffset);

			// Get Reprojection Constants
			cx = -sccp->qMatrix.at<double>(0, 3);
			cy = -sccp->qMatrix.at<double>(1, 3);
			baseline = -sccp->translationVector.at<double>(0, 0);
			focal_length = sccp->qMatrix.at<double>(2, 3);
			deltaCx = sccp->qMatrix.at<double>(3, 3) * baseline; 

			
			double z;
			for(int y = 0; y < dispImage.rows; y++)	{
				for(int x = 0; x < dispImage.cols; x++)	{
					disparity = static_cast<double>(dispImage.at<T>(y, x)) * disparityScaleFactor;				
					if(disparity == 0)	{
						filteredBGRImage.at<cv::Vec3b>(y, x)[0] = 0;
						filteredBGRImage.at<cv::Vec3b>(y, x)[1] = 0;
						filteredBGRImage.at<cv::Vec3b>(y, x)[2] = 0;
						continue;
					}

					/*
						Reproject points to 3D world coordinate by the following equations
						X = Z * (x - cx) / f
						Y = Z * (y - cy) / f
						Z = f * baseline / disparity
					*/

					z = (focal_length * baseline / (disparity + deltaCx)) * zScaleFactor + zOffset;
					// if the Z is > than max Z, we will ignore it
					if(abs(z) < minZ || abs(z) > maxZ)	{
					// if(abs(z) > maxZ)	{
						filteredBGRImage.at<cv::Vec3b>(y, x)[0] = 0;
						filteredBGRImage.at<cv::Vec3b>(y, x)[1] = 0;
						filteredBGRImage.at<cv::Vec3b>(y, x)[2] = 0;
						continue;
					}

					//Get RGB info
					point.r = bgrImage.at<uint8_t>(y, 3 * x + 2);
					point.g = bgrImage.at<uint8_t>(y, 3 * x + 1);
					point.b = bgrImage.at<uint8_t>(y, 3 * x);
					// Get XY coordinate
					point.x = x;
					point.y = -y;
					if(rMode == RMODE_2D)	{
						point.z = 0.0;
					}
					else	{
						point.z = -z;
					}

					// Add point to PCL PointXYZRGB Container
					pclPointCloud->push_back(point);
				}
			}

			return filteredBGRImage;
		}


		template <typename T>
		void reprojectionWorldDepth(const cv::Mat bgrImage, cv::Mat dispImage, 
									pcl::PointCloud<pcl::PointXYZRGB>::Ptr worldPC,
									pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthPC,
									double minZ, double maxZ, cv::Mat& filteredBGRImage,
									REPROJECTION_MODE rMode = RMODE_3D)	{
			// Check if disparity type is allowed
			int disparityType = getCVMatType(dispImage);
			if(disparityType != CV_8U && disparityType != CV_8UC1 && 
			   disparityType != CV_16U && disparityType != CV_16UC1)	{
				std::cerr << "Only CV_8U && CV_16U are supported\n";
				return bgrImage;
			}

			// Local variables
			pcl::PointXYZRGB depthPoint, worldPoint;
			filteredBGRImage = bgrImage.clone();
			double disparity, zScaleFactor, zOffset, disparityScaleFactor;
			// Reprojection Variables
			double cx, cy, baseline, focal_length, deltaCx;

			// Get point scaler
			pointScaler(disparityType, zScaleFactor, disparityScaleFactor, zOffset);

			// Get Reprojection Constants
			cx = -sccp->qMatrix.at<double>(0, 3);
			cy = -sccp->qMatrix.at<double>(1, 3);
			baseline = -sccp->translationVector.at<double>(0, 0);
			focal_length = sccp->qMatrix.at<double>(2, 3);
			deltaCx = sccp->qMatrix.at<double>(3, 3) * baseline; 


			double pointZ;
			for(int y = 0; y < dispImage.rows; y++)	{
				for(int x = 0; x < dispImage.cols; x++)	{
					disparity = static_cast<double>(dispImage.at<T>(y, x)) * disparityScaleFactor;
					
					if(disparity == 0)	{
						filteredBGRImage.at<cv::Vec3b>(y, x)[0] = 0;
						filteredBGRImage.at<cv::Vec3b>(y, x)[1] = 0;
						filteredBGRImage.at<cv::Vec3b>(y, x)[2] = 0;
						continue;
					}

					/*
						Reproject points to 3D world coordinate by the following equations
						X = Z * (x - cx) / f
						Y = Z * (y - cy) / f
						Z = f * baseline / disparity
					*/

					pointZ = (focal_length * baseline / (disparity + deltaCx)) * zScaleFactor + zOffset;
					// if the Z is > than max Z, we will ignore it
					if(abs(pointZ) < minZ || abs(pointZ) > maxZ)	{
					// if(abs(pointZ) > maxZ)	{
						filteredBGRImage.at<cv::Vec3b>(y, x)[0] = 0;
						filteredBGRImage.at<cv::Vec3b>(y, x)[1] = 0;
						filteredBGRImage.at<cv::Vec3b>(y, x)[2] = 0;
						continue;
					}

					//Get RGB info
					worldPoint.r = bgrImage.at<uint8_t>(y, 3 * x + 2);
					worldPoint.g = bgrImage.at<uint8_t>(y, 3 * x + 1);
					worldPoint.b = bgrImage.at<uint8_t>(y, 3 * x);

					depthPoint.r = bgrImage.at<uint8_t>(y, 3 * x + 2);
					depthPoint.g = bgrImage.at<uint8_t>(y, 3 * x + 1);
					depthPoint.b = bgrImage.at<uint8_t>(y, 3 * x);

					// Get XY coordinate
					worldPoint.x = pointZ * (x - cx) / focal_length;
					worldPoint.y = pointZ * (y - cy) / focal_length;

					depthPoint.x = x;
					depthPoint.y = -y;
					if(rMode == RMODE_2D)	{
						worldPoint.z = 0.0;
						depthPoint.z = 0.0;
					}
					else	{
						worldPoint.z = pointZ;
						depthPoint.z = -pointZ;
					}

					worldPoint = rotation3D<pcl::PointXYZRGB>(worldPoint, 180, X_AXIS);
					// Add points to PCL PointXYZRGB Container
					worldPC->push_back(worldPoint);
					depthPC->push_back(depthPoint);
					// Add points to pixel to world map
					pixelToWorld.insert(std::pair<cv::Point2d, pcl::PointXYZRGB>(cv::Point2d(x, y), worldPoint));
				}
			}
		}

		template <typename T>
		cv::Mat reprojectionWorldDepth(const cv::Mat bgrImage, cv::Mat dispImage, 
									   pcl::PointCloud<pcl::PointXYZRGB>::Ptr worldPC,
									   pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthPC, double minZ,
									   double maxZ, REPROJECTION_MODE rMode = RMODE_3D)	{
			// Check if disparity type is allowed
			int disparityType = getCVMatType(dispImage);
			if(disparityType != CV_8U && disparityType != CV_8UC1 && 
			   disparityType != CV_16U && disparityType != CV_16UC1)	{
				std::cerr << "Only CV_8U && CV_16U are supported\n";
				return bgrImage;
			}

			// Local variables
			pcl::PointXYZRGB depthPoint, worldPoint;
			cv::Mat filteredBGRImage = bgrImage.clone();
			double disparity, zScaleFactor, zOffset, disparityScaleFactor;
			// Reprojection Variables
			double cx, cy, baseline, focal_length, deltaCx;

			// Get point scaler
			pointScaler(disparityType, zScaleFactor, disparityScaleFactor, zOffset);

			// Get Reprojection Constants
			cx = -sccp->qMatrix.at<double>(0, 3);
			cy = -sccp->qMatrix.at<double>(1, 3);
			baseline = -sccp->translationVector.at<double>(0, 0);
			focal_length = sccp->qMatrix.at<double>(2, 3);
			deltaCx = sccp->qMatrix.at<double>(3, 3) * baseline; 


			double pointZ;
			for(int y = 0; y < dispImage.rows; y++)	{
				for(int x = 0; x < dispImage.cols; x++)	{
					disparity = static_cast<double>(dispImage.at<T>(y, x)) * disparityScaleFactor;
					
					if(disparity == 0)	{
						filteredBGRImage.at<cv::Vec3b>(y, x)[0] = 0;
						filteredBGRImage.at<cv::Vec3b>(y, x)[1] = 0;
						filteredBGRImage.at<cv::Vec3b>(y, x)[2] = 0;
						continue;
					}

					/*
						Reproject points to 3D world coordinate by the following equations
						X = Z * (x - cx) / f
						Y = Z * (y - cy) / f
						Z = f * baseline / disparity
					*/

					pointZ = (focal_length * baseline / (disparity + deltaCx)) * zScaleFactor + zOffset;
					// if the Z is > than max Z, we will ignore it
					if(abs(pointZ) < minZ || abs(pointZ) > maxZ)	{
					// if(abs(pointZ) > maxZ)	{
						filteredBGRImage.at<cv::Vec3b>(y, x)[0] = 0;
						filteredBGRImage.at<cv::Vec3b>(y, x)[1] = 0;
						filteredBGRImage.at<cv::Vec3b>(y, x)[2] = 0;
						continue;
					}

					//Get RGB info
					worldPoint.r = bgrImage.at<uint8_t>(y, 3 * x + 2);
					worldPoint.g = bgrImage.at<uint8_t>(y, 3 * x + 1);
					worldPoint.b = bgrImage.at<uint8_t>(y, 3 * x);

					depthPoint.r = bgrImage.at<uint8_t>(y, 3 * x + 2);
					depthPoint.g = bgrImage.at<uint8_t>(y, 3 * x + 1);
					depthPoint.b = bgrImage.at<uint8_t>(y, 3 * x);

					// Get XY coordinate
					worldPoint.x = pointZ * (x - cx) / focal_length;
					worldPoint.y = pointZ * (y - cy) / focal_length;

					depthPoint.x = x;
					depthPoint.y = -y;
					if(rMode == RMODE_2D)	{
						worldPoint.z = 0.0;
						depthPoint.z = 0.0;
					}
					else	{
						worldPoint.z = pointZ;
						depthPoint.z = -pointZ;
					}

					worldPoint = rotation3D<pcl::PointXYZRGB>(worldPoint, 180, X_AXIS);
					// Add points to PCL PointXYZRGB Container
					worldPC->push_back(worldPoint);
					depthPC->push_back(depthPoint);
					// Add points to pixel to world map
					pixelToWorld.insert(std::pair<cv::Point2d, pcl::PointXYZRGB>(cv::Point2d(x, y), worldPoint));
				}
			}

			return filteredBGRImage;
		}
};

#endif