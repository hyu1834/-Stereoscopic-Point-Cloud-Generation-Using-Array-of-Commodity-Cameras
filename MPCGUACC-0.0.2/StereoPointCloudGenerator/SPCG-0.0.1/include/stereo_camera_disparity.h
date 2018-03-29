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
#ifndef STEREO_CAMERA_DISPARITY_H
	#define STEREO_CAMERA_DISPARITY_H
// Standard Libraries
#include <iostream>
#include <string>
#include <cassert>

// 3rd Parties Libraries
#include <cv.h>
#include <highgui.h>

// Local Libraries
#include "debugging.h"
#include "opencv_utils.h"
#include "stereo_camera_utils.h"
#include "stereo_camera_parameters.h"
#include "stereo_camera_calibration_parameters.h"
#include "stereo_camera_sgbm_parameters.h"

//MACRO

//Enum

class StereoCameraDisparity	{
	private:
		const bool calibratedCameras;
		// StereoCameraCalibrationParameters* scCP;
		STEREO_BM_ALG blockMatchAlg;

		// Recitify Variables
		cv::Mat imageMapX1;
		cv::Mat imageMapY1;
		cv::Mat imageMapX2;
		cv::Mat imageMapY2;
		cv::Mat rectifiedImage1;
		cv::Mat rectifiedImage2;
		cv::Mat rawDisparityMap;
		cv::Rect roi1;
		cv::Rect roi2;

		// Disparity Map Variables
		cv::StereoBM sbm;
		cv::StereoSGBM ssgbm;

		// Unrectified Camera Parameters
		StereoCameraCalibrationParameters* cameraCalibrationUnrectified;
		// Rectified Camera Parameters
		StereoCameraCalibrationParameters* cameraCalibrationRectified;

	protected:
	public:
		/*
			Constructor/Destructor
		*/
		StereoCameraDisparity(StereoCameraCalibrationParameters* cameraCalibrationUnrectified, 
							  StereoCameraCalibrationParameters* cameraCalibrationRectified, const bool calibrated = true);
		~StereoCameraDisparity();

		/*
			Getter/Setter
		*/
		cv::Mat getRawDisparityMap();
		cv::Mat getNormDisparityMap(int type);

		/*
			Non-Template Class Methods
		*/
		// rectify methods
		void computeRectificationMatrix(cv::Size  imageSize);
		void computeRectificationMatrix(CvSize imageSize);
		bool computeRectificationImage(cv::Mat matImage1, cv::Mat matImage2, cv::Size imageSize);
		bool computeRectificationImage(IplImage* matImage1, IplImage* matImage2, CvSize imageSize);
		
		//Disparity Methods
		void stereoBMInit(cv::Size imageSize);
		void updateSBM(int pfc, int sadws, int md, int nod, int tt, int ur, int sws, int sr, int dmf);
		void stereoSGBMInit(cv::Size imageSize, SGBM_Params sgbmParams);
		void updateSGBM(SGBM_Params sgbmParams);
		void updateSGBM(int pfc, int sadws, int md, int nod, int ur, 
						int sws, int sr, int dmf, int fdp);
		bool disparityInit(cv::Size imageSize, StereoCameraParameters* params);

		bool computeDisparityMap(cv::Mat matImage1, cv::Mat matImage2, cv::Size imageSize, 
								 CHANNELS inputChannel = SINGLE);
		bool computeDisparityMap(IplImage* matImage1, IplImage* matImage2, CvSize imageSize, CHANNELS inputChannel = SINGLE);
			

		/*
			Template Class Methods
		*/
		
	

};

#endif