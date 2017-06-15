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
#include "stereo_camera_disparity.h"

/*
	Constructor/Destructor
*/
StereoCameraDisparity::StereoCameraDisparity(StereoCameraCalibrationParameters* scp,
							  				 const bool calibrated) : calibratedCameras(calibrated)	{
	scCP = scp;
	blockMatchAlg = NA;
}

StereoCameraDisparity::~StereoCameraDisparity()	{

}

/*
	Getter/Setter
*/
cv::Mat StereoCameraDisparity::getRawDisparityMap()	{
	return rawDisparityMap;
}

cv::Mat StereoCameraDisparity::getNormDisparityMap(int type)	{
	cv::Mat normDisparityMap;
	/* ===================================================
		convert to normalized floating point disparity map
		SGBM and SBM both give 16bit signed-single channel
		Max 8Bit Sign = 2^7
		Max 16Bit Sign = 2^15
		Max 
		16Bit Sign --> 8Bit Sign: /16.
	====================================================*/
	if(type == CV_8S || type == CV_8SC1)	{
		cv::normalize(rawDisparityMap, normDisparityMap, -128, 127, CV_MINMAX, CV_8SC1);
	}
	else if(type == CV_8U || type == CV_8UC1)	{
		// rawDisparityMap.convertTo(normDisparityMap, CV_8UC1, 8.0/16.0);
		cv::normalize(rawDisparityMap, normDisparityMap, 0, 255, CV_MINMAX, CV_8UC1);
	}
	else if(type == CV_16S || type == CV_16SC1)	{
		return rawDisparityMap;
		// cv::normalize(rawDisparityMap, normDisparityMap, -32768, 32767, CV_MINMAX, CV_16S);
	}
	else if(type == CV_16U || type == CV_16UC1)	{
		// rawDisparityMap.convertTo(normDisparityMap, CV_16U);
		cv::normalize(rawDisparityMap, normDisparityMap, 0, 65535, CV_MINMAX, CV_16UC1);
	}
	else if(type == CV_32F)	{
		// return rawDisparityMap;
		rawDisparityMap.convertTo(normDisparityMap, CV_32F);
	}


	#ifdef DEBUGGING
		showImage("Norm Disparity", normDisparityMap);
	#endif

	return normDisparityMap;
}

/*
	Non-Template Class Methods
*/
// Rectify Methods
void StereoCameraDisparity::computeRectificationMatrix(cv::Size imageSize)	{
	cv::Mat R1 = cv::Mat(3, 3, CV_64F);
	cv::Mat R2 = cv::Mat(3, 3, CV_64F);
	if(calibratedCameras)	{
		cv::Mat P1 = cv::Mat(3, 4, CV_64F);
		cv::Mat P2 = cv::Mat(3, 4, CV_64F);

		// cv::stereoRectify(scCP->cameraMatrix1, scCP->distortionVector1, scCP->cameraMatrix2, scCP->distortionVector2, 
		// 				  imageSize, scCP->rotationMatrix, scCP->translationVector, R1, R2, P1, P2, scCP->qMatrix, 
		// 				  0,
		// 				  -1, imageSize, &roi1, &roi2);

		cv::stereoRectify(scCP->cameraMatrix1, scCP->distortionVector1, scCP->cameraMatrix2, scCP->distortionVector2, 
						  imageSize, scCP->rotationMatrix, scCP->translationVector, R1, R2, P1, P2, scCP->qMatrix, 
						  CV_CALIB_ZERO_DISPARITY,
						  -1, imageSize, &roi1, &roi2);
		
		//Precompute maps for cvRemap()
		cv::initUndistortRectifyMap(scCP->cameraMatrix1, scCP->distortionVector1, R1, P1, imageSize, CV_16SC2, imageMapX1, imageMapY1);
		cv::initUndistortRectifyMap(scCP->cameraMatrix2, scCP->distortionVector2, R2, P2, imageSize, CV_16SC2, imageMapX2, imageMapY2);
		// cv::initUndistortRectifyMap(scCP->cameraMatrix1, scCP->distortionVector1, R1, P1, imageSize, CV_32FC1, imageMapX1, imageMapY1);
		// cv::initUndistortRectifyMap(scCP->cameraMatrix2, scCP->distortionVector2, R2, P2, imageSize, CV_32FC1, imageMapX2, imageMapY2);
	}
	else	{
		// not implemented yet
	}
}

void StereoCameraDisparity::computeRectificationMatrix(CvSize imageSize)	{
	
}



bool StereoCameraDisparity::computeRectificationImage(cv::Mat matImage1, cv::Mat matImage2, cv::Size imageSize)	{
	if(matImage1.empty() || matImage2.empty())	{
		std::cerr<<"Invalid Image\n";
		return false;
	}
	
	// Stereo correspondences mapping
	// Linear no good
	// cv::remap(matImage1, *rectifiedImage1, *imageMapX1, *imageMapY1, CV_INTER_LINEAR);
	// cv::remap(matImage2, *rectifiedImage2, *imageMapX2, *imageMapY2, CV_INTER_LINEAR);
	// cv::remap(matImage1, rectifiedImage1, imageMapX1, imageMapY1, cv::INTER_NEAREST);
	// cv::remap(matImage2, rectifiedImage2, imageMapX2, imageMapY2, cv::INTER_NEAREST);
	// cv::remap(matImage1, rectifiedImage1, imageMapX1, imageMapY1, cv::INTER_CUBIC);
	// cv::remap(matImage2, rectifiedImage2, imageMapX2, imageMapY2, cv::INTER_CUBIC);
	cv::remap(matImage1, rectifiedImage1, imageMapX1, imageMapY1, cv::INTER_LANCZOS4);
	cv::remap(matImage2, rectifiedImage2, imageMapX2, imageMapY2, cv::INTER_LANCZOS4);

	#ifdef DEBUGGING
		// create side by side rectified image
		// cv::Mat* pair = stitchTwoImages(rectifiedImage1, rectifiedImage2, imageSize, scCP->getCameraMode());
		// showImage("Rectified", *pair);
		// delete pair;
		showImage("Overlay", overLayImages(rectifiedImage1, rectifiedImage2, 0.5));
		
	#endif	

    return true;
}


bool StereoCameraDisparity::computeRectificationImage(IplImage* matImage1, IplImage* matImage2, CvSize imageSize)	{

}

/*
	Disparity Methods
*/

void StereoCameraDisparity::stereoBMInit(cv::Size imageSize)	{

}

void StereoCameraDisparity::updateSBM(int pfc, int sadws, int md, int nod, int tt, int ur, 
									 int sws, int sr, int dmf)	{

}

void StereoCameraDisparity::stereoSGBMInit(cv::Size imageSize, SGBM_Params sgbmParams)	{
	ssgbm.preFilterCap = sgbmParams.preFilterCap;
	ssgbm.SADWindowSize = sgbmParams.SADWindowSize;
	ssgbm.minDisparity = sgbmParams.minDisparity;
	ssgbm.numberOfDisparities = sgbmParams.numberOfDisparities;
	ssgbm.uniquenessRatio = sgbmParams.uniquenessRatio;
	ssgbm.speckleWindowSize = sgbmParams.speckleWindowSize;
	ssgbm.speckleRange = sgbmParams.speckleRange;
	ssgbm.disp12MaxDiff = sgbmParams.disp12MaxDiff;
	ssgbm.fullDP = sgbmParams.fullDP;
	ssgbm.P1 = 600;//ssgbm.SADWindowSize * ssgbm.SADWindowSize * 8;
	ssgbm.P2 = 2400;//ssgbm.SADWindowSize * ssgbm.SADWindowSize * 32;
}

void StereoCameraDisparity::updateSGBM(SGBM_Params sgbmParams)	{
	ssgbm.preFilterCap = sgbmParams.preFilterCap;
	ssgbm.SADWindowSize = sgbmParams.SADWindowSize;
	ssgbm.minDisparity = sgbmParams.minDisparity;
	ssgbm.numberOfDisparities = sgbmParams.numberOfDisparities;
	ssgbm.uniquenessRatio = sgbmParams.uniquenessRatio;
	ssgbm.speckleWindowSize = sgbmParams.speckleWindowSize;
	ssgbm.speckleRange = sgbmParams.speckleRange;
	ssgbm.disp12MaxDiff = sgbmParams.disp12MaxDiff;
	ssgbm.fullDP = sgbmParams.fullDP;
}

void StereoCameraDisparity::updateSGBM(int pfc, int sadws, int md, int nod, int ur, 
									   int sws, int sr, int dmf, int fdp)	{
	ssgbm.preFilterCap = pfc;
	ssgbm.SADWindowSize = sadws;
	ssgbm.minDisparity = md;
	ssgbm.numberOfDisparities = nod;
	ssgbm.uniquenessRatio = ur;
	ssgbm.speckleWindowSize = sws;
	ssgbm.speckleRange = sr;
	ssgbm.disp12MaxDiff = dmf;
	ssgbm.fullDP = fdp;
}

bool StereoCameraDisparity::disparityInit(cv::Size imageSize, StereoCameraParameters* params)	{
	// Init block matching algorithm
	if(params->getClassType() == SCSGBMP)	{
		blockMatchAlg = SGBM;
		stereoSGBMInit(imageSize, ((StereoCameraSGBMParameters*)params)->getParams());
	}
	else	{
		std::cerr << "Error: Unsupported Block Matching Algorithm\nSupported Block Matching Algorithm: SGBM\n";
		return false;
	}
	// Compute Rectify Matrix, this is a one time process
	computeRectificationMatrix(imageSize);

	return true;
}

bool StereoCameraDisparity::computeDisparityMap(cv::Mat matImage1, cv::Mat matImage2, cv::Size imageSize, 
												CHANNELS inputChannel)	{
	if(inputChannel == TRIPLE)	{
		changeChannel(matImage1, matImage1, CV_RGB2GRAY);
		changeChannel(matImage2, matImage2, CV_RGB2GRAY);
	}
	
	// Compute Rectified and undistorted image 
	computeRectificationImage(matImage1, matImage2, imageSize);
	
	if(rectifiedImage1.empty() || rectifiedImage2.empty())	{
		std::cerr<<"Rectified Image not computed\n";
		return false;
	}

	// update
	if(scCP->getCameraMode() == SIDE_BY_SIDE || !calibratedCameras)	{
		if(blockMatchAlg == SGBM)	{
			#ifdef DEBUGGING
				std::clog<<"Using SGBM\n";
			#endif
			// Using SGBM to compute disparity map, output 16bit unsigned 
			ssgbm(rectifiedImage1, rectifiedImage2, rawDisparityMap);
		}
		else	{
			std::cerr << "Error: Unsupported Block Matching Algorithm\nSupported Block Matching Algorithm: SGBM\n";
		}
	}

	#ifdef DEBUGGING
		// show disparity map is show result is true
		getNormDisparityMap(SCIT_8U);
	#endif

	return true;
}

bool StereoCameraDisparity::computeDisparityMap(IplImage* matImage1, IplImage* matImage2, CvSize imageSize, 
												CHANNELS inputChannel)	{

}

/*
	Template Class Methods
*/
