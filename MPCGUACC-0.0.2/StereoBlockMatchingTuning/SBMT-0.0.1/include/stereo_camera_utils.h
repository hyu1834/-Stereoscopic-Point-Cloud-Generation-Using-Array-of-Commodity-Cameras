#ifndef STEREO_CAMERA_UTILS_H
	#define STEREO_CAMERA_UTILS_H
// Standard Libraries
#include <cstring>
#include <string>

// 3rd Parties Libraries
#include <cv.h>
#include <highgui.h>

// Local Libraries


//MACRO
// #define DEBUGGING
enum CALIBRATIONMODE	{
	CALIBRATED = true,
	NOT_CALIBRATED = false
};

enum STEREOMODE	{
	SIDE_BY_SIDE = 0,
	TOP_BOTTOM
};

enum STEREO_BM_ALG	{
	SBM = 0,
	SGBM,
	NA
};

enum CHANNELS	{
	TRIPLE = 0,
	SINGLE
};

enum IMAGE_TYPE	{
	SCIT_8S = 0,
	SCIT_8U,
	SCIT_16S,
	SCIT_16U,
	SCIT_32F
};

enum IMAGE_RESOLUTION	{
	IMR_480 = 480,
	IMR_720 = 720,
	IMR_1080 = 1080,
	IMR_5MP = 5400,
    IMR_8MP = 8640,
	IGNORE = 0xffff
};

/*
	IplImage C Functions
*/


/*
	cv::Mat C++ Functions
*/
cv::Mat* stitchTwoImages(cv::Mat* image1, cv::Mat* image2, cv::Size imageSize, STEREOMODE mode);
cv::Mat* stitchTwoImages(cv::Mat image1, cv::Mat image2, cv::Size imageSize, STEREOMODE mode);


#endif
