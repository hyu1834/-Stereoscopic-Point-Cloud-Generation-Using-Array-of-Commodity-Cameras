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


*/
#ifndef STEREO_CAMERA_SGBM_PARAMETERS_H
	#define STEREO_CAMERA_SGBM_PARAMETERS_H
// Standard Libraries
#include <iostream>
#include <fstream>
#include <string>
#include <cassert>
#include <vector>

// 3rd Parties Libraries
//OpenCV
#include <cv.h>
#include <highgui.h>
//Jsoncpp
#include <json/json.h>

// Local Libraries
#include "stereo_camera_utils.h"
#include "stereo_camera_parameters.h"

// MACRO

// Enum
struct SGBM_Params	{
	int preFilterCap;
	int SADWindowSize;
	int minDisparity;
	int numberOfDisparities;
	int uniquenessRatio;
	int speckleWindowSize;
	int speckleRange;
	int disp12MaxDiff;
	int fullDP;
};

class StereoCameraSGBMParameters : public StereoCameraParameters	{
	private:
		Json::Value bmParams;
        std::string bmParamsFilePath;

		/*
			Blocking Matching Algorithm Params
		*/
		SGBM_Params params;
	protected:
	public:
		/*
			Constructor/Destructor
		*/
		StereoCameraSGBMParameters(std::string bmFilePath, IMAGE_RESOLUTION imr);
		StereoCameraSGBMParameters(int preFilterCap = 0, int SADWindowSize = 0, int minDisparity = 0, 
								   int numberOfDisparities = 0, int uniquenessRatio = 0, int speckleWindowSize = 0, 
								   int speckleRange = 0, int disp12MaxDiff = 0, int fullDP = 0);
		//@breif class destructor
		~StereoCameraSGBMParameters();

		/*
			Getter/Setter
		*/
		SGBM_Params getParams();
		int getClassType();
		void setBMPPath(std::string path);

		/*
			Non Template Class Method
		*/
		bool storeParams(Json::Value bmValues, SGBM_Params& params);
		bool readParams(IMAGE_RESOLUTION imageResolution);
		void printParams(std::ostream& out);
		void updateParams(int preFilterCap = 0, int SADWindowSize = 0, int minDisparity = 0, 
						  int numberOfDisparities = 0, int uniquenessRatio = 0, int speckleWindowSize = 0, 
						  int speckleRange = 0, int disp12MaxDiff = 0, int fullDP = 0);

		/*
			Template Class Method
		*/
};

#endif
