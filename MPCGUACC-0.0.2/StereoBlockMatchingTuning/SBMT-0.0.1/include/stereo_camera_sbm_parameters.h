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
#ifndef STEREO_CAMERA_SBM_PARAMETERS_H
	#define STEREO_CAMERA_SBM_PARAMETERS_H
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
struct SBM_Params	{
	int preFilterCap;
	int SADWindowSize;
	int minDisparity;
	int numberOfDisparities;
	int uniquenessRatio;
	int speckleWindowSize;
	int speckleRange;
	int disp12MaxDiff;
	int textureThreshold;
};

class StereoCameraSBMParameters : public StereoCameraParameters	{
	private:
		const IMAGE_RESOLUTION imageResolution;
		Json::Value bmParams;
		const std::string bmParamsFilePath;

		/*
			Blocking Matching Algorithm Params
		*/
		SBM_Params params;
	protected:
	public:
		/*
			Constructor/Destructor
		*/
		StereoCameraSBMParameters(std::string bmFilePath, IMAGE_RESOLUTION imr);
		//@breif class destructor
		~StereoCameraSBMParameters();

		/*
			Getter/Setter
		*/
		SBM_Params getParams();
		IMAGE_RESOLUTION getImageResolution();
		int getClassType();

		/*
			Non Template Class Method
		*/
		bool storeParams(Json::Value bmValues, SBM_Params& params);
		bool readParams();
		void printParams(std::ostream& out);

		/*
			Template Class Method
		*/
};

#endif