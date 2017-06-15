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
#ifndef STEREO_CAMERA_PARAMETERS_H
	#define STEREO_CAMERA_PARAMETERS_H
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

// Local Libraries
#include "stereo_camera_utils.h"

// MACRO

// Enum
enum STEREO_CAMERA_PARAMETERS_TYPE	{
	SCP,
	SCCP,
	SCSGBMP,
	SCSBMP
};

class StereoCameraParameters	{
	private:
		
	protected:
		// const STEREOMODE cameraMode;
		// const IMAGE_RESOLUTION imageResolution;
	public:
		

		/*
			Blocking Matching Algorithm Params
		*/
		// SGBM_Params sgbmParams;
		/*
			Constructor/Destructor
		*/
		StereoCameraParameters();
		//@breif class destructor
		~StereoCameraParameters();

		/*
			Getter/Setter
		*/
		// STEREOMODE getCameraMode();
		virtual int getClassType();

		/*
			Non Template Class Method
		*/
		virtual bool readParams();
		virtual void printParams(std::ostream& out);

		/*
			Template Class Method
		*/
};

#endif