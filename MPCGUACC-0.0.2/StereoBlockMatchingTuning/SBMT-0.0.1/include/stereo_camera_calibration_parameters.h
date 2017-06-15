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
#ifndef STEREO_CAMERA_CALIBRATION_PARAMETERS_H
	#define STEREO_CAMERA_CALIBRATION_PARAMETERS_H
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
#include "stereo_camera_parameters.h"

// MACRO

// Enum

class StereoCameraCalibrationParameters: public StereoCameraParameters	{
	private:
		const STEREOMODE cameraMode;
		std::string cameraParamsFilePath;
		double projectionError;

	protected:
		
	public:
		// L can also means Top
		// R can also means Bottom
		// if it is under Top Bottom Setup
		cv::Mat cameraMatrix1;
		cv::Mat cameraMatrix2;
		cv::Mat distortionVector1;
		cv::Mat distortionVector2;
		cv::Mat rotationMatrix;
		cv::Mat translationVector;
		cv::Mat essentialMatrix;
		cv::Mat fundamentalMatrix;
		cv::Mat qMatrix;

		/*
			Constructor/Destructor
		*/
		StereoCameraCalibrationParameters(STEREOMODE sMode);
		~StereoCameraCalibrationParameters();

		/*
			Getter/Setter
		*/
		STEREOMODE getCameraMode();
		int getClassType();
		void setCalibrationFP(std::string cpfp);

		/*
			Non-Template Class Methods
		*/
		bool readParams();
		void printParams(std::ostream& out);

		/*
			Template Class Mathods
		*/
};

#endif

