#include "stereo_camera_calibration_parameters.h"


/*
	Constructor/Destructor
*/
StereoCameraCalibrationParameters::StereoCameraCalibrationParameters(STEREOMODE sMode) :
																	cameraParamsFilePath(""), cameraMode(sMode),
																	StereoCameraParameters()	{

}

StereoCameraCalibrationParameters::~StereoCameraCalibrationParameters()	{

}

/*
	Getter/Setter
*/
STEREOMODE StereoCameraCalibrationParameters::getCameraMode()	{
	return cameraMode;
}

int StereoCameraCalibrationParameters::getClassType()	{
	return SCCP;
}

void StereoCameraCalibrationParameters::setCalibrationFP(std::string cpfp)	{
	cameraParamsFilePath = cpfp;
}

/*
	Non-Template Class Methods
*/
bool StereoCameraCalibrationParameters::readParams()	{
	cv::FileStorage fs(cameraParamsFilePath, cv::FileStorage::READ);
	if(!fs.isOpened()) {
		std::cerr<<"Stereo Parameters File: "<<cameraParamsFilePath<<" DOES NOT EXIST/Unable Open\n";
		return false;
	}

	// read camera Matrix L
	fs["cameraMatrix1"] >> cameraMatrix1;
	if(cameraMatrix1.cols != 3 || cameraMatrix1.rows != 3)	{
		std::cerr<<"ERROR: cameraMatrix1 not 3x3 Matrix\n";
		return false;
	}

	// read distortion matrix L
	fs["distortionVector1"] >> distortionVector1;
	if(distortionVector1.cols != 5 || distortionVector1.rows != 1)	{
		std::cerr<<"ERROR: distortionVector1 not 5 element vector\n";
		return false;
	}

	// read camera Matrix R
	fs["cameraMatrix2"] >> cameraMatrix2;
	if(cameraMatrix2.cols != 3 || cameraMatrix2.rows != 3)	{
		std::cerr<<"ERROR: cameraMatrix2 not 3x3 Matrix\n";
		return false;
	}
	
	// read distortion matrix R
	fs["distortionVector2"] >> distortionVector2;
	if(distortionVector2.cols != 5 || distortionVector2.rows != 1)	{
		std::cerr<<"ERROR: distortionVector2 not 5 element vector\n";
		return false;
	}

	// read rotation matrix from stereo calibration
	fs["rotationMatrix"] >> rotationMatrix;
	if(rotationMatrix.cols != 3 || rotationMatrix.rows != 3)	{
		std::cerr<<"ERROR: rotationMatrix not 3x3 Matrix\n";
		return false;
	}

	// read translation Vector from stereo calibration
	fs["translationVector"] >> translationVector;
	if(translationVector.cols != 1 || translationVector.rows != 3)	{
		std::cerr<<"ERROR: translationVector not 3 element vector\n";
		return false;
	}

	// read essential Matrix from stereo calibration
	fs["essentialMatrix"] >> essentialMatrix;
	if(essentialMatrix.cols != 3 || essentialMatrix.rows != 3)	{
		std::cerr<<"ERROR: essentialMatrix not 3x3 Matrix\n";
		return false;
	}

	// read rotation matrix from stereo calibration
	fs["fundamentalMatrix"] >> fundamentalMatrix;
	if(fundamentalMatrix.cols != 3 || fundamentalMatrix.rows != 3)	{
		std::cerr<<"ERROR: fundamentalMatrix not 3x3 Matrix\n";
		return false;
	}

	fs.release();

	#ifdef DEBUGGING
		printParams(std::cout);
	#endif

	return true;
}

void StereoCameraCalibrationParameters::printParams(std::ostream& out)	{
	out << "Camera Matrix 1:\n" << cameraMatrix1 << "\n\n" <<
		   "Distortion Vector 1:\n" << distortionVector1 << "\n\n" <<
		   "Camera Matrix 2:\n" << cameraMatrix2 << "\n\n" <<
		   "Distortion Vector 2:\n" << distortionVector2 << "\n\n" <<
		   "Rotation Matrix:\n" << rotationMatrix << "\n\n" <<
		   "Translation Vector:\n" << translationVector << "\n\n" <<
		   "Essential Matrix:\n" << essentialMatrix << "\n\n" <<
		   "Fundamental Matrix:\n" << fundamentalMatrix << "\n\n";
}

/*
	Template Class Mathods
*/