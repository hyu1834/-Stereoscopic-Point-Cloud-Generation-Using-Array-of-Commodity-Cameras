#include "stereo_camera_parameters.h"

/*
	Constructor/Destructor
*/

StereoCameraParameters::StereoCameraParameters()	{

}

StereoCameraParameters::~StereoCameraParameters()	{

}

/*
	Getter/Setter
*/
// STEREOMODE StereoCameraParameters::getCameraMode()	{
// 	return cameraMode;
// }

// IMAGE_RESOLUTION StereoCameraParameters::getImageResolution()	{
// 	return imageResolution;
// }

int StereoCameraParameters::getClassType()	{
	return SCP;
}

/*
	Non Template Class Method
*/


bool StereoCameraParameters::readParams()	{
    return false;
}

void StereoCameraParameters::printParams(std::ostream& out)	{
	
}
