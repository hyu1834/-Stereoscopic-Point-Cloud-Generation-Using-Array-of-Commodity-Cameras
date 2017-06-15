#include "stereo_camera_sbm_parameters.h"

/*
	Constructor/Destructor
*/
StereoCameraSBMParameters::StereoCameraSBMParameters(std::string bmFilePath, IMAGE_RESOLUTION imr) : 
													   bmParamsFilePath(bmFilePath), imageResolution(imr),
													   StereoCameraParameters()	{

}

//@breif class destructor
StereoCameraSBMParameters::~StereoCameraSBMParameters()	{

}

/*
	Getter/Setter
*/
SBM_Params StereoCameraSBMParameters::getParams()	{
	return params;
}

IMAGE_RESOLUTION StereoCameraSBMParameters::getImageResolution()	{
	return imageResolution;
}


int StereoCameraSBMParameters::getClassType()	{
	return SCSBMP;
}

/*
	Non Template Class Method
*/
bool StereoCameraSBMParameters::storeParams(Json::Value bmValues, SBM_Params& params)	{
	// // Store SBM
	// params.preFilterCap = bmValues["SBM"]["preFilterCap"].asInt();
	// params.SADWindowSize = bmValues["SBM"]["SADWindowSize"].asInt();
	// params.minDisparity = bmValues["SBM"]["minDisparity"].asInt();
	// params.numberOfDisparities = bmValues["SBM"]["numberOfDisparities"].asInt();
	// if(params.numberOfDisparities % 16 != 0)	{
	// 	std::cerr << "ERROR: numberOfDisparities %% 16 must = 0\n";
	// 	return false;
	// }
	// params.uniquenessRatio = bmValues["SBM"]["uniquenessRatio"].asInt();
	// params.speckleWindowSize = bmValues["SBM"]["speckleWindowSize"].asInt();
	// params.speckleRange = bmValues["SBM"]["speckleRange"].asInt();
	// params.disp12MaxDiff = bmValues["SBM"]["disp12MaxDiff"].asInt();
	// params.fullDP = bmValues["SBM"]["fullDP"].asInt();

	return true;
}

bool StereoCameraSBMParameters::readParams()	{
	// std::ifstream bmFp(bmParamsFilePath, std::ifstream::binary);
	// if(!bmFp.is_open())	{
	// 	std::cerr<<"Stereo Block Matching Parameters File: " << bmParamsFilePath << 
	// 			   " DOES NOT EXIST/Unable Open\n";
	// 	return false;
	// }
	
	// bmFp >> bmParams;
	bool readSuccess = true;

	// if(imageResolution == IMR_480)	{
	// 	readSuccess = storeParams(bmParams["480"], params);
	// }
	// else if(imageResolution == IMR_720)	{
	// 	readSuccess = storeParams(bmParams["720"], params);
	// }
	// else if(imageResolution == IMR_1080)	{
	// 	readSuccess = storeParams(bmParams["1080"], params);
	// }
	// else if(imageResolution == IMR_5MP)	{
	// 	readSuccess = storeParams(bmParams["5MP"], params);
	// }
	// else if(imageResolution == IMR_8MP)	{
	// 	readSuccess = storeParams(bmParams["8MP"], params);
	// }

	// bmFp.close();

	// #ifdef DEBUGGING
	// 	printParams();
	// #endif

	return readSuccess;
}

void StereoCameraSBMParameters::printParams(std::ostream& out)	{
	// out <<"SBM Parameters: uint8_t\n"<<
		  // "\tpreFilterCap: " << params.preFilterCap << "\n" <<
		  // "\tSADWindowSize: " << params.SADWindowSize << "\n" <<
		  // "\tminDisparity: " << params.minDisparity << "\n" <<
		  // "\tnumberOfDisparities: " << params.numberOfDisparities << "\n" <<
		  // "\tuniquenessRatio: " << params.uniquenessRatio << "\n" <<
		  // "\tspeckleWindowSize: " << params.speckleWindowSize << "\n" <<
		  // "\tspeckleRange: " << params.speckleRange << "\n" <<
		  // "\tdisp12MaxDiff: " << params.disp12MaxDiff << "\n" <<
		  // "\tfullDP: " << params.fullDP << "\n\n";
}


/*
	Template Class Method
*/