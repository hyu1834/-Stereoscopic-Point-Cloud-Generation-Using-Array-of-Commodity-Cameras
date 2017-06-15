#include "stereo_camera_sgbm_parameters.h"

/*
	Constructor/Destructor
*/
StereoCameraSGBMParameters::StereoCameraSGBMParameters(std::string bmFilePath, IMAGE_RESOLUTION imr) : 
													   bmParamsFilePath(bmFilePath),
													   StereoCameraParameters()	{
	params.preFilterCap = 0;
	params.SADWindowSize = 0;
	params.minDisparity = 0;
	params.numberOfDisparities = 0;
	params.uniquenessRatio = 0;
	params.speckleWindowSize = 0;
	params.speckleRange = 0;
	params.disp12MaxDiff = 0;
	params.fullDP = 0;
}

StereoCameraSGBMParameters::StereoCameraSGBMParameters(int preFilterCap, int SADWindowSize, int minDisparity, 
													   int numberOfDisparities, int uniquenessRatio, 
													   int speckleWindowSize, int speckleRange, int disp12MaxDiff,
                                                       int fullDP)	{
	params.preFilterCap = preFilterCap;
	params.SADWindowSize = SADWindowSize;
	params.minDisparity = minDisparity;
	params.numberOfDisparities = numberOfDisparities;
	params.uniquenessRatio = uniquenessRatio;
	params.speckleWindowSize = speckleWindowSize;
	params.speckleRange = speckleRange;
	params.disp12MaxDiff = disp12MaxDiff;
	params.fullDP = fullDP;
}

//@breif class destructor
StereoCameraSGBMParameters::~StereoCameraSGBMParameters()	{

}

/*
	Getter/Setter
*/
SGBM_Params StereoCameraSGBMParameters::getParams()	{
	return params;
}

int StereoCameraSGBMParameters::getClassType()	{
	return SCSGBMP;
}

void StereoCameraSGBMParameters::setBMPPath(std::string path)	{
	bmParamsFilePath = path;
}

/*
	Non Template Class Method
*/
bool StereoCameraSGBMParameters::storeParams(Json::Value bmValues, SGBM_Params& params)	{
	// Store SGBM
	params.preFilterCap = bmValues["SGBM"]["preFilterCap"].asInt();
	params.SADWindowSize = bmValues["SGBM"]["SADWindowSize"].asInt();
	params.minDisparity = bmValues["SGBM"]["minDisparity"].asInt();
	params.numberOfDisparities = bmValues["SGBM"]["numberOfDisparities"].asInt();
	if(params.numberOfDisparities % 16 != 0)	{
		std::cerr << "ERROR: numberOfDisparities %% 16 must = 0\n";
		return false;
	}
	params.uniquenessRatio = bmValues["SGBM"]["uniquenessRatio"].asInt();
	params.speckleWindowSize = bmValues["SGBM"]["speckleWindowSize"].asInt();
	params.speckleRange = bmValues["SGBM"]["speckleRange"].asInt();
	params.disp12MaxDiff = bmValues["SGBM"]["disp12MaxDiff"].asInt();
	params.fullDP = bmValues["SGBM"]["fullDP"].asInt();

	return true;
}

bool StereoCameraSGBMParameters::readParams(IMAGE_RESOLUTION imageResolution)	{
	std::ifstream bmFp(bmParamsFilePath, std::ifstream::binary);
	if(!bmFp.is_open())	{
		std::cerr<<"Stereo Block Matching Parameters File: " << bmParamsFilePath << 
				   " DOES NOT EXIST/Unable Open\n";
		return false;
	}
	
	bmFp >> bmParams;
	bool readSuccess = true;

	if(imageResolution == IMR_480)	{
		readSuccess = storeParams(bmParams["480"], params);
	}
	else if(imageResolution == IMR_720)	{
		readSuccess = storeParams(bmParams["720"], params);
	}
	else if(imageResolution == IMR_1080)	{
		readSuccess = storeParams(bmParams["1080"], params);
	}
	else if(imageResolution == IMR_5MP)	{
		readSuccess = storeParams(bmParams["5MP"], params);
	}
	else if(imageResolution == IMR_8MP)	{
		readSuccess = storeParams(bmParams["8MP"], params);
	}

	bmFp.close();

	#ifdef DEBUGGING
		printParams(std::cout);
	#endif

	return readSuccess;
}

void StereoCameraSGBMParameters::printParams(std::ostream& out)	{
	out <<"SGBM Parameters: uint8_t\n"<<
		  "\tpreFilterCap: " << params.preFilterCap << "\n" <<
		  "\tSADWindowSize: " << params.SADWindowSize << "\n" <<
		  "\tminDisparity: " << params.minDisparity << "\n" <<
		  "\tnumberOfDisparities: " << params.numberOfDisparities << "\n" <<
		  "\tuniquenessRatio: " << params.uniquenessRatio << "\n" <<
		  "\tspeckleWindowSize: " << params.speckleWindowSize << "\n" <<
		  "\tspeckleRange: " << params.speckleRange << "\n" <<
		  "\tdisp12MaxDiff: " << params.disp12MaxDiff << "\n" <<
		  "\tfullDP: " << params.fullDP << "\n\n";
}

void StereoCameraSGBMParameters::updateParams(int preFilterCap, int SADWindowSize, int minDisparity,
                                              int numberOfDisparities, int uniquenessRatio, int speckleWindowSize,
                                              int speckleRange, int disp12MaxDiff, int fullDP)	{
	params.preFilterCap = preFilterCap;
	params.SADWindowSize = SADWindowSize;
	params.minDisparity = minDisparity;
	params.numberOfDisparities = numberOfDisparities;
	params.uniquenessRatio = uniquenessRatio;
	params.speckleWindowSize = speckleWindowSize;
	params.speckleRange = speckleRange;
	params.disp12MaxDiff = disp12MaxDiff;
	params.fullDP = fullDP;
}




/*
	Template Class Method
*/
