#include "opencv_utils.h"

//MACRO

/*
	General Functions
*/
void cvVersion()	{
	std::cout << "OpenCV version: " << CV_VERSION << std::endl;
	// cout << "Major version : " << CV_MAJOR_VERSION << endl;
	// cout << "Minor version : " << CV_MINOR_VERSION << endl;
	// cout << "Subminor version : " << CV_SUBMINOR_VERSION << endl;
}

std::string getCVVersion()	{
	return CV_VERSION;
}

/*
	IplImage C OpenCV function
*/
IplImage* loadIplImage(std::string filename, int option)	{
	return cvLoadImage(filename.c_str(), option);
}

CvSize getImageSize(IplImage* image)	{
	return cvGetSize(image);
}

void showImage(std::string windowTitle, IplImage* image)	{
	cvNamedWindow(windowTitle.c_str(), 1);
	cvShowImage(windowTitle.c_str(), image);
	cvWaitKey(0);
}

bool isEqualResolution(IplImage* image1, IplImage* image2)  {
	cv::Mat mat1 = cv::cvarrToMat(image1);
	cv::Mat mat2 = cv::cvarrToMat(image2);
	if(mat1.size() != mat2.size())  {
		return false;
	}

	return true;
}

IplImage* constructSideBySideImage(IplImage* image1, IplImage* image2, int height, int width) {

}

/*
	cv::Mat C++ OpenCV Function
*/
cv::Mat loadMatImage(std::string filepath, int option)	{
	return cv::imread(filepath.c_str(), option);
}

cv::Mat loadRawBGRImage(std::string filepath)	{
	cv::Mat image;
	return image;
}

cv::Size getImageSize(cv::Mat* image) {
	return image->size();
}

cv::Size getImageSize(cv::Mat image) {
	return image.size();
}

void changeChannel(cv::Mat src, cv::Mat& dest, int color)	{
	cv::cvtColor(src, dest, color);
}


void showImage(std::string windowTitle, cv::Mat image) {
	cv::imshow(windowTitle.c_str(), image);
	cv::waitKey(0);
}

void saveImage(std::string filename, const cv::Mat image)	{
	imwrite(filename.c_str(), image);
}

bool isEqualResolution(cv::Mat mat1, cv::Mat mat2)	{
	if(mat1.size() != mat2.size())	{
		return false;
	}

	return true;
}

cv::Mat overLayImages(cv::Mat image1, cv::Mat image2, double alpha)	{
	cv::Mat dst;
	double beta = 1.0 - alpha;

	addWeighted(image1, alpha, image2, beta, 0.0, dst);

	return dst;
}

cv::Mat readFileStorage(std::string filename, std::string name)	{
	cv::Mat input;
	cv::FileStorage fs(filename.c_str(), cv::FileStorage::READ);
	fs[name] >> input;
	fs.release();
	return input;
}
void writeFileStorage(std::string filename, std::string name, cv::Mat item)	{
	cv::FileStorage fs(filename.c_str(), cv::FileStorage::WRITE);
	fs << name << item;
	fs.release();
}

int getCVMatType(cv::Mat mat)	{
	return (int)(mat.type() & CV_MAT_DEPTH_MASK);
}

int getCVMatBits(cv::Mat mat)	{
	return (int)(1 + (mat.type() >> CV_CN_SHIFT));
}

int getCVMatEnum(cv::Mat mat)	{
	int type = (int)(mat.type() & CV_MAT_DEPTH_MASK);
	int channel = (int)(1 + (mat.type() >> CV_CN_SHIFT));

	switch(type)	{
		case 0:
			if(channel == 1)	return CV_8UC1;
			else if(channel == 2)	return CV_8UC2;
			else	return CV_8UC3;
		case 1:
			if(channel == 1)	return CV_8SC1;
			else if(channel == 2)	return CV_8SC2;
			else	return CV_8SC3;
		case 2:
			if(channel == 1)	return CV_16UC1;
			else if(channel == 2)	return CV_16UC2;
			else	return CV_16UC3;
		case 3:
			if(channel == 1)	return CV_16SC1;
			else if(channel == 2)	return CV_16SC2;
			else	return CV_16SC3;
		case 4:
			if(channel == 1)	return CV_32SC1;
			else if(channel == 2)	return CV_32SC2;
			else	return CV_32SC3;
		case 5:
			if(channel == 1)	return CV_32FC1;
			else if(channel == 2)	return CV_32FC2;
			else	return CV_32FC3;
		case 6:
			if(channel == 1)	return CV_64FC1;
			else if(channel == 2)	return CV_64FC2;
			else	return CV_64FC3;
	}
}
#include "opencv_utils.h"