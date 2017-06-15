#ifndef OPENCV_UTILS_H
	#define OPENCV_UTILS_H
// Standard Libraries
#include <iostream>
#include <fstream>
#include <cstring>
#include <string>
#include <vector>


// 3rd Parties Libraries
#include <cv.h>
#include <highgui.h>

// Local Libraries

//MACRO

// Struct for 2D point comparsion
struct PointComp	{
	bool operator() (const cv::Point2d& lhs, const cv::Point2d& rhs) const 	{
		if(lhs.x != rhs.x)	{
			return lhs.x < rhs.x;
		}
		else	{
			return lhs.y < rhs.y;
		}
	}
	bool operator() (const cv::Point2f& lhs, const cv::Point2f& rhs) const 	{
		if(lhs.x != rhs.x)	{
			return lhs.x < rhs.x;
		}
		else	{
			return lhs.y < rhs.y;
		}
	}
	bool operator() (const cv::Point2i& lhs, const cv::Point2i& rhs) const 	{
		if(lhs.x != rhs.x)	{
			return lhs.x < rhs.x;
		}
		else	{
			return lhs.y < rhs.y;
		}
	}
};

/*
	General Functions
*/
void cvVersion();
std::string getCVVersion();

/*
	IplImage C Functions
*/
IplImage* loadIplImage(std::string filename, int option = 0);
CvSize getImageSize(IplImage* image);
void showImage(std::string windowTitle, IplImage* image);
bool isEqualResolution(IplImage* image1, IplImage* image2);

/*
	cv::Mat C++ Functions
*/
cv::Mat loadMatImage(std::string filepath, int option = 0);
cv::Mat loadRawBGRImage(std::string filepath);
cv::Size getImageSize(cv::Mat* image);
cv::Size getImageSize(cv::Mat image);
void changeChannel(cv::Mat src, cv::Mat& dest, int color);
void showImage(std::string windowTitle, cv::Mat image);
void saveImage(std::string filename, const cv::Mat image);
bool isEqualResolution(cv::Mat image1, cv::Mat image2);
cv::Mat overLayImages(cv::Mat image1, cv::Mat image2, double alpha);
cv::Mat readFileStorage(std::string filename, std::string name);
void writeFileStorage(std::string filename, std::string name, cv::Mat item);
int getCVMatType(cv::Mat mat);
int getCVMatChannel(cv::Mat mat);
int getCVMatEnum(cv::Mat mat);

#endif