#include "stereo_camera_utils.h"

/*
	IplImage C Functions
*/


/*
	cv::Mat C++ Functions
*/
cv::Mat* stitchTwoImages(cv::Mat* image1, cv::Mat* image2, cv::Size imageSize, STEREOMODE mode) {
	cv::Mat* stitchedImage;
	cv::Mat temp;
	if(mode == SIDE_BY_SIDE)	{
		stitchedImage = new cv::Mat(imageSize.height, imageSize.width * 2, CV_8UC3);
		// fill in alined rectify image into "stitchedImage container"
		// alined image will be in BGR scale
		// change color from gray to bgr
		cv::cvtColor(*image1, temp, CV_GRAY2BGR);
		for(int i = 0; i < imageSize.width; i++)	{
			temp.col(i).copyTo(stitchedImage->col(i));
		}
		// change color from gray to bgr
		cv::cvtColor(*image2, temp, CV_GRAY2BGR);
		for(int i = imageSize.width; i < imageSize.width * 2; i++)  {
			temp.col(i - imageSize.width).copyTo(stitchedImage->col(i));
		}
		// draw the green line
		for(int j = 0; j < imageSize.height; j += 16)
			cv::line(*stitchedImage, cv::Point(0, j), cv::Point(imageSize.width * 2,j), CV_RGB(0, 255, 0));
	}
	else	{
		stitchedImage = new cv::Mat(imageSize.height * 2, imageSize.width, CV_8UC3);
		// fill in alined rectify image into "stitchedImage container"
		// alined image will be in BGR scale
		// change color from gray to bgr
		cv::cvtColor(*image1, temp, CV_GRAY2BGR);
		// fill in pixel
		for(int i = 0; i < imageSize.height; i++)   {
			temp.row(i).copyTo(stitchedImage->row(i));
		}
		// change color from gray to bgr
		cv::cvtColor(*image2, temp, CV_GRAY2BGR);
		// fill in pixel
		for(int i = imageSize.height; i < imageSize.height * 2; i++)	{
			temp.row(i - imageSize.height).copyTo(stitchedImage->row(i));
		}
		for(int j = 0; j < imageSize.width; j += 16)
			cv::line(*stitchedImage, cv::Point(j, 0), cv::Point(j,imageSize.height * 2), CV_RGB(0, 255, 0));
	}

	return stitchedImage;
}

cv::Mat* stitchTwoImages(cv::Mat image1, cv::Mat image2, cv::Size imageSize, STEREOMODE mode) {
	cv::Mat* stitchedImage;
	cv::Mat temp;
	if(mode == SIDE_BY_SIDE)	{
		stitchedImage = new cv::Mat(imageSize.height, imageSize.width * 2, CV_8UC3);
		// fill in alined rectify image into "stitchedImage container"
		// alined image will be in BGR scale
		// change color from gray to bgr
		cv::cvtColor(image1, temp, CV_GRAY2BGR);
		for(int i = 0; i < imageSize.width; i++)	{
			temp.col(i).copyTo(stitchedImage->col(i));
		}
		// change color from gray to bgr
		cv::cvtColor(image2, temp, CV_GRAY2BGR);
		for(int i = imageSize.width; i < imageSize.width * 2; i++)  {
			temp.col(i - imageSize.width).copyTo(stitchedImage->col(i));
		}
		// draw the green line
		for(int j = 0; j < imageSize.height; j += 16)
			cv::line(*stitchedImage, cv::Point(0, j), cv::Point(imageSize.width * 2,j), CV_RGB(0, 255, 0));
	}
	else	{
		stitchedImage = new cv::Mat(imageSize.height * 2, imageSize.width, CV_8UC3);
		// fill in alined rectify image into "stitchedImage container"
		// alined image will be in BGR scale
		// change color from gray to bgr
		cv::cvtColor(image1, temp, CV_GRAY2BGR);
		// fill in pixel
		for(int i = 0; i < imageSize.height; i++)   {
			temp.row(i).copyTo(stitchedImage->row(i));
		}
		// change color from gray to bgr
		cv::cvtColor(image2, temp, CV_GRAY2BGR);
		// fill in pixel
		for(int i = imageSize.height; i < imageSize.height * 2; i++)	{
			temp.row(i - imageSize.height).copyTo(stitchedImage->row(i));
		}
		for(int j = 0; j < imageSize.width; j += 16)
			cv::line(*stitchedImage, cv::Point(j, 0), cv::Point(j,imageSize.height * 2), CV_RGB(0, 255, 0));
	}

	return stitchedImage;
}