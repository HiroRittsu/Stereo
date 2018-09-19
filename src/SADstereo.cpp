#include <stdio.h>
#include <iostream>
#include "Stereo.hpp"

int main(int argc, char const *argv[]) {

	Stereo stereo;

	double min, max;

	cv::Mat left_gray, right_gray;
	cv::Mat result1, result2;

	cv::Mat imleft =  cv::imread("/home/migly/catkin_ws/src/stereo/src/left.png");
	cv::Mat imright =  cv::imread("/home/migly/catkin_ws/src/stereo/src/right.png");

	cvtColor(imleft, left_gray, CV_RGB2GRAY);
	cvtColor(imright, right_gray, CV_RGB2GRAY);

	result1 = stereo.getDistanceMap(left_gray, right_gray, &max, &min);

	result1.convertTo(result2, CV_8UC1, 255 / (max - min) , -255 * min / (max - min));

	cv::imshow("result", result2);

	cv::waitKey();

	return 0;
}