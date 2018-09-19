#include <stdio.h>
#include <iostream>
#include <math.h>
#include "opencv2/opencv.hpp"

#ifndef   ObjectStereo_HPP
#define   ObjectStereo_HPP


using namespace std;

class ObjectStereo {
public:
	ObjectStereo();
	~ObjectStereo();

	cv::Mat getDistanceMap(cv::Mat left, cv::Point left_object[], cv::Mat right, cv::Point right_object[], double * max, double * min);

	cv::Mat joinImage(cv::Mat image1, cv::Mat image2);

	double getSimilarity(cv::Mat image1, cv::Point image1_point, cv::Mat image2, cv::Point image2_point, int size[]);
	cv::Point matchPoint(cv::Mat left, cv::Mat right, cv::Point right_object[], cv::Point target);
	double calcDistance(double xl, double xr);
	void getPointDistance();

private:
	int TEMP_H;
	int TEMP_W;
	double T;
	double f;


};

ObjectStereo::ObjectStereo() {

	this->TEMP_H = 3;
	this->TEMP_W = 3;
	this->T = 10;
	this->f = 7.5;

}

ObjectStereo::~ObjectStereo() {

}

cv::Mat ObjectStereo::joinImage(cv::Mat image1, cv::Mat image2) {

	cv::Mat result;

	cv::hconcat(image1, image2, result);

	return result;
}

double ObjectStereo::getSimilarity(cv::Mat image1, cv::Point image1_point, cv::Mat image2, cv::Point image2_point, int size[]) {

	double sum = 0;

	for (int y = 0; y < size[0]; ++y) {
		for (int x = 0; x < size[1]; ++x) {

			sum += abs(image1.at<unsigned char>(image1_point.y + y, image1_point.x + x) - image2.at<unsigned char>(image2_point.y + y, image2_point.x + x));

		}

	}

	return sum;
}


cv::Point ObjectStereo::matchPoint(cv::Mat left, cv::Mat right, cv::Point right_object[], cv::Point target) {

	cv::Point result;
	double sim_min = std::numeric_limits<double>::max();
	double sim;
	int x_min = -1;
	int size[] = {TEMP_H, TEMP_W};

	//将来的にはエピポーラ線も考慮する
	for (int x = right_object[0].x + ((TEMP_W - 1) / 2); x <= right_object[1].x - ((TEMP_W - 1) / 2); x += 1) {

		sim = this->getSimilarity(left, cv::Point(target.x - ((TEMP_W - 1) / 2), target.y - ((TEMP_H - 1) / 2)), right, cv::Point(x, target.y - ((TEMP_H - 1) / 2)), size);

		if (sim_min > sim) {
			sim_min = sim;
			x_min = x;
		}
	}

	result.x = x_min;
	result.y = target.y;

	return result;
}

double ObjectStereo::calcDistance(double xl, double xr) {

	if (xl < xr) return -1;

	return f * T / (xl - xr);
}

cv::Mat ObjectStereo::getDistanceMap(cv::Mat left, cv::Point left_object[], cv::Mat right, cv::Point right_object[], double * max, double * min) {

	//cv::Mat map(left.size[0], left.size[1], CV_MAKETYPE(CV_64F, 1));
	cv::Mat map(left_object[1].y - left_object[0].y, left_object[1].x - left_object[0].x, CV_MAKETYPE(CV_64F, 1));
	cv::Point result;
	double result_distance;

	*max = 0;
	*min = std::numeric_limits<double>::max();

	for (int y = left_object[0].y + (TEMP_H - 1) / 2; y < left_object[1].y - ((TEMP_H - 1) / 2); ++y) {

		for (int x = left_object[0].x + (TEMP_W - 1) / 2; x < left_object[1].x - ((TEMP_W - 1) / 2); ++x) {
			//対応点の取得
			result = this->matchPoint(left, right, right_object, cv::Point(x, y));

			if (result.x != -1) {

				result_distance = this->calcDistance(x, result.x);

				if (result_distance <= 10) {
					map.at<double>(y - left_object[0].y, x - left_object[0].x) = result_distance;
					if (*max < result_distance) *max = result_distance;
					if (*min > result_distance) *min = result_distance;
				}

			}

		}

	}

	return map;
}

#endif