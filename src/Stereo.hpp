#include <stdio.h>
#include <iostream>
#include <math.h>
#include "CalcTimer.hpp"
#include "opencv2/opencv.hpp"

#ifndef Stereo_HPP
#define Stereo_HPP

CalcTimer timer_hpp;

using namespace std;

class Stereo {
public:
	Stereo();
	~Stereo();

	cv::Mat getDistanceMap(cv::Mat left, cv::Mat right, double *max, double *min);

	double getSimilarity(cv::Mat image1, cv::Point image1_point, cv::Mat image2, cv::Point image2_point, int size[]);
	cv::Point matchPoint(cv::Mat left, cv::Mat right, cv::Point target);
	double calcDistance(double xl, double xr);
	void getPointDistance();

private:
	int TEMP_H;
	int TEMP_W;
	double T;
	double f;
};

Stereo::Stereo() {

	this->TEMP_H = 3;
	this->TEMP_W = 3;
	this->T = 10;
	this->f = 7.5;
}

Stereo::~Stereo() {

}

//SAD類似度計算
double Stereo::getSimilarity(cv::Mat image1, cv::Point image1_point, cv::Mat image2, cv::Point image2_point, int size[]) {

	double sum = 0;

	for (int y = 0; y < size[0]; ++y) {
		
		for (int x = 0; x < size[1]; ++x) {

			sum += abs(image1.at<unsigned char>(image1_point.y + y, image1_point.x + x) - image2.at<unsigned char>(image2_point.y + y, image2_point.x + x));
		}
	}

	return sum;
}


cv::Point Stereo::matchPoint(cv::Mat left, cv::Mat right, cv::Point target) {

	cv::Point result;
	double sim_min = std::numeric_limits<double>::max();
	double sim;
	int x_min = -1;
	int size[] = {TEMP_H, TEMP_W};

	//将来的にはエピポーラ線も考慮する
	for (int x = 0; x <= target.x - ((TEMP_W - 1) / 2); x += 3) { //rightimage

		//類似度計算
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

double Stereo::calcDistance(double xl, double xr) {

	if (xl < xr) return -1;

	return f * T / (xl - xr);
}

cv::Mat Stereo::getDistanceMap(cv::Mat left, cv::Mat right, double *max, double *min) {

	cv::Mat map(left.size[0], left.size[1], CV_MAKETYPE(CV_64F, 1));
	cv::Point result;
	double result_distance;

	*max = 0;
	*min = std::numeric_limits<double>::max();

	//オリジナル画像すべての画素を参照
	for (int y = (TEMP_H - 1) / 2; y < left.size[0] - ((TEMP_H - 1) / 2); ++y) {

		//timer_hpp.startTime();

		for (int x = (TEMP_W - 1) / 2; x < left.size[1] - ((TEMP_W - 1) / 2); ++x) {

			//オリジナル画像のある画素と一致する場所を計算
			result = this->matchPoint(left, right, cv::Point(x, y));

			if (result.x != -1) {

				result_distance = this->calcDistance(x, result.x);

				if (result_distance <= 10) {
					map.at<double>(y, x) = result_distance;
					if (*max < result_distance) *max = result_distance;
					if (*min > result_distance) *min = result_distance;
				}
			}
		}
	}
	return map;
}

#endif