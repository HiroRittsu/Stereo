#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#define T 10 //カメラの距離
#define f 8 //焦点距離

cv::Point_<double> resultPoint[2];

//赤い物体の重心を計算
cv::Point_<double> TergetPoint(cv::Mat image) {

	cv::Mat image_hsv;
	cv::Point_<double> point2f;
	double centroid_x = 0;
	double centroid_y = 0;

	cv::cvtColor(image, image_hsv, cv::COLOR_BGR2HSV);

	cv::Mat color_result = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
	cv::Mat contour_result = cv::Mat::zeros(image.rows, image.cols, CV_8UC3);

	//しきい値の設定
	auto lower = cv::Scalar(150, 128, 0);
	auto upper = cv::Scalar(179, 255, 255);

	cv::inRange(image_hsv, lower, upper, color_result);

	std::vector<std::vector<cv::Point>> contours;

	//輪郭の抽出
	cv::findContours(color_result, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	if (contours.size() != 0) {

		//輪郭の面積
		double area;
		double area_max = 0;
		int max_index = 0;

		for (unsigned i = 0; i < contours.size(); ++i) {
			area = cv::contourArea(contours.at(i));
			if (area_max < area) {
				area_max = area;
				max_index = i;
			}
		}

		if (area_max > 150) {

			cv::drawContours(image, contours, max_index, cv::Scalar(0, 0, 255), 3);

			int count = 0;
			double sum_x = 0;
			double sum_y = 0;

			//重心取得
			for (unsigned i = 0; i < contours.at(max_index).size(); ++i) {

				sum_x += contours.at(max_index).at(i).x;
				sum_y += contours.at(max_index).at(i).y;
				count++;
			}

			if (count == 0) {
				centroid_x = 0;
				centroid_y = 0;
			} else {
				centroid_x = sum_x / count;
				centroid_y = sum_y / count;
			}

		}

	}

	point2f.x = centroid_x;
	point2f.y = centroid_y;

	return point2f;

}

void imageLeft(const sensor_msgs::Image::ConstPtr& msg) {

	cv::Mat image;
	cv::Point_<double> point2f;

	try {
		// ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
		image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	//重心の計算
	point2f = TergetPoint(image);

	resultPoint[0] = point2f;

	cv::Point position(point2f.x, point2f.y);
	cv::Scalar color(0, 255, 0);
	cv::circle(image, position, 3, color, -1);

	cv::imshow("left", image);
	cv::waitKey(1);

}

void imageRight(const sensor_msgs::Image::ConstPtr& msg) {

	cv::Mat image;
	cv::Point_<double> point2f;

	try {
		// ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
		image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	//重心の計算
	point2f = TergetPoint(image);

	resultPoint[1] = point2f;

	cv::Point position(point2f.x, point2f.y);
	cv::Scalar color(0, 255, 0);
	cv::circle(image, position, 3, color, -1);

	cv::imshow("right", image);
	cv::waitKey(1);

}

double calcDistance(cv::Point_<double> point[]) {

	return f * T / (point[0].x - point[1].x);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "colortracking");

	ros::NodeHandle n;

	ros::Subscriber left = n.subscribe("/stereo/left/image_raw", 1000, imageLeft);
	ros::Subscriber right = n.subscribe("/stereo/right/image_raw", 1000, imageRight);

	while (ros::ok()) {

		ros::spinOnce();
		printf("%f\n", calcDistance(resultPoint));

	}

	return 0;
}