#include <opencv2/opencv.hpp>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "CalcTimer.hpp"
#include "Stereo.hpp"
#include "ObjectStereo.hpp"

cv::Mat imleft, imright;
Stereo stereo;
CalcTimer timer;

//ROS系処理
void imageLeft(const sensor_msgs::Image::ConstPtr& msg) {

	cv::Mat image;

	try {
		// ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
		image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	//cv::resize(image, image, cv::Size(), 0.5, 0.5);

	imleft = image.clone();

	cv::imshow("left", image);
	cv::waitKey(1);

}

void imageRight(const sensor_msgs::Image::ConstPtr& msg) {

	cv::Mat image;

	try {
		// ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
		image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	//cv::resize(image, image, cv::Size(), 0.5, 0.5);

	imright = image.clone();

	cv::imshow("right", image);
	cv::waitKey(1);

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "colortracking");

	ros::NodeHandle n;

	ros::Subscriber left_sub = n.subscribe("/stereo/left/image_rect_color", 1, imageLeft);
	ros::Subscriber right_sub = n.subscribe("/stereo/right/image_rect_color", 1, imageRight);

	while (ros::ok()) {

		ros::spinOnce();

		if (!imleft.empty() && !imright.empty()) {

			double min, max;

			cv::Mat left_gray, right_gray;

			cv::resize(imleft, imleft, cv::Size(), 0.5, 0.5);
			cv::resize(imright, imright, cv::Size(), 0.5, 0.5);

			cvtColor(imleft, left_gray, CV_RGB2GRAY);
			cvtColor(imright, right_gray, CV_RGB2GRAY);

			cv::Mat result1, result2;

			result1 = stereo.getDistanceMap(left_gray, right_gray, &max, &min);

			result1.convertTo(result2, CV_8UC1, 255 / (max - min) , -255 * min / (max - min));

			cv::imshow("result", result2);

			cv::waitKey(1);
		}
	}

	return 0;
}