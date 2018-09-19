#include "ros/ros.h"
#include <iostream>
#include <stdio.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "CalcTimer.hpp"

cv::Mat imLeft_data, imRight_data;

CalcTimer timer;

cv::Mat noise_filter(cv::Mat input) {

	cv::Mat result = input.clone();

	cv::edgePreservingFilter(input, result);

	//cv::imshow("noise", result);

	//cv::waitKey(1);

	return result;

}

void imageLeft(const sensor_msgs::Image::ConstPtr& msg) {

	cv_bridge::CvImagePtr cv_ptr;

	try {
		// ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::imshow("Display window Left", cv_ptr->image );
	cv::waitKey(1);

	imLeft_data = cv_ptr->image;

}

void imageRight(const sensor_msgs::Image::ConstPtr& msg) {

	cv_bridge::CvImagePtr cv_ptr;

	try {
		// ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::imshow("Display window Right", cv_ptr->image );
	cv::waitKey(1);

	imRight_data = cv_ptr->image;

}

void stereo() {


	double min, max;

	//const cv::Mat imLeft_data = cv::imread("/home/migly/Desktop/SuXT483.png", cv::IMREAD_GRAYSCALE);
	//const cv::Mat imRight_data = cv::imread("/home/migly/Desktop/Yeuna9x.png", cv::IMREAD_GRAYSCALE);

	cv::Mat dis_data;
	cv::Mat dis_map;

	timer.startTime();

	cv::Ptr<cv::StereoSGBM> sbm = cv::StereoSGBM::create(0, 16, 10);

	sbm->setSpeckleWindowSize(200);
	sbm->setSpeckleRange(1);

	//sbm->compute(image1, image2, dis_data);
	if (!imLeft_data.empty() && !imRight_data.empty()) {

		sbm->compute((imLeft_data), (imRight_data), dis_data);
		minMaxLoc(dis_data, &min, &max);

		printf("%f\n", timer.getTime() );

		//dis_data.convertTo(dis_map, CV_8UC1, 255 / (max - min) , -255 * min / (max - min));
		dis_data.convertTo(dis_map, CV_8UC1);
		imshow("window1", dis_map);

		cv::waitKey(1);

	}

}




int main(int argc, char **argv) {

	ros::init(argc, argv, "image_kinect");

	ros::NodeHandle n;

	ros::Subscriber left_sub = n.subscribe("/stereo/left/image_rect_color", 1, imageLeft);
	ros::Subscriber right_sub = n.subscribe("/stereo/right/image_rect_color", 1, imageRight);

	while (ros::ok()) {
		ros::spinOnce();
		stereo();
	}

	return 0;
}