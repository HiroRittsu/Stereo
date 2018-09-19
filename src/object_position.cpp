#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "Stereo.hpp"
#include "ObjectStereo.hpp"

cv::Mat imleft, imright;

Stereo stereo;
ObjectStereo objectstereo;

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

	//ros::Subscriber left = n.subscribe("/camera_left/image_raw", 1000, calc);
	//ros::Subscriber left_sub = n.subscribe("/stereo/left/image_rect_color", 1, imageLeft);
	//ros::Subscriber right_sub = n.subscribe("/stereo/right/image_rect_color", 1, imageRight);


	cv::Point left_object[2];
	cv::Point right_object[2];
	//ros::Subscriber objectdata = n.subscribe("/darknet_ros/bounding_boxes", 1, objectList);

	while (ros::ok()) {

		ros::spinOnce();

		cv::Mat imleft =  cv::imread("/home/migly/catkin_ws/src/stereo/src/left.png");
		cv::Mat imright =  cv::imread("/home/migly/catkin_ws/src/stereo/src/right.png");

		cv::imshow("left", imleft);
		cv::imshow("right", imright);
		cv::waitKey(1);

		//cv::Mat imleft =  cv::imread("/home/migly/catkin_ws/src/stereo/src/ballerina_left.jpg");
		//cv::Mat imright =  cv::imread("/home/migly/catkin_ws/src/stereo/src/ballerina_right.jpg");

		if (!imleft.empty() && !imright.empty()) {
			
			double min, max;

			cv::Mat left_gray, right_gray;

			//cv::resize(imleft, imleft, cv::Size(), 0.5, 0.5);
			//cv::resize(imright, imright, cv::Size(), 0.5, 0.5);

			cvtColor(imleft, left_gray, CV_RGB2GRAY);
			cvtColor(imright, right_gray, CV_RGB2GRAY);

			cv::Mat result1, result2;

			left_object[0].x = 90;
			left_object[0].y = 130;
			left_object[1].x = 200;
			left_object[1].y = 250;

			right_object[0].x = 100;
			right_object[0].y = 120;
			right_object[1].x = 200;
			right_object[1].y = 250;

			cv::Mat left_image(imleft, cv::Rect(left_object[0].x, left_object[0].y, left_object[1].x - left_object[0].x, left_object[1].y - left_object[0].y));

			result1 = objectstereo.getDistanceMap(left_gray, left_object, right_gray, right_object, &max, &min);

			//result1 = stereo.getDistanceMap(left_gray, right_gray, &max, &min);

			result1.convertTo(result2, CV_8UC1);

			for (int y = 0; y < result2.size[0]; ++y) {
				for (int x = 0; x < result2.size[1]; ++x) {
					//		if(result2.at<typename _Tp>(int i0, int i1))
				}
			}

			//cv::GaussianBlur(result2, result2, cv::Size(5, 5), 5, 5);

			//cv::Canny(result2, result2, 50, 200);

			//輪郭の抽出
			std::vector<std::vector<cv::Point>> contours;

			cv::findContours(result2, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

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

			cv::drawContours(left_image, contours, max_index, cv::Scalar(255, 255, 255), 1);

			int count = 0;
			double centroid_x, centroid_y;
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

			cv::Point position(centroid_x, centroid_y);
			cv::Scalar color(0, 255, 0);
			cv::circle(left_image, position, 3, color, -1);

			cv::imshow("minileft", left_image);
			cv::imshow("result", result1);

			cv::waitKey(1);
		}


	}
	return 0;

}