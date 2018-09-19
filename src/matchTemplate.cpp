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
ObjectStereo objectstereo;
CalcTimer timer;

//小さな画像とマッチする座標を求める
cv::Point matchPoint(cv::Mat target, cv::Mat temp) {

	//printf("debug1\n");

	cv::Mat result;

	//テンプレートマッチング
	cv::matchTemplate(target, temp, result, cv::TM_CCOEFF_NORMED);

	cv::Point max_point;

	cv::minMaxLoc(result, 0, 0, 0, &max_point);

	//cv::imshow("window", temp);

	//cv::waitKey(0);


	return max_point;
}

//視差から距離を計算
/*double calcDistance(cv::Mat target, cv::Point terget_point, cv::Mat temp) {

	cv::Point result = matchPoint(target, temp);

	//printf("%d\n", result.x );

	//公式
	return f * T / (result.x - terget_point.x);
}*/

//視差マップを作成
/*cv::Mat createParallaxMap(cv::Mat left, cv::Mat right) {

	cv::Mat map = left.clone();
	cv::Point point;
	double result;

	if (!left.empty() && !right.empty()) {

		//timer.startTime();

		for (int y = (TEMP_H - 1) / 2; y < right.size[0] - (TEMP_H - 1); y += TEMP_H) {

			//帯状に画像切り出し
			cv::Mat right_image(right, cv::Rect(0, y - (TEMP_H - 1) / 2, right.size[1], TEMP_H));

			for (int x = (TEMP_W - 1) / 2; x < left.size[1] - (TEMP_W - 1); x += TEMP_W) {

				//帯状に画像切り出し
				//cv::Mat right_image(right, cv::Rect(0, y - (TEMP_H - 1) / 2, x + (TEMP_W - 1) / 2, TEMP_H));

				//テンプレート画像切り出し
				cv::Mat temp_image(left, cv::Rect(x, y, TEMP_W, TEMP_H));

				point.x = x;
				point.y = y;

				result = calcDistance(right_image, point, temp_image);

				//printf("x:%d distance:%f\n", x, result );

				cv::Point position(x, y);

				cv::Scalar color(0, 255, 0);

				cv::drawMarker(left, position, color);

				cv::imshow("point", left);

				cv::waitKey(1);

			}

		}

		//printf("%f\n", timer.getTime() );

		//全画素を参照
		for (int i = (TEMP_H - 1) / 2; i < image.size[1] - (TEMP_H - 1) / 2 - TEMP_H; i += TEMP_H) {
			for (int j = (TEMP_W - 1) / 2; j < image.size[0] - (TEMP_W - 1) / 2 - TEMP_W; j += TEMP_W) {

				//start = std::chrono::system_clock::now();

				//timer.startTime();

				//printf("%d  %d\n", i, j );
				//切り取り,テンプレート画像作成
				cv::Mat temp_image(image, cv::Rect(i, j, TEMP_H, TEMP_W));
				point.x = i;
				point.y = j;

				result = calcDistance(sub_image, point, temp_image);





				//printf("%f\n", timer.getTime() );



				//map.at<cv::Vec3d>(i, j) = cv::Vec3b(result, result, result);

				//printf("%f ", calcDistance(sub_image, point, temp_image));

			}

			//printf("\n");
		}

	}

	return map;

}*/

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

void objectList(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg) {

	cv::Point left_object[2];
	cv::Point right_object[2];

	left_object[1].x = 0;
	right_object[1].x = 0;

	//カップのみ取得
	for (int i = 0; i < (int)msg->bounding_boxes.size(); ++i) {

		if (msg->bounding_boxes.at(i).Class == "bottle" || msg->bounding_boxes.at(i).Class == "cup" || msg->bounding_boxes.at(i).Class == "vase") {

			if (msg->bounding_boxes.at(i).xmax < imleft.size[1]) { //左側の画像の物体の場合

				left_object[0].x = msg->bounding_boxes.at(i).xmin;
				left_object[0].y = msg->bounding_boxes.at(i).ymin;
				left_object[1].x = msg->bounding_boxes.at(i).xmax;
				left_object[1].y = msg->bounding_boxes.at(i).ymax;

				printf("left cup\n");

			} else { //右側の画像の物体の場合

				right_object[0].x = msg->bounding_boxes.at(i).xmin - imleft.size[1];
				right_object[0].y = msg->bounding_boxes.at(i).ymin;
				right_object[1].x = msg->bounding_boxes.at(i).xmax - imleft.size[1];
				right_object[1].y = msg->bounding_boxes.at(i).ymax;

				printf("right cup\n");

			}
		}

	}

	if (!imleft.empty() && !imright.empty() && left_object[1].x != 0 && right_object[1].x != 0) {

		double min, max;

		cv::Mat left_gray, right_gray, result_color;
		//cv::Mat result_color(480, 640, CV_8UC3);
		printf("debug2\n");
		cvtColor(imleft, left_gray, CV_RGB2GRAY);
		cvtColor(imright, right_gray, CV_RGB2GRAY);

		cv::Mat result1, result2;

		result1 = objectstereo.getDistanceMap(left_gray, left_object, right_gray, right_object, &max, &min);
		//cout << imleft.size[0] << "\n";
		result1.convertTo(result2, CV_8UC1, 255 / (max - min) , -255 * min / (max - min));

		//cvtColor(result2, result_color, CV_GRAY2RGB);

		/*//輪郭の抽出
		std::vector<std::vector<cv::Point>> contours;

		cv::findContours(result2, contours, CV_RETR_LIST, CV_CHAIN_APPROX_TC89_KCOS);

		cv::drawContours(result2, contours, -1, cv::Scalar(255, 255, 255), 3);*/

		//cv::Canny(result2, result2, 50.0, 200.0);

		cv::imshow("result", result2);

		cv::waitKey(1);
	}
}

//cout << msg->bounding_boxes.size() << "\n";



void pubImage(const ros::Publisher & pub, cv::Mat image) {

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

	pub.publish(msg);

}



int main(int argc, char **argv) {

	ros::init(argc, argv, "colortracking");

	ros::NodeHandle n;

	//cv::Mat imleft =  cv::imread("/home/migly/catkin_ws/src/stereo/src/left.png");
	//cv::Mat imright =  cv::imread("/home/migly/catkin_ws/src/stereo/src/right.png");

	//ros::Subscriber left = n.subscribe("/camera_left/image_raw", 1000, calc);
	ros::Subscriber left_sub = n.subscribe("/stereo/left/image_rect_color", 1, imageLeft);
	ros::Subscriber right_sub = n.subscribe("/stereo/right/image_rect_color", 1, imageRight);
	ros::Subscriber objectdata = n.subscribe("/darknet_ros/bounding_boxes", 1, objectList);
	ros::Publisher imagepub = n.advertise<sensor_msgs::Image>("/camera/rgb/image_raw", 1);

	//printf("debug\n");

	//printf("%f\n", stereo.getSimilarity(terget, temp));

	//stereo.getDistanceMap(terget, temp);

	/*cv::Point image1_point(0, 0);
	cv::Point image2_point(0, 0);
	int size[] = {3, 3};

	double min, max;

	cv::Mat left_gray, right_gray;

	//cv::resize(imleft, imleft, cv::Size(), 0.5, 0.5);
	//cv::resize(imright, imright, cv::Size(), 0.5, 0.5);

	cvtColor(imleft, left_gray, CV_RGB2GRAY);
	cvtColor(imright, right_gray, CV_RGB2GRAY);

	cv::Mat result1, result2;

	//printf("%f\n", stereo.getSimilarity(left_gray, image1_point, right_gray, image2_point, size));

	timer.startTime();

	printf("debug\n");

	result1 = stereo.getDistanceMap(left_gray, right_gray, &max, &min);

	printf("%f\n", timer.getTime() );

	printf("%f\n", max );

	result1.convertTo(result2, CV_8UC1, 255 / (max - min) , -255 * min / (max - min));

	cv::imshow("result", result2);

	cv::waitKey();
	*/

	/*cv::Point left_object[2];
	cv::Point right_object[2];

	left_object[0].x = 331.000000;
	left_object[0].y = 49.000000;
	left_object[1].x = left_object[0].x + 211.000000;
	left_object[1].y = left_object[0].y + 202.000000;

	right_object[0].x = 68.000000;
	right_object[0].y = 71.000000;
	right_object[1].x = right_object[0].x + 217.000000;
	right_object[1].y = right_object[0].y + 200.000000;

	double min, max;

	cv::Mat left_gray, right_gray;

	cv::Mat imleft =  cv::imread("/home/migly/catkin_ws/src/stereo/src/left_screenshot_13.09.2018.png");
	cv::Mat imright =  cv::imread("/home/migly/catkin_ws/src/stereo/src/right_screenshot_13.09.2018.png");


	//cv::resize(imleft, imleft, cv::Size(), 0.5, 0.5);
	//cv::resize(imright, imright, cv::Size(), 0.5, 0.5);

	cvtColor(imleft, left_gray, CV_RGB2GRAY);
	cvtColor(imright, right_gray, CV_RGB2GRAY);

	cv::Mat result1, result2;

	result1 = objectstereo.getDistanceMap(left_gray, left_object, right_gray, right_object, &max, &min);

	result1.convertTo(result2, CV_8UC1, 255 / (max - min) , -255 * min / (max - min));

	cv::Mat left_image(imleft, cv::Rect(left_object[0].x, left_object[0].y, left_object[1].x - left_object[0].x, left_object[1].y - left_object[0].y));
	cv::Mat right_image(imright, cv::Rect(right_object[0].x, right_object[0].y, right_object[1].x - right_object[0].x, right_object[1].y - right_object[0].y));

	cv::imshow("left", left_image);
	cv::imshow("right", right_image);

	cv::imshow("result", result2);

	cv::waitKey(0);
	*/


	while (ros::ok()) {

		ros::spinOnce();

		if (!imleft.empty() && !imright.empty()) {

			pubImage(imagepub, objectstereo.joinImage(imleft, imright));

		}

		//cv::Mat imleft =  cv::imread("/home/migly/catkin_ws/src/stereo/src/ballerina_left.jpg");
		//cv::Mat imright =  cv::imread("/home/migly/catkin_ws/src/stereo/src/ballerina_right.jpg");

		/*if (!imleft.empty() && !imright.empty()) {
			printf("debug\n");
			double min, max;

			cv::Mat left_gray, right_gray;

			cv::resize(imleft, imleft, cv::Size(), 0.5, 0.5);
			cv::resize(imright, imright, cv::Size(), 0.5, 0.5);

			cvtColor(imleft, left_gray, CV_RGB2GRAY);
			cvtColor(imright, right_gray, CV_RGB2GRAY);

			cv::Mat result1, result2;

			result1 = stereo.getDistanceMap(left_gray, right_gray, &max, &min);

			result1.convertTo(result2, CV_8UC1, 255 / (max - min) , -255 * min / (max - min));

			//輪郭の抽出
			std::vector<std::vector<cv::Point>> contours;

			cv::findContours(result2, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

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

			cv::drawContours(imleft, contours, -1, cv::Scalar(255, 255, 255), 3);

			cv::imshow("result", result2);

			cv::waitKey(1);
		}*/



		/*double rightTarget[4] = {0, 0, 0, 0};
		double leftTarget[4] = {0, 0, 0, 0};

		rightTarget[0] = 200.000000;
		rightTarget[1] = 217.000000;
		rightTarget[2] = 68.000000;
		rightTarget[3] = 71.000000;

		leftTarget[0] = 202.000000;
		leftTarget[1] = 211.000000;
		leftTarget[2] = 331.000000;
		leftTarget[3] = 49.000000;

		cv::Mat imleft =  cv::imread("/home/migly/catkin_ws/src/stereo/src/left_screenshot_13.09.2018.png");
		cv::Mat imright =  cv::imread("/home/migly/catkin_ws/src/stereo/src/right_screenshot_13.09.2018.png");

		cv::Mat left_image(imleft, cv::Rect(leftTarget[2], leftTarget[3], leftTarget[1], leftTarget[0]));
		cv::Mat right_image(imright, cv::Rect(rightTarget[2], rightTarget[3], rightTarget[1], rightTarget[0]));

		if (!imleft.empty() && !imright.empty()) {


		}*/



		//createParallaxMap(imRight_data, imLeft_data);

	}



	//cv::Point result = matchPoint(terget, temp);

	//std::cout << result << "\n";

	/*cv::Point position(result.x, result.y);

	cv::Scalar color(0, 255, 0);

	cv::drawMarker(terget, position, color);*/

	//cv::rectangle(terget, result, cv::Point(result.x + temp.cols, result.y + temp.rows), CV_RGB(0, 255, 0), 2);

	//cv::imshow("window", terget);

	//cv::waitKey(0);


	return 0;
}