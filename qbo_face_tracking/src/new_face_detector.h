/*
 * Software License Agreement (GPLv2 License)
 *
 * Copyright (c) 2016 TurinTech, S.L.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 * Author: Arena Marco <arena@turintech.it>
 */

#ifndef FACEDETECTOR_H_
#define FACEDETECTOR_H_

#include <iostream>
#include <vector>
#include <cmath>
#include <string>

#include <cv.h>
#include <highgui.h>
#include <sstream>
#include <cvaux.h>
#include <cxmisc.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/SetCameraInfo.h>
//#include <qbo_face_msgs/FacePosAndDist.h>
//#include <image_transport/image_transport.h>
//#include <qbo_face_msgs/RecognizeFace.h>
//#include <qbo_face_msgs/GetName.h>
//#include <qbo_arduqbo/Nose.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#define HEAD_SIZE 0.20

//#define WITH_GPU

using namespace std;

class FaceDetector
{
private:
	void onInit();
	/*
	 * ROS Parameters
	 */
	string face_classifier_path_;
    string alternative_face_classifier_path_;
	cv::Point2d default_pos_;
	int check_Haar_;
	int check_track_obj_;
	bool move_base_bool_;
	bool move_head_bool_;
	double search_min_pan_;
	double search_max_pan_;
	double search_pan_vel_;
	double search_min_tilt_;
	double search_max_tilt_;
	double search_tilt_vel_;
	double desired_distance_;
	int undetected_threshold_;
	bool first_time_;
//	double distance_threshold_;
//	bool send_to_face_recognizer_;
//	bool print_recognized_face_;


	/*
	 * ROS Elements
	 */
	ros::NodeHandle private_nh_;

	ros::Subscriber image_sub_;
	ros::Subscriber info_sub_;
	ros::Subscriber joint_states_sub_;

//	ros::Publisher face_position_and_size_pub_;
//	ros::Publisher face_pub_;
//	ros::Publisher nose_color_pub_;
//	image_transport::ImageTransport it_;
//	image_transport::Publisher viewer_image_pub_;

	ros::Publisher joint_pub_;
	ros::Publisher base_control_pub_;

	/*
	 * Elements for the face recognition client
	 */
//	ros::ServiceClient client_recognize_;
//	qbo_face_msgs::RecognizeFace srv;

//	ros::ServiceClient client_get_name;
//	qbo_face_msgs::GetName srv_get_name;

	/*
	 * Internal parameters
	 */
	CvFont font_;
	CvScalar textColor_;
	cv::Size image_size_;
	bool track_object_;
	bool face_detected_bool_;
//	std::string name_detected_;
	cv::Rect detected_face_roi_;
	cv::Mat detected_face_;

	cv::KalmanFilter kalman_filter_;
	cv::Mat kalman_predicted_state_;
	cv::Mat kalman_estimate_state_;
	vector<float> head_distances_;
	cv::Mat p_;
	//kalman filter stuff:
	cv::Mat meas_;
	cv::Mat state_;
	bool found_;
	int notFoundCount_;
	double ticks_;


	int loop_counter_;
	float face_ratio_;
	int undetected_count_;
	bool random_;

	bool dynamic_check_haar_;
	int cam_shift_detected_count_;
	int cam_shift_undetected_count_;
	bool exist_alternative_;
	float max_pan_;
	float max_tilt_;

	cv::CascadeClassifier face_classifier_;
    cv::CascadeClassifier alternative_face_classifier_;
	//Regarding FaceRecognizer

    float yaw_from_joint_;
    float pitch_from_joint_;

	/*
	 * Methods
	 */

	//ROS Callbacks
	void imageCallback(const sensor_msgs::Image::ConstPtr& image_ptr);
	void infoCallback (const sensor_msgs::CameraInfo::ConstPtr& info);
	void jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state);
	//Initialization methods
	void sendBaseVelocity(float);
	void setFaceClassifierPath(std::string face_classifier_path);
	void initializeKalmanFilter();
	void setROSParams();

	//Recognizer methods
//	void sendToRecognizer();
//	void sendToRecognizerGetName();


	//Face detection methods
	unsigned int detectFacesHaar(cv::Mat image, std::vector<cv::Rect> &faces, cv::CascadeClassifier);
 //   unsigned int detectFacesAltHaar(cv::Mat image, std::vector<cv::Rect> &faces);
	unsigned int detectFaceCamShift(cv::Mat img);
	void classifierDetect(cv::Mat image, std::vector<cv::Rect>& detections, cv::CascadeClassifier classifier, int flag= CV_HAAR_FIND_BIGGEST_OBJECT, cv::Size size=cv::Size());

	//Some other auxiliary methods
	double euclidDist(cv::Point2f pt1, cv::Point2f pt2);
	float calcDistanceToHead(cv::Mat& head, cv::KalmanFilter& kalman_filter);
	void deleteROSParams();
	void debug();
	float scalePan(float);
	float scaleTilt(float);

public:
	FaceDetector();

	virtual ~FaceDetector();
};

#endif /* FACEDETECTOR_H_ */
