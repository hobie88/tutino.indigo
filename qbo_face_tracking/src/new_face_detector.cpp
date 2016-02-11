#include "new_face_detector.h"
//#define marco_debug



FaceDetector::FaceDetector()
{
	ROS_INFO("Initializing Qbo face detector");
	onInit();
	ROS_INFO("Ready for face following. Waiting for video acquisition");
}

FaceDetector::~FaceDetector()
{
	deleteROSParams();
	printf("Qbo face detector successfully ended\n");
}



void FaceDetector::setROSParams()
{
	string default_classifier_path = "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml";
	//string alternative_classifier_path = "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml";
	string alternative_classifier_path ="/usr/share/opencv/haarcascades/haarcascade_eye.xml";
	//Set default parameter for face classifier path
	private_nh_.param("/qbo_face_tracking/face_classifier_path", face_classifier_path_, default_classifier_path);
	private_nh_.param("/qbo_face_tracking/alternative_face_classifier_path", alternative_face_classifier_path_, alternative_classifier_path);

	//Parameters that define the movement when Qbo is searching for faces
	private_nh_.param("/qbo_face_following/search_min_pan", search_min_pan_, -1.2);
	private_nh_.param("/qbo_face_following/search_max_pan", search_max_pan_, 1.2);
	private_nh_.param("/qbo_face_following/search_pan_vel", search_pan_vel_, 0.3);
	private_nh_.param("/qbo_face_following/search_max_tilt", search_max_tilt_, 1.0);
	private_nh_.param("/qbo_face_following/search_min_tilt", search_min_tilt_, -1.0);
	private_nh_.param("/qbo_face_following/search_tilt_vel", search_tilt_vel_, 0.3);
	private_nh_.param("/qbo_face_following/desired_distance", desired_distance_, 0.6);

	private_nh_.param<int>("/qbo_face_tracking/undetected_threshold", undetected_threshold_, 5);
}

void FaceDetector::deleteROSParams()
{
	private_nh_.deleteParam("/qbo_face_tracking/face_classifier_path");
	private_nh_.deleteParam("/qbo_face_tracking/alternative_face_classifier_path");
	private_nh_.deleteParam("/qbo_face_following/search_min_pan");
	private_nh_.deleteParam("/qbo_face_following/search_max_pan");
	private_nh_.deleteParam("/qbo_face_following/search_pan_vel");
	private_nh_.deleteParam("/qbo_face_following/search_max_tilt");
	private_nh_.deleteParam("/qbo_face_following/search_min_tilt");
	private_nh_.deleteParam("/qbo_face_following/search_tilt_vel");
	private_nh_.deleteParam("/qbo_face_following/desired_distance");

	private_nh_.deleteParam("/qbo_face_tracking/undetected_threshold");

}
void FaceDetector::onInit()
{
	/*
	 * Setting ROS Parameters
	 */
	setROSParams();

	cvNamedWindow("Kalman predictor", CV_WINDOW_AUTOSIZE); 	// output screen
	cvInitFont(&font_,CV_FONT_HERSHEY_PLAIN, 1.0, 4.0, 2,2,CV_AA);
	textColor_ = CV_RGB(0,255,255);
	first_time_=true;
	notFoundCount_=0;

	found_=false;
	random_=true;

	if(!face_classifier_.load(face_classifier_path_))
	{
		ROS_ERROR("Error importing face Haar cascade classifier from the specified path: %s", face_classifier_path_.c_str());
		deleteROSParams();
		exit(-1);
	}
	else
	{
		ROS_INFO("Haar cascade classifier successfully loaded from %s", face_classifier_path_.c_str());
	}
	if(!alternative_face_classifier_.load(alternative_face_classifier_path_))
    {
		ROS_WARN("Error importing alternative face Haar cascade classifier from the specified path: %s", alternative_face_classifier_path_.c_str());
        exist_alternative_=false;
    }
    else
    {
        ROS_INFO("Alternative Haar cascade classifier successfully loaded from %s", alternative_face_classifier_path_.c_str());
        exist_alternative_=true;
	}

	/*
	 * Subscribers of the node
	 */
	info_sub_=private_nh_.subscribe<sensor_msgs::CameraInfo>("/usb_cam/camera_info",10,&FaceDetector::infoCallback, this);
	joint_states_sub_=private_nh_.subscribe<sensor_msgs::JointState>("/joint_states",10,&FaceDetector::jointStateCallback, this);

	/*
	 * Publishers of the node
	 */
	//Publisher of the head joint positions
	joint_pub_=private_nh_.advertise<sensor_msgs::JointState>("/cmd_joints",1);
	//Publisher of the body movements
	//base_control_pub_=private_nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
	base_control_pub_=private_nh_.advertise<geometry_msgs::Twist>("/pre_cmd_vel",1);
	face_distance_pub_=private_nh_.advertise<std_msgs::Float32>("/head_distance",1);


	/*
	 * Initialize some internal parameters values
	 */
	face_detected_bool_ = false;
	image_size_ = cv::Size(0,0);

	//Initialize Kalman filter
	initializeKalmanFilter();
}


void FaceDetector::infoCallback(const sensor_msgs::CameraInfo::ConstPtr& info)
{
#ifdef marco_debug
	ROS_INFO("infocallback called");
#endif
	if(p_.data==NULL)
	{

		cv::Mat p=cv::Mat(3,4,CV_64F); // CV_64F : simple (3x4 array) grayscale image
		for (int i=0;i<3;i++)
		{
			for (int j=0;j<4;j++)
			{
				p.at<double>(i,j)=info->P[4*i+j];
			}
		}
		p(cv::Rect(0,0,3,3)).convertTo(p_,CV_32F); //CV_32F is float
												//the pixel can have any value between 0-1.0,
												//this is useful for some sets of calculations on data
												//but it has to be converted into 8bits to save or
												//display by multiplying each pixel by 255.


		image_sub_=private_nh_.subscribe<sensor_msgs::Image>("/usb_cam/image_raw",1,&FaceDetector::imageCallback, this);
		info_sub_.shutdown();
		//TODO - Unsubscribe to the camera info topic // done
	}
}


void FaceDetector::imageCallback(const sensor_msgs::Image::ConstPtr& image_ptr)
{
#ifdef marco_debug
	ROS_INFO("imagecallback called");
#endif
	cv_bridge::CvImagePtr cv_ptr;
    try
    {
    	cv_ptr = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
	    ROS_ERROR("cv_bridge exception: %s", e.what());
	    return;
    }

    cv::Mat image_received = (cv_ptr->image).clone();
    image_size_ = cv_ptr->image.size();

    vector<cv::Rect> faces_roi;
    cv::Mat grayscale;
    cv::cvtColor(image_received,grayscale,CV_RGB2GRAY); // marco: converting to grayscale before haar detection
    detectFacesHaar(image_received, faces_roi,face_classifier_);

    if(faces_roi.size() > 0) //If Haar cascade classifier found a face
    {
#ifdef marco_debug
    		ROS_INFO("haar cascade found a face");
#endif
        vector<cv::Rect> eyes_roi;
        detectFacesHaar(grayscale(faces_roi[0]), eyes_roi,alternative_face_classifier_);
    	if (eyes_roi.size()>0)
    	{
        	//ROS_INFO("HAAR CLASSIFIER FOUND A FACE");
        	face_detected_bool_ = true;
        	track_object_ = false;

        		//Changed
        	detected_face_roi_ = faces_roi[0];
        	detected_face_ = cv_ptr->image(detected_face_roi_);

    	}
    	else face_detected_bool_=false;


    }
    else
    {
    	face_detected_bool_=false;
    }

    // >>>>>>>> KALMAN
    double precTick = ticks_;
    ticks_ = (double) cv::getTickCount();
    double dT = (ticks_ - precTick) / cv::getTickFrequency(); //seconds

    if (found_)
          {
             // >>>> Matrix A
             kalman_filter_.transitionMatrix.at<float>(2) = dT;
             kalman_filter_.transitionMatrix.at<float>(9) = dT;
             // <<<< Matrix A

             state_ = kalman_filter_.predict();
             //cout << "State post:" << endl << state_ << endl;
             cv::Rect predRect;
             predRect.width = state_.at<float>(4);
             predRect.height = state_.at<float>(5);
             predRect.x = state_.at<float>(0) - predRect.width / 2;
             predRect.y = state_.at<float>(1) - predRect.height / 2;
             cv::Point center;
             center.x = state_.at<float>(0);
             center.y = state_.at<float>(1);
             cv::circle(image_received, center, 2, CV_RGB(255,0,0), -1);
             cv::rectangle(image_received, predRect, CV_RGB(255,0,0), 2);
          }


    if (!face_detected_bool_)
    {
    	notFoundCount_++;
        //cout << "notFoundCount:" << notFoundCount_ << endl;
        if( notFoundCount_ >= 10 )
        {

            found_ = false;
        }
        else
        {
            kalman_filter_.statePost = state_;
        }
    }
    else
    {
    	notFoundCount_ = 0;

        meas_.at<float>(0) = (float)(detected_face_roi_.x + detected_face_roi_.width/2);
        meas_.at<float>(1) = (float)(detected_face_roi_.y + detected_face_roi_.height/2);
        meas_.at<float>(2) = (float)detected_face_roi_.width;
        meas_.at<float>(3) = (float)detected_face_roi_.height;

             if (!found_) // First detection!
             {
                // >>>> Initialization
                kalman_filter_.errorCovPre.at<float>(0) = 1; // px
                kalman_filter_.errorCovPre.at<float>(7) = 1; // px
                kalman_filter_.errorCovPre.at<float>(14) = 1;
                kalman_filter_.errorCovPre.at<float>(21) = 1;
                kalman_filter_.errorCovPre.at<float>(28) = 1; // px
                kalman_filter_.errorCovPre.at<float>(35) = 1; // px

                state_.at<float>(0) = meas_.at<float>(0);
                state_.at<float>(1) = meas_.at<float>(1);
                state_.at<float>(2) = 0;
                state_.at<float>(3) = 0;
                state_.at<float>(4) = meas_.at<float>(2);
                state_.at<float>(5) = meas_.at<float>(3);
                // <<<< Initialization

                found_ = true;
             }
             else
                kalman_filter_.correct(meas_); // Kalman Correction


           //  cout << "Measure matrix:" << endl << meas_ << endl;
          }


    float head_distance;
   // ROS_INFO("Computing head distance");
	if(!face_detected_bool_)
    {
	  if(head_distances_.size()!=0)
	  {
			head_distance=head_distances_[0];//100;
	  }
		else
		  head_distance = 3.0;


	}
	else
	{
		head_distance=calcDistanceToHead(detected_face_, kalman_filter_);
#ifdef marco_debug
		ROS_ERROR ("COMPUTED HEAD DISTANCE: %f",head_distance);
#endif

	}

    if(head_distances_.size()==0)
    {
      for(int i=0;i<10;i++)
	    head_distances_.push_back(head_distance);
    }
    else
    {
      head_distances_.pop_back();
      head_distances_.insert(head_distances_.begin(),head_distance);
    }
#ifdef marco_debug
    ROS_INFO("head distance computed");
#endif

    head_distance=0; //Reuse variable to compute mean head distance

    //Use mean distance of last measured head distances
    for(unsigned int i=0;i<head_distances_.size();i++)
    	head_distance+=head_distances_[i];

    head_distance=head_distance/head_distances_.size();

	//ROS_ERROR ("HEAD DISTANCE AFTER MEAN COMPUTING: %f",head_distance);


#ifdef marco_debug
	ROS_INFO("updated undetected count");
#endif
	if(head_distance != head_distance) //not a number
		head_distance=3.0;


	//Create HeadDistance message
	std_msgs::Float32 head_distance_msgs;
	head_distance_msgs.data=head_distance;
	face_distance_pub_.publish(head_distance_msgs);
	//Create Face Pos and Size message
	sensor_msgs::JointState message;
	int servos_count=2;
	message.name.resize(servos_count);
	message.position.resize(servos_count);
	message.velocity.resize(servos_count);

	message.name[0]="head_pan_joint";	//izquierda-derecha

	message.name[1]="head_tilt_joint";	//arriba-abajo

	message.header.stamp = ros::Time::now();
#ifdef marco_debug
	ROS_INFO("message created");
#endif

	if( notFoundCount_ >= 10 )
	{
		float rand_tilt = search_min_tilt_+((search_max_tilt_-search_min_tilt_) * double(rand()%20)/20.);
		float rand_pan = search_min_pan_+((search_max_pan_-search_min_pan_) * double(rand()%20)/20.);
		message.position[0]=rand_pan;
		message.position[1]=rand_tilt;
		message.velocity[0]=search_pan_vel_;
		message.velocity[1]=search_tilt_vel_;
		//ROS_ERROR("RANDOM MOVE: tilt: %f  pan: %f",rand_tilt,rand_pan);
		random_=true;
	}
	else
	{
		if(face_detected_bool_)
		{
			float pan_pos=meas_.at<float>(0); // it was meas_
			float tilt_pos=meas_.at<float>(1);
			pan_pos=scalePan(pan_pos);
			tilt_pos=scaleTilt(tilt_pos);
			message.position[0]=yaw_from_joint_ + pan_pos;
			message.position[1]=pitch_from_joint_ + tilt_pos;
			message.velocity[0]=abs(pan_pos*1.3)  ;
			message.velocity[1]=abs(tilt_pos*1.5);
		}
		else
		{
			float pan_pos=state_.at<float>(0); // it was meas_
			float tilt_pos=state_.at<float>(1);
			pan_pos=scalePan(pan_pos);
			tilt_pos=scaleTilt(tilt_pos);
			message.position[0]=yaw_from_joint_ + pan_pos;
			message.position[1]=pitch_from_joint_ + tilt_pos;
			message.velocity[0]=abs(pan_pos*1.3);
			message.velocity[1]=abs(tilt_pos*1.5);
		}
		//ROS_ERROR("KALMAN MOVE: pan: %f --- tilt: %f",message.position[0], message.position[1]);
		random_=false;
	}

	cv::imshow("Kalman predictor", image_received);
	cvWaitKey(1);



    /*
     * Publish joint state
     */
#ifdef marco_debug
	ROS_INFO("publishing: pan_pos: %lg --- tilt_pos: %lg",message.position[0],message.position[1]);
#endif
ROS_INFO("PAN: %lg -- TILT: %lg",message.position[0],message.position[1]);
	joint_pub_.publish(message);

	sendBaseVelocity(head_distance);

}

void FaceDetector::setFaceClassifierPath(std::string face_classifier_path)
{
	face_classifier_.load(face_classifier_path);
}


void FaceDetector::initializeKalmanFilter()
{
	int stateSize = 6;
	int measSize = 4;
	int contrSize = 0;

	unsigned int type = CV_32F;
	kalman_filter_ = cv::KalmanFilter(stateSize, measSize, contrSize, type);

	state_=cv::Mat(stateSize,1,type);
	meas_=cv::Mat(measSize,1,type);
	//state_(6, 1, type);  // [x,y,v_x,v_y,w,h]
	//meas_(measSize, 1, 5);    // [z_x,z_y,z_w,z_h]

	// Transition State Matrix A
	// Note: set dT at each processing step!
	// [ 1 0 dT 0  0 0 ]
	// [ 0 1 0  dT 0 0 ]
	// [ 0 0 1  0  0 0 ]
	// [ 0 0 0  1  0 0 ]
	// [ 0 0 0  0  1 0 ]
	// [ 0 0 0  0  0 1 ]
	cv::setIdentity(kalman_filter_.transitionMatrix);

	 // Measure Matrix H
	 // [ 1 0 0 0 0 0 ]
	 // [ 0 1 0 0 0 0 ]
	 // [ 0 0 0 0 1 0 ]
	 // [ 0 0 0 0 0 1 ]
	kalman_filter_.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
	kalman_filter_.measurementMatrix.at<float>(0) = 1.0f;
	kalman_filter_.measurementMatrix.at<float>(7) = 1.0f;
	kalman_filter_.measurementMatrix.at<float>(16) = 1.0f;
	kalman_filter_.measurementMatrix.at<float>(23) = 1.0f;

	// Process Noise Covariance Matrix Q
	 // [ Ex 0  0    0 0    0 ]
	 // [ 0  Ey 0    0 0    0 ]
	 // [ 0  0  Ev_x 0 0    0 ]
	 // [ 0  0  0    1 Ev_y 0 ]
	 // [ 0  0  0    0 1    Ew ]
	 // [ 0  0  0    0 0    Eh ]
	 //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
	kalman_filter_.processNoiseCov.at<float>(0) = 1e-2;
	kalman_filter_.processNoiseCov.at<float>(7) = 1e-2;
	kalman_filter_.processNoiseCov.at<float>(14) = 2.0f;
	kalman_filter_.processNoiseCov.at<float>(21) = 1.0f;
	kalman_filter_.processNoiseCov.at<float>(28) = 1e-2;
	kalman_filter_.processNoiseCov.at<float>(35) = 1e-2;

	// Measures Noise Covariance Matrix R
	cv::setIdentity(kalman_filter_.measurementNoiseCov, cv::Scalar(1e-1));


}

unsigned int FaceDetector::detectFacesHaar(cv::Mat image, std::vector<cv::Rect> &faces, cv::CascadeClassifier classifier)
{
	classifierDetect(image,faces,classifier);
	return faces.size();
}



void FaceDetector::classifierDetect(cv::Mat image, std::vector<cv::Rect>& detections, cv::CascadeClassifier classifier,int flag, cv::Size size)
{
	classifier.detectMultiScale(image, detections, 1.1, 2,  flag, size);//, cv::Size(100,100));

}

float FaceDetector::calcDistanceToHead(cv::Mat& head, cv::KalmanFilter& kalman_filter)
{
	float u_left=kalman_filter.statePost.at<float>(0,0)-head.cols/2;
	cv::Mat uv_left(3,1,CV_32F);
	uv_left.at<float>(0,0)=u_left;
	uv_left.at<float>(1,0)=kalman_filter.statePost.at<float>(1,0);
	uv_left.at<float>(2,0)=1;
	float u_right=u_left+head.cols;
	cv::Mat uv_right(3,1,CV_32F);
	uv_right.at<float>(0,0)=u_right;
	uv_right.at<float>(1,0)=kalman_filter.statePost.at<float>(1,0);
	uv_right.at<float>(2,0)=1;


	cv::Mat cart_left=p_.inv()*uv_left;
	cv::Mat cart_right=p_.inv()*uv_right;
//	std::cout << "cart left: " << cart_left << std::endl;

	cart_left=cart_left/sqrt(cart_left.dot(cart_left));
	cart_right=cart_right/sqrt(cart_right.dot(cart_right));

//	std::cout << "cart left after: " << cart_left << std::endl;
//	std::cout << "acos argument: " << cart_left.dot(cart_right) << std::endl;
	float theta=acos(cart_left.dot(cart_right));
	float l=(HEAD_SIZE/2)/tan(theta/2);
	//ROS_ERROR("THETA= %f  --- DISTANCE TO HEAD= %f",theta,l);
	return l;
}

double FaceDetector::euclidDist(cv::Point2f pt1, cv::Point2f pt2)
{
	return sqrt(pow(pt1.x-pt2.x, 2) + pow(pt1.y-pt2.y, 2));
}

void FaceDetector::jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state)
{
	if(joint_state->position.size()!=joint_state->name.size())
    {
        ROS_ERROR("Malformed JointState message has arrived. Names size and positions size do not match");
        return;
    }
    for (int j=0;j<(int)joint_state->position.size();j++)
    {
	//ROS_INFO("**********Position %d: %f",j,joint_state->position[j]);
    }
    for (int i=0;i<(int)joint_state->name.size();i++)
    {
        if (joint_state->name[i].compare("head_pan_joint")==0)
        {
        	yaw_from_joint_=joint_state->position[i];

		//ROS_INFO("********* head pan joint: %f",yaw_from_joint_);
        	//std::cout << "Mensaje pan: " <<  yaw_from_joint_ << std::endl;
        }
        if (joint_state->name[i].compare("head_tilt_joint")==0)
        {
        	pitch_from_joint_=joint_state->position[i];
        	//std::cout << "Mensaje tilt: " <<  pitch_from_joint_ << std::endl;
        }
    }
}

void FaceDetector::debug()
{

	ROS_INFO("Printing kalman filter elements...");
	for(int i=0; i<kalman_filter_.statePost.total() ; i++)
	{
		ROS_INFO("statePost %d: %f",i,kalman_filter_.statePost.at<float>(i,1));
	}
	if(!kalman_predicted_state_.empty())
	{
		ROS_INFO("Printing predicted kalman filter elements...");
		for(int i=0; i<kalman_predicted_state_.total() ; i++)
		{
			ROS_INFO("predicted %d: %f",i,kalman_predicted_state_.at<float>(i));
		}
	}
	if(!kalman_estimate_state_.empty())
	{
		ROS_INFO("Printing corrected kalman filter elements...");
		for(int i=0; i<kalman_estimate_state_.total() ; i++)
		{
			ROS_INFO("estimated %d: %f",i,kalman_estimate_state_.at<float>(i));
		}
	}
}

void FaceDetector::sendBaseVelocity(float head_distance)
{
	geometry_msgs::Twist message;
	if(random_) // stop body
	{

		//message.linear.x=0;
		message.linear.x=0.1;
		message.linear.y=0;
		message.linear.z=0;
		message.angular.x=0;
		message.angular.y=0;
		message.angular.z=0;
	}
	else
	{
		float linear= (head_distance - desired_distance_);
		float angular = yaw_from_joint_;
		linear*=0.5;
		//angular*=0.5;
		if (linear > 0.2) linear = 0.2;
		if (linear < -0.2) linear = -0.2;
		if (angular > 0.8) angular = 0.8;
		if (angular < -0.8) angular = -0.8;

		message.linear.x=linear;
		message.linear.y=0;
		message.linear.z=0;
		message.angular.x=0;
		message.angular.y=0;
		message.angular.z=angular;
	}
	base_control_pub_.publish(message);
//ROS_ERROR("linear: %f, angular: %f",message.linear.x,message.angular.z);
	//ROS_INFO("Head Distance: %f",head_distance);
}

float FaceDetector::scalePan(float x)
{
	return -((2*(x)/640)-1);

}
float FaceDetector::scaleTilt(float x)
{
	return (0.6*(x)/480)-0.3;

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "qbo_face_tracking");

	FaceDetector face_detector;

	ros::spin();

	return 0;
}
