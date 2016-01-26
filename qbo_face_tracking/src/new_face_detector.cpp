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
	//Set default value for move base boolean
	private_nh_.param("/qbo_face_following/move_base", move_base_bool_, true);

	//set default value for move head boolean
	private_nh_.param("/qbo_face_following/move_head", move_head_bool_, true);

	private_nh_.param<int>("check_Haar", check_Haar_, -1);
	//Track object period for CAM Shift. When track object is true, the CAM shift algorithm will refresh the color skin
	private_nh_.param<int>("/qbo_face_tracking/check_track_object", check_track_obj_, check_Haar_);


	//Parameters that define the movement when Qbo is searching for faces
	private_nh_.param("/qbo_face_following/search_min_pan", search_min_pan_, -1.2);
	private_nh_.param("/qbo_face_following/search_max_pan", search_max_pan_, 1.2);
	private_nh_.param("/qbo_face_following/search_pan_vel", search_pan_vel_, 0.3);
	private_nh_.param("/qbo_face_following/search_max_tilt", search_max_tilt_, 0.9);
	private_nh_.param("/qbo_face_following/search_min_tilt", search_min_tilt_, -0.9);
	private_nh_.param("/qbo_face_following/search_tilt_vel", search_tilt_vel_, 0.3);
	private_nh_.param("/qbo_face_following/desired_distance", desired_distance_, 0.6);

	private_nh_.param<int>("/qbo_face_tracking/undetected_threshold", undetected_threshold_, 5);
}

void FaceDetector::deleteROSParams()
{
	private_nh_.deleteParam("/qbo_face_tracking/face_classifier_path");
	private_nh_.deleteParam("/qbo_face_tracking/alternative_face_classifier_path");
	private_nh_.deleteParam("/qbo_face_following/move_base");
	private_nh_.deleteParam("/qbo_face_following/move_head");
	private_nh_.deleteParam("/qbo_face_following/search_min_pan");
	private_nh_.deleteParam("/qbo_face_following/search_max_pan");
	private_nh_.deleteParam("/qbo_face_following/search_pan_vel");
	private_nh_.deleteParam("/qbo_face_following/search_max_tilt");
	private_nh_.deleteParam("/qbo_face_following/search_min_tilt");
	private_nh_.deleteParam("/qbo_face_following/search_tilt_vel");
	private_nh_.deleteParam("/qbo_face_following/desired_distance");

}
void FaceDetector::onInit()
{
	/*
	 * Setting ROS Parameters
	 */
	setROSParams();

	max_pan_ = 0;
	max_tilt_ = 0;
	cvNamedWindow("Input", CV_WINDOW_AUTOSIZE); 	// output screen
	cvInitFont(&font_,CV_FONT_HERSHEY_PLAIN, 1.0, 4.0, 2,2,CV_AA);
	textColor_ = CV_RGB(0,255,255);



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
	//info_sub_=private_nh_.subscribe<sensor_msgs::CameraInfo>("/stereo/left/camera_info",10,&FaceDetector::infoCallback, this);
	info_sub_=private_nh_.subscribe<sensor_msgs::CameraInfo>("/usb_cam/camera_info",10,&FaceDetector::infoCallback, this);
	joint_states_sub_=private_nh_.subscribe<sensor_msgs::JointState>("/joint_states",10,&FaceDetector::jointStateCallback, this);

	/*
	 * Publishers of the node
	 */
	//Publisher of the head joint positions
	joint_pub_=private_nh_.advertise<sensor_msgs::JointState>("/cmd_joints",10);
	//Publisher of the body movements
	base_control_pub_=private_nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);

	/*
	 * Initialize some internal parameters values
	 */
	face_detected_bool_ = false;
	image_size_ = cv::Size(0,0);

	//Initialize Kalman filter
	initializeKalmanFilter();

	loop_counter_ = 0;

	undetected_count_ = undetected_threshold_;


	/*
	 * Set dynamic Haar Cascade check
	 */
	cam_shift_detected_count_ = 0;
	dynamic_check_haar_ = false;

	if(check_Haar_<=0)
	{	dynamic_check_haar_ = true;
		check_Haar_=50;
		track_object_ = check_Haar_;
	}

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

		//TODO - Unsubscribe to the camera info topic
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
    string detection_type = "";

    if(face_detected_bool_) //If face was detected in previous iteration - use CAM shift
    {

    	if(detectFaceCamShift(image_received) != 0)
    	{
#ifdef marco_debug
    		ROS_INFO("CAMshift no face found");
#endif
    		face_detected_bool_ = false;
    		//Reset cam_shift_detected
    		cam_shift_detected_count_ = 0;
    		cam_shift_undetected_count_++;
    	}
    	else
    	{
#ifdef marco_debug
    		ROS_INFO("CAMshift found a face");
#endif
    		detection_type = "CAMSHIFT";

    		//Update cam_shift counter
    		cam_shift_detected_count_++;
    		cam_shift_undetected_count_ = 0;
    	}
    }


    if(!face_detected_bool_) //If face was not detected - use Haar Cascade
    {
#ifdef marco_debug
    	ROS_INFO("FACE WAS NOT DETECTED; USE HAAR CASCADE");
#endif
        vector<cv::Rect> faces_roi;
        cv::Mat grayscale;
        cv::cvtColor(image_received,grayscale,CV_RGB2GRAY); // marco: converting to grayscale before haar detection
    	detectFacesHaar(image_received, faces_roi,face_classifier_);

    	if(faces_roi.size() > 0) //If Haar cascade classifier found a face
    	{
#ifdef marco_debug
    		ROS_INFO("haar cascade found a face");
#endif
    		//ROS_INFO("USING ALTERNATE HAAR CASCADE");
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
        		detection_type = "HAAR";

        		//Adjust face ratio because Haar Cascade always returns a square
        		face_ratio_ = 1.3*detected_face_roi_.height/detected_face_roi_.width;
    		}


    	}
    }



	/*
	 * Kalman filter for Face pos estimation
	 */
#ifdef marco_debug
    ROS_INFO("kalman predicting");
#endif
    kalman_predicted_state_ = kalman_filter_.predict();
	if(face_detected_bool_) //IF face detected, use measured position to update kalman filter
	{
		if(undetected_count_>=undetected_threshold_)
		{
#ifdef marco_debug
			ROS_INFO("first time new face. initialize kalman");
#endif
			cv::Mat new_state(2,1, CV_32FC1);
			new_state.at<float>(0) = float(detected_face_roi_.x+detected_face_roi_.width/2.); //misure
			new_state.at<float>(1) = float(detected_face_roi_.y+detected_face_roi_.height/2.); // misure
			new_state.at<float>(2) = 0.; // dati passati
			new_state.at<float>(3) = 0.; // dati passati

			new_state.copyTo(kalman_filter_.statePre);
			//ROS_INFO("kalman initialized");
		}
		else
		{
			cv::Mat measures(2,1,CV_32FC1);
			measures.at<float>(0)= (float)(detected_face_roi_.x + detected_face_roi_.width/2);
			measures.at<float>(1)= (float)(detected_face_roi_.y + detected_face_roi_.height/2);
			measures.at<float>(2)= float(detected_face_roi_.x+detected_face_roi_.width/2.)-kalman_filter_.statePost.at<float>(0,0);
			measures.at<float>(3)= float(detected_face_roi_.y+detected_face_roi_.height/2.)-kalman_filter_.statePost.at<float>(1,0);
			// TO DO AGGIUNGERE VELOCITA
			kalman_estimate_state_ = kalman_filter_.correct(measures);
			//ROS_INFO("kalman corrected");
		}
	}
    else //If face haven't been found, use only Kalman prediction
    {
    	//ROS_INFO("kalman face just lost");
    		//ROS_INFO("KALMAN FILTER: NO FACE FOUND");
    	    kalman_filter_.statePre.copyTo(kalman_filter_.statePost);
    	    kalman_filter_.errorCovPre.copyTo(kalman_filter_.errorCovPost);
    }

	/*
	 * Compute head distance
	 */
    float head_distance;
   // ROS_INFO("Computing head distance");
	if(!face_detected_bool_)
    {
	  if(head_distances_.size()!=0)
	  {
			head_distance=head_distances_[0];//100;
	  }
		else
		  head_distance = 5;


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

    //Update undetected count
	if(!face_detected_bool_)
	{	undetected_count_++;

	}
	else
	{
		undetected_count_ = 0;
	}
#ifdef marco_debug
	ROS_INFO("updated undetected count");
#endif
	//Create Face Pos and Size message
	sensor_msgs::JointState message;
	int servos_count=2;
	message.name.resize(servos_count);
	message.position.resize(servos_count);
	message.velocity.resize(servos_count);

	message.name[0]="head_pan_joint";	//izquierda-derecha
	//message.position[0]= kalman_filter_.statePost.at<float>(0,0) - cv_ptr->image.cols/2 ;
	//message.position[0]=detected_face_roi_.x+detected_face_roi_.width/2.;
	message.velocity[0]= 0.3;

	message.name[1]="head_tilt_joint";	//arriba-abajo
	//message.position[1]= kalman_filter_.statePost.at<float>(1,0) - cv_ptr->image.rows/2;
	//message.position[1]=detected_face_roi_.y+detected_face_roi_.height/2.;
	message.velocity[1]= 0.3;

	// MARCO: TODO:
	/*
	 * prova a mandare la posizione della testa, senza cv_ptr->image...
	 * calcolare velocità
	 */

	message.header.stamp = ros::Time::now();
#ifdef marco_debug
	ROS_INFO("message created");
#endif

	if(undetected_count_>=undetected_threshold_ && !face_detected_bool_)
	{
		// random move
		srand(time(NULL));
		float rand_tilt = search_min_tilt_+(search_max_tilt_-search_min_tilt_) * (double)(rand()%20)/20.;
		float rand_pan = search_min_pan_+(search_max_pan_-search_min_pan_) * (double)(rand()%20)/20.;
		ROS_ERROR("RANDOM MOVE -- BEFORE ATAN: tilt: %f  pan: %f",rand_tilt,rand_pan);
		//rand_pan=atan2((rand_pan - p_.at<float>(0,2)),p_.at<float>(0,0));
		//rand_tilt=atan2((rand_tilt - p_.at<float>(1,2)),p_.at<float>(1,1));
		message.position[0] = - (yaw_from_joint_ + rand_pan);
		message.position[1] = - (pitch_from_joint_ + rand_tilt);
		//ROS_ERROR("RANDOM MOVE -- AFTER ATAN: tilt: %f  pan: %f",rand_tilt,rand_pan);
	}
	else
	{
		// use kalman
		//ROS_ERROR("KALMAN MOVE -- BEFORE ATAN: tilt: %f  pan: %f",kalman_filter_.statePost.at<float>(0),kalman_filter_.statePost.at<float>(1));
		//float pan_pos = kalman_filter_.statePost.at<float>(0) - cv_ptr->image.cols/2.; // marco old
		//float tilt_pos = kalman_filter_.statePost.at<float>(1) - cv_ptr->image.rows/2.; // marco old
		float pan_pos = kalman_filter_.statePost.at<float>(0);
		float tilt_pos = kalman_filter_.statePost.at<float>(1);

		//pan_pos= -1.57 + pan_pos*(3.14/640); //scaling
		//tilt_pos= -1.0 + tilt_pos*(2/480);
		ROS_INFO("tilt_pos before atan: %f",tilt_pos);
		pan_pos = atan2((pan_pos - p_.at<float>(0,2)),p_.at<float>(0,0));
		//tilt_pos = atan2((- tilt_pos - p_.at<float>(1,2)),p_.at<float>(1,1)); // added minus
		tilt_pos = atan2((- tilt_pos + p_.at<float>(1,2)),p_.at<float>(1,1));
		ROS_INFO("tilt_pos after atan: %f",tilt_pos);
		//ROS_INFO("scaled pan_pos: %f -- scaled tilt_pos: %f",pan_pos,tilt_pos);
		//ROS_INFO("yaw from joint: %f -- pitch_from_joint: %f", yaw_from_joint_,pitch_from_joint_);
		pan_pos = - (yaw_from_joint_ + pan_pos);
		tilt_pos = - (pitch_from_joint_ + tilt_pos);
		ROS_INFO("tilt_pos sent: %f",tilt_pos);
		if (abs(pan_pos) > 1.6)
		{
			pan_pos= 0;
		}
		if (abs(tilt_pos) > 1.6)
		{
			tilt_pos = 0;
		}
		message.position[0]= pan_pos;
		message.position[1]= tilt_pos;
		message.velocity[0]= abs(pan_pos)*0.8;
		message.velocity[1]= abs(tilt_pos)*1.3;
		ROS_ERROR("KALMAN MOVE: pan: %f pan_vel: %f  tilt: %f tilt_vel %f",message.position[0], message.velocity[0],message.position[1],message.velocity[1]);

	}

	    // * Draws for the Viewer
	    // * Velocity vector, face rectangle and face center



	    int pred_x = int(detected_face_roi_.x) + int(detected_face_roi_.width)/2;
	    int pred_y = int(detected_face_roi_.y) + int(detected_face_roi_.height)/2;

	    cv::Point pred_kal = cv::Point(pred_x, pred_y);
	    cv::Point tip_u_vel = cv::Point(pred_kal.x + (kalman_filter_.statePost.at<float>(2,0))/1., pred_kal.y);
	    cv::Point tip_v_vel = cv::Point(pred_kal.x, pred_kal.y + (kalman_filter_.statePost.at<float>(3,0))/1.);
	    //Draw face center and rectangle
	 //   ROS_INFO("float(0)= %f --- float(1)= %f",kalman_estimate_state_.at<float>(0),kalman_estimate_state_.at<float>(1));
	    cv::Rect prevision(kalman_filter_.statePost.at<float>(0),kalman_filter_.statePost.at<float>(1),detected_face_roi_.width,detected_face_roi_.height);

	    cv::circle(image_received, pred_kal,3,cv::Scalar(0,0,255), 3);
		cv::rectangle(image_received, detected_face_roi_, cv::Scalar(255,0,0), 2);
		cv::rectangle(image_received, prevision, cv::Scalar(0,255,0), 2);
	    if(tip_u_vel.x >= 0 && tip_u_vel.y >= 0 && tip_u_vel.x < image_received.cols && tip_u_vel.y < image_received.rows)
	    {
	    	cv::line(image_received, pred_kal, tip_u_vel, cv::Scalar(0,255,0), 2);
	    }
	    if(tip_v_vel.x >= 0 && tip_v_vel.y >= 0 && tip_v_vel.x < image_received.cols && tip_v_vel.y < image_received.rows)
	    {
	    	cv::line(image_received, pred_kal, tip_v_vel, cv::Scalar(255,0,0), 2);
	    }


	    cv_bridge::CvImagePtr cv_ptr2;
	    try
	    {
	      cv_ptr2 = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::RGB8);
	    }
	    catch (cv_bridge::Exception& e)
	    {
		    ROS_ERROR("cv_bridge exception: %s", e.what());
		    return;
	    }

	    int text = detected_face_roi_.x;

	//    std::string ciao = std::to_string(5);

	    cv_ptr2->image = image_received;
	    cv::putText(cv_ptr2->image,std::to_string(text),cv::Point(detected_face_roi_.x,detected_face_roi_.y),5,1.0,textColor_);
	    //cv::putText(cv_ptr2->image, x_pos, cvPoint( 10, 50), &font_, textColor_);

	    cv::imshow("Input", cv_ptr2->image);
	    cvWaitKey(1);






    /*
     * Publish joint state
     */
#ifdef marco_debug
	ROS_INFO("publishing: pan_pos: %lg --- tilt_pos: %lg",message.position[0],message.position[1]);
#endif
	joint_pub_.publish(message);
	//ROS_INFO("Message published");
	sendBaseVelocity(head_distance);

    /*
     * Update Haar cascade use frequency for Dynamic Haar Cascade
     */
    if(dynamic_check_haar_)
    {
    	//ROS_INFO("dynamic check haar");
	if(undetected_count_ > 10)
	{
		check_Haar_ = 2;
		cam_shift_detected_count_ = 0;
		cam_shift_undetected_count_ = 0;
	}
	else if(cam_shift_undetected_count_>3)
	{
		check_Haar_/=2;
    	if(check_Haar_<2)
    		check_Haar_ = 2;
    }

   	if(cam_shift_detected_count_>10)
    {
    	check_Haar_+=5;
    	if(check_Haar_>100)
    		check_Haar_ = 100;
    }


    	track_object_= check_Haar_ ;
    }


    //ROS_INFO("refreshing face detected bool");
	/*
	 * Refresh face_detected bool to use Haar Cascade once in a while
	 */
    if(loop_counter_%check_Haar_==0)
    	face_detected_bool_ = false;

    if(loop_counter_%check_track_obj_==0)
    	track_object_ = false;

    loop_counter_++;


}

void FaceDetector::setFaceClassifierPath(std::string face_classifier_path)
{
	face_classifier_.load(face_classifier_path);
}


void FaceDetector::initializeKalmanFilter()
{
	kalman_filter_ = cv::KalmanFilter(4, 2, 0);


	kalman_filter_.transitionMatrix = (cv::Mat_<float>(4,4) <<  1,0,1,0,
																0,1,0,1,
																0,0,1,0,
																0,0,0,1);


	setIdentity(kalman_filter_.measurementMatrix);
	setIdentity(kalman_filter_.processNoiseCov, cv::Scalar::all(1e-4));
	setIdentity(kalman_filter_.measurementNoiseCov, cv::Scalar::all(1e-1));
	setIdentity(kalman_filter_.errorCovPost, cv::Scalar::all(.1));

}

unsigned int FaceDetector::detectFacesHaar(cv::Mat image, std::vector<cv::Rect> &faces, cv::CascadeClassifier classifier)
{
	classifierDetect(image,faces,classifier);
	return faces.size();
}

unsigned int FaceDetector::detectFaceCamShift(cv::Mat img)
{
#ifdef marc_debug
	ROS_INFO("detectFaceCamShift");
#endif
	/* Variables for CAM Shift */
	cv::Mat hsv = cv::Mat(img.size(), CV_8UC3 );
	cv::Mat mask = cv::Mat(img.size(), CV_8UC1);
	cv::Mat grayscale;
	cv::Mat backproject = cv::Mat( img.size(), CV_8UC1 );
	cv::Mat histimg = cv::Mat::zeros( img.size(), CV_8UC3);
	cv::Mat hue[hsv.channels()];
	cv::MatND hist;
	int vmin = 10, vmax = 256, smin = 30;
	int n = 16;
	float hranges_arr[] = {0,180};
	const float* hranges = hranges_arr;
	int channels[]={0};
	cv::Rect track_window, face_roi;
	cv::RotatedRect track_box;

	int mean_score_threshold = 70; //Minimum value of mean score of the CAM Shift. Value between 0 and 100
	float max_face_deslocation = detected_face_roi_.height/3.;

	unsigned int max_distance_width = detected_face_roi_.width/3.;
	unsigned int max_distance_height = detected_face_roi_.height/3.;

	/*********************/


	cv::cvtColor( img, hsv, CV_RGB2HSV );
	cv::cvtColor(img, grayscale, CV_RGB2GRAY);
	cv::inRange(hsv, cv::Scalar(0,smin,MIN(vmin,vmax),0),cv::Scalar(180,256,MAX(vmin,vmax),0), mask );
	split( hsv,hue);
	face_roi = detected_face_roi_;


	if(!track_object_) //Select a region with the face color skin and calc histogram
	{
		double max_val = 0.0;

		//ROI selection is a bit different now and uses operator ()

		/*
		 * little_face is a ROI that contains exclusively the color skin
		 */
		cv::Rect little_face;

		little_face = face_roi;

		little_face.width = face_roi.width -face_roi.width/3;
		little_face.height = face_roi.height -face_roi.height/3;
		little_face.x += face_roi.width/(2.*3.);
		little_face.y += face_roi.height/(2.*3.);

		cv::Mat hueRoi=hue[0](little_face);
		cv::Mat maskRoi=mask(little_face);
		cv::calcHist( &hueRoi,1,channels, maskRoi , hist, 1 ,  &n  ,  &hranges, true, 0 );
		cv::minMaxLoc(hist, 0, &max_val, 0, 0);

		float scale =max_val ? 255. / max_val : 0.;
		hist.convertTo(hist,hist.type(),scale,0);
		track_window = face_roi;
		histimg.setTo(0);
	}


	cv::calcBackProject(hue,1,channels,hist,backproject,&hranges,1,true);
	cv::bitwise_and( backproject, mask, backproject);

	//Use camshift over computed histogram
	track_box = cv::CamShift( backproject, track_window, cv::TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));


	if(track_box.size.height>0 && track_box.size.width > 0
			&& track_box.center.x>0 && track_box.center.y >0)
		face_roi = track_box.boundingRect();

	face_roi.height = face_ratio_*face_roi.width;

	face_roi.x = min(face_roi.x, img.cols);
	face_roi.y = min(face_roi.y, img.rows);

	face_roi.width = min(img.cols-face_roi.x-3, face_roi.width);
	face_roi.height = min(img.rows-face_roi.y-3, face_roi.height);


	face_roi.height = max(face_roi.height, 0);
	face_roi.width = max(face_roi.width, 0);

	face_roi.x = max(face_roi.x, 0);
	face_roi.y = max(face_roi.y, 0);


	if(face_roi.height <= 0 || face_roi.width <= 0) //If face ROI has invalid dimensions
		return 1;

	/*
	 * This is used to check mean score of CAM shift selected ROI
	 */
	double mean_score = 0;

	if(face_roi.x > 0 && face_roi.y > 0 && face_roi.width>20 && face_roi.height>20)
	{
		cv::Mat temp = backproject(face_roi);
		for(int r = 0; r < temp.rows; r++)
			for(int c = 0; c < temp.cols; c++)
				mean_score+= temp.at<unsigned char>(r,c);

		mean_score = mean_score/double(temp.rows*temp.cols);
	}


	if(mean_score<mean_score_threshold) //Let's see if CAM Shift respects mean score threshold
	{
		face_detected_bool_ = false;
		return 1;
	}

	//If face position have moved considerably, ignore CAM shift
	if(euclidDist(cv::Point2f(face_roi.x + face_roi.width/2.,face_roi.y+face_roi.height/2.),
			cv::Point2f(detected_face_roi_.x+detected_face_roi_.width/2,detected_face_roi_.y+detected_face_roi_.height/2.)) > max_face_deslocation)
	{
		face_detected_bool_ = false;

		return 1;
	}


	//This is to avoid big increases in the size of the detected_face_roi
	if(abs(face_roi.width - detected_face_roi_.width)> max_distance_width
			|| abs(face_roi.height - detected_face_roi_.height)> max_distance_height)
	{
		face_roi.x = face_roi.x + face_roi.width/2. -detected_face_roi_.width/2.;
		face_roi.y = face_roi.y + face_roi.height/2. -detected_face_roi_.height/2.;

		face_roi.width = detected_face_roi_.width;
		face_roi.height = detected_face_roi_.height;

	}

	//Check if Face ROI respects image's dimensions
	if(face_roi.x<0 || face_roi.y < 0
	|| (face_roi.x+face_roi.width)>= img.cols || (face_roi.y+face_roi.height)>= img.rows)
		return 1;


	//CAM Shift have succesfully found face
	detected_face_roi_ = face_roi;
	detected_face_ = img(detected_face_roi_);
	face_detected_bool_ = true;
	return 0;
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
	float linear= (head_distance - desired_distance_);
	float angular = yaw_from_joint_;
	linear*=0.5;
	angular*=0.5;
	if (linear > 0.2) linear = 0.2;
	if (linear < -0.2) linear = -0.2;
	if (angular > 0.5) angular = 0.5;
	if (angular < -0.5) angular = -0.5;
	message.linear.x=linear;
	message.linear.y=0;
	message.linear.z=0;
	message.angular.x=0;
	message.angular.y=0;
	message.angular.z=angular;
	base_control_pub_.publish(message);
	//ROS_INFO("Head Distance: %f",head_distance);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "qbo_face_tracking");

	FaceDetector face_detector;

	ros::spin();

	return 0;
}
