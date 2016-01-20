/*
 * Software License Agreement (GPLv2 License)
 * 
 * Copyright (c) 2012 Thecorpora, S.L.
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
 * Author: Arturo Bajuelos <arturo@openqbo.com>
 */

#include "face_follower.h"


FaceFollower::FaceFollower()
{
	ROS_INFO("Initializing Qbo face following");
	onInit();
	ROS_INFO("Ready for face following. Waiting for face positions");
}

FaceFollower::~FaceFollower()
{
	deleteROSParams();
	printf("Qbo face tracking successfully ended\n");
}



void FaceFollower::setROSParams()
{
	//Set default value for move base boolean
	private_nh_.param("/qbo_face_following/move_base", move_base_bool_, false);
	
	//set default value for move head boolean
	private_nh_.param("/qbo_face_following/move_head", move_head_bool_, true);

	//Parameters that define the movement when Qbo is searching for faces
	private_nh_.param("/qbo_face_following/search_min_pan", search_min_pan_, -0.3);
	private_nh_.param("/qbo_face_following/search_max_pan", search_max_pan_, 0.3);
	private_nh_.param("/qbo_face_following/search_pan_vel", search_pan_vel_, 0.3);
	//private_nh_.param("/qbo_face_following/search_max_tilt", search_max_tilt_, 0.7);
	//private_nh_.param("/qbo_face_following/search_min_tilt", search_min_tilt_, 0.7);
	private_nh_.param("/qbo_face_following/search_max_tilt", search_max_tilt_, 0.5);
	private_nh_.param("/qbo_face_following/search_min_tilt", search_min_tilt_, 0.5);
	private_nh_.param("/qbo_face_following/search_tilt_vel", search_tilt_vel_, 0.3);
	private_nh_.param("/qbo_face_following/desired_distance", desired_distance_, 0.2);
	//private_nh_.param("/qbo_face_following/desired_distance", desired_distance_, 5.0);
	private_nh_.param("/qbo_face_following/send_stop", send_stop_, true);
}

void FaceFollower::deleteROSParams()
{
	private_nh_.deleteParam("/qbo_face_following/move_base");
	private_nh_.deleteParam("/qbo_face_following/move_head");
	private_nh_.deleteParam("/qbo_face_following/search_min_pan");
	private_nh_.deleteParam("/qbo_face_following/search_max_pan");
	private_nh_.deleteParam("/qbo_face_following/search_pan_vel");
	private_nh_.deleteParam("/qbo_face_following/search_max_tilt");
	private_nh_.deleteParam("/qbo_face_following/search_min_tilt");
	private_nh_.deleteParam("/qbo_face_following/search_tilt_vel");
	private_nh_.deleteParam("/qbo_face_following/desired_distance");
	private_nh_.deleteParam("/qbo_face_following/send_stop");

}

void FaceFollower::onInit()
{
	/*
	 * Set node's subscriptions
	 */
	//camera_info_sub_=private_nh_.subscribe<sensor_msgs::CameraInfo>("/stereo/left/camera_info",10,&FaceFollower::cameraInfoCallback, this);
	camera_info_sub_=private_nh_.subscribe<sensor_msgs::CameraInfo>("/usb_cam/camera_info",10,&FaceFollower::cameraInfoCallback, this);
    
    


	/*
	 * Set node's publishers
	 */
	joint_pub_ = private_nh_.advertise<sensor_msgs::JointState>("/cmd_joints", 1000); //To move the head
	base_control_pub_=private_nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1); //to move robot's base
	//face_detected_pub_=private_nh_.advertise<std_msgs::Bool>("/face_detected",1);

	setROSParams();

	yaw_from_joint_=0;
	pitch_from_joint_=0;


	//TODO - Read from rosparam server this value
	min_pitch_ = -0.5;

	face_detected_count_=0;
	time_limit_=ros::Duration(20);

	//TODO - Should be ROS parameters
	/*
	 * Setting PIDs values
	 */
	//For head's pan movement
	u_act_=0;
	u_prev_=0;
	diff_u_=0;
	kp_u_=1; //0.0066
	ki_u_=0.1;
	kd_u_=0.00;
	

	//For head's tilt movement
	v_act_=0;
	v_prev_=0;
	diff_v_=0;
	kp_v_=1;
	ki_v_=0.1;
	kd_v_=0.00;
	

	//For base's linear movement
	//ros::Time delta;
	//ros::Time T;
	distance_act_=0;
	distance_prev_=0;
	diff_distance_=0;
	kp_distance_=0.3;
	//kp_distance_=0.02;
	ki_distance_=0;
	kd_distance_=0.001;
//	max_linear_speed_=0.001;

	//For base's angular movement
	yaw_act_=0;
	yaw_prev_=0;
	diff_yaw_=0;
	kp_yaw_=1.3;
	ki_yaw_=0;
	kd_yaw_=0.0;
//	max_angular_speed_=0.001;

	/*
	 * Initialize values
	 */
	image_width_ = 320;
	image_height_ = 240;

	//////////////////////////////////////////////////////////////////////////////////////// Grande begin
	e_hor_act = 0;
	e_vert_act = 0;
	e_hor_prev = 0;
	e_vert_prev = 0;
	pan_pos = 0;
	e_hor_act_der = 0;
	e_hor_act_int = 0;
	e_vert_act_der = 0;
	e_vert_act_int = 0;
	left_to_right = 0;
	up_to_down = 0;
	t_prev = 0;
	///////////////////////////////////////////////////////////////////////////////////////// Grande end
}


void FaceFollower::jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state)
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


void FaceFollower::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info)
{
//	ROS_ERROR("CAMERA_INFO_CALLBAK");
	if(p_.data==NULL)
	{
		cv::Mat p=cv::Mat(3,4,CV_64F);
		for (int i=0;i<3;i++)
		{
			for (int j=0;j<4;j++)
			{
				p.at<double>(i,j)=info->P[4*i+j];
			}
		}
		p(cv::Rect(0,0,3,3)).convertTo(p_,CV_32F);

		joint_states_sub_=private_nh_.subscribe<sensor_msgs::JointState>("/joint_states",10,&FaceFollower::jointStateCallback, this);
		face_pos_sub_ = private_nh_.subscribe<qbo_face_msgs::FacePosAndDist>("/qbo_face_tracking/face_pos_and_dist", 10, &FaceFollower::facePositionCallback, this);
	}

}

/*
 * Callback of the face position and size
 * This is the main callback of the node that moves the robot's head and base
 * to follow the face
 */
void FaceFollower::facePositionCallback(const qbo_face_msgs::FacePosAndDistConstPtr& head_pos_size)
{
//    ROS_ERROR("facepositionCallback called");
	image_width_ = head_pos_size->image_width;
	image_height_ = head_pos_size->image_height;
//	ROS_ERROR("FACE_POSITION_CALLBACK");
	//Refresh status of move_base

	private_nh_.getParam("/qbo_face_following/move_base", move_base_bool_);
	private_nh_.getParam("/qbo_face_following/move_head", move_head_bool_);

	if(head_pos_size->face_detected) //If a face was detected by the face detector
	{
		/*
		 * Velocities for head movement
		 */
		float pan_vel;
		float tilt_vel;
/****************** MARCO COMMENTED OUT ***************************
		face_detected_count_++;

		if(face_detected_count_ > 10)
		{
		    ros::Time now=ros::Time::now();
		    if ((now-sent_)>time_limit_)
		    {
			    std_msgs::Bool msg;
			    msg.data=true;
			    
			   // face_detected_pub_.publish(msg);
			    sent_=ros::Time::now();
			    face_detected_count_=0;
		    }
		}
***************** MARCO END ************************************/

		/*
		 * HEAD MOVEMENT
		 */

		u_act_=head_pos_size->u;
		diff_u_=u_act_-u_prev_;
		//pan_vel=controlPID(u_act_,0,diff_u_,kp_u_,ki_u_,kd_u_);  Grande commented
		//u_prev_=u_act_; Grande commented

		v_act_=head_pos_size->v;
		diff_v_= v_act_-v_prev_;
		//tilt_vel=controlPID(v_act_,0,diff_v_,kp_v_,ki_v_,kd_v_); Grande commented
		//v_prev_=v_act_; Grande commented

		//////////////////////////////////////////////////////////////////////////////////////// Grande begin
		/*
		e_hor_act = u_act_ - u_prev_;
		if(t_prev == 0) {
			t_prev = ros::Time::now().toNSec() - 100000;
		}
		e_hor_act_der = (e_hor_act - e_hor_prev)/(ros::Time::now().toNSec() - t_prev);
		e_hor_act_int = e_hor_act*(ros::Time::now().toNSec() - t_prev) + e_hor_act_int;

		left_to_right=control_PID(e_hor_act,e_hor_act_int,e_hor_act_der,kp_u_,ki_u_,kd_u_);
		e_hor_prev = e_hor_act;
		u_prev_= u_act_;

		e_vert_act = v_act_ - v_prev_;
		if(t_prev == 0) {
			t_prev = ros::Time::now().toNSec() - 100000;
		}
		deltaT = ros::Time::now().toNSec() - t_prev;
		e_vert_act_der = (e_vert_act - e_vert_prev)/(ros::Time::now().toNSec() - t_prev);
		e_vert_act_int = e_vert_act*(ros::Time::now().toNSec() - t_prev) + e_vert_act_int;
		up_to_down=control_PID(e_vert_act,e_vert_act_int,e_vert_act_der,kp_u_,ki_u_,kd_u_);
		e_vert_prev = e_vert_act;
		t_prev = ros::Time::now().toNSec();
		v_prev_= v_act_;
		*/
		///////////////////////////////////////////////////////////////////////////////////////// Grande end

		///////////////////////////////////////////////////////////////////////////////Grande begin
		pan_vel = atan2((diff_u_ - p_.at<float>(0,2)),p_.at<float>(0,0));
		tilt_vel = atan2((diff_v_ - p_.at<float>(1,2)),p_.at<float>(1,1));
		///////////////////////////////////////////////////////////////////////////////Grande end

		if(move_head_bool_)
		{	
			//setHeadPositionToFace(v_act_,u_act_,tilt_vel,pan_vel); Grande: commented because below the new version
			//setHeadPositionToFace(up_to_down,left_to_right,1,1); //Grande: new version
			setHeadPositionToFace(head_pos_size->v,head_pos_size->u,tilt_vel,pan_vel); // MARCO
			//ROS_ERROR("FOLLOWING HEAD");
		}

		if(!move_base_bool_)
			return;

		/*
		 * BASE MOVEMENT
		 */

		/*
		 * Velocities for base movement
		 */
		 

		float linear_vel = 0;
		float angular_vel = 0;

		
		//ROS_INFO("*******distance to head: %f",head_pos_size->distance_to_head);
		distance_act_ = (head_pos_size->distance_to_head)-desired_distance_;
		diff_distance_=distance_act_-distance_prev_;
		//linear_vel=controlPID(distance_act_,0,diff_distance_,kp_distance_,ki_distance_,kd_distance_);
		linear_vel=controlPID(distance_act_,0,diff_distance_,kp_distance_,ki_distance_,0.0);
//		ROS_ERROR("distance to head: %lg - desired_distance: %lg", head_pos_size->distance_to_head, desired_distance_);
//		ROS_ERROR("distance_act: %lg, diff_distance: %lg, linear_vel: %lg", distance_act_,diff_distance_,linear_vel);
		distance_prev_=distance_act_;

		bool head_near_to_border = ((head_pos_size->v)+(head_pos_size->image_height/2))<100;

		if(pitch_from_joint_ <= min_pitch_ && head_near_to_border && linear_vel>0.0)
		{
			ROS_INFO("Head near border\n");
			linear_vel = 0.0;
		}

//		if (linear_vel>-0.1 && linear_vel < 0.1) linear_vel=0;
//		else if (linear_vel>3.0) linear_vel=2.0;
//		else if (linear_vel<-3.0) linear_vel=-2.0;
//
//		else if (linear_vel<-0.6) linear_vel=-1.3;
//		else if (linear_vel>0.6) linear_vel=1.3;

		//TODO - Use PID control
		yaw_act_ = yaw_from_joint_;
		diff_yaw_=yaw_act_-yaw_prev_;
		angular_vel=controlPID(yaw_act_,0,diff_yaw_,kp_yaw_,ki_yaw_,kd_yaw_);
		yaw_prev_=yaw_act_;

//		if(yaw_from_joint_<-0.3)
//		  angular_vel = -1.1;
//		else if(yaw_from_joint_>0.3)
//		  angular_vel = 1.1;
//		else if(yaw_from_joint_>0.1 && yaw_from_joint_<=0.3)
//			angular_vel = 0.9;
//		else if(yaw_from_joint_<-0.1 && yaw_from_joint_>=-0.3)
//			angular_vel = -0.9;
//		else
//			angular_vel=0;

		//linear_vel=linear_vel/10;
/******************************
		if(linear_vel!=linear_vel)
		{
			 linear_vel=0.0;
			ROS_INFO("*********** Linear_vel is NaN");
		}
		else
		{
			ROS_INFO("--------- Linear_vel is not a Nan");
		}
/*****************************/

		////////////////////////////////////////////////////////////////Grande begin
		if(linear_vel!=linear_vel)
		{
			 linear_vel=0.0;
			ROS_INFO("*********** Linear_vel is NaN");
		}
		else
		{
			ROS_INFO("--------- Linear_vel is not a Nan");
		}
		if(angular_vel!=angular_vel)
		{
			angular_vel=0.0;
			ROS_INFO("*********** angular_vel is NaN");
		}
		else
		{
			ROS_INFO("--------- angular_vel is not a Nan");
		}
		////////////////////////////////////////////////////////////////Grande end
		ROS_INFO("++++++++Moving base: linear velocity: %lg, angular vel: %lg",linear_vel,angular_vel);
		sendVelocityBase(linear_vel,angular_vel);  //Grande decommented


	}
	else //Qbo didn't detect any face and it's searching for one
	{

		//if(face_detected_count_ > 0) face_detected_count_--;
		//face_detected_count_=0;

		srand(time(NULL));
		float rand_tilt = search_min_tilt_+((search_max_tilt_-search_min_tilt_)/20.0) * double(rand()%20);
		float rand_pan = search_min_pan_+((search_max_pan_-search_min_pan_)/20.0) * double(rand()%20);


		ROS_INFO("Randomly moving head: pos(%lg, %lg) and vel(%lg, %lg)", rand_tilt, rand_pan, search_tilt_vel_, search_pan_vel_);

		if(move_head_bool_)
		{
		//	ROS_ERROR("move_head_bool_ is true");
			setHeadPositionGlobal(rand_tilt, rand_pan, 0.5, 0.5);
		//	setHeadPositionGlobal(rand_tilt, rand_pan, 0.0001, 0.0001); // MARCO
		}
		//TODO - Analyse this
		if(move_base_bool_ && send_stop_)  //Grande decommented
			sendVelocityBase(0,0);         //Grande decommented
	}


}
/*
 * Given the head position in the image and the velocities to move the head, this will move the head with the
 * given velocity to the destinated position
 */
void FaceFollower::setHeadPositionToFace(float pos_updown, float pos_leftright, float vel_updown, float vel_leftright)
{
	if(p_.data == NULL)
		return;
//    ROS_ERROR("SETHEADPOSITIONTOFACE CALLED");
//	printf("Pos_left_right: %lg\n",pos_leftright);
	
	//float pan_pos,tilt_pos;  Grande: defined in FaceFollower.h as member of the class

	//pan_pos = pos_leftright+ image_width_/2;
	//tilt_pos = pos_updown+ image_height_/2;
	pan_pos = pos_leftright;
	tilt_pos = pos_updown;

	pan_pos = atan2((pan_pos - p_.at<float>(0,2)),p_.at<float>(0,0));
	tilt_pos = atan2((tilt_pos - p_.at<float>(1,2)),p_.at<float>(1,1));

//	printf("Angle pos: %lg. Yaw from joint: %lg \n", pan_pos, yaw_from_joint_);


	//pan_pos = yaw_from_joint_- pan_pos;              //Grande:commented because the robot is stopped, it was minus
	//tilt_pos = tilt_pos + pitch_from_joint_;        //Grande:commented because the robot is stopped


//	printf("SUM: %lg\n", pan_pos);
	//printf("Yaw from joint: %lg\n", yaw_from_joint_);
	//printf("Image width: %d\n", image_width_);
	//printf("Image height: %d\n", image_height_);

	if(vel_leftright<0)
		vel_leftright=-vel_leftright;

	if(vel_updown<0)
		vel_updown=-vel_updown;

	

	ROS_INFO("Moving head to face: pos(%lg, %lg) and vel(%lg, %lg)", tilt_pos, pan_pos, vel_updown, vel_leftright);
	
	sensor_msgs::JointState joint_state;
	int servos_count=2;
	joint_state.name.resize(servos_count);
	joint_state.position.resize(servos_count);
	joint_state.velocity.resize(servos_count);

	joint_state.name[0]="head_pan_joint";	//izquierda-derecha

	//if(pos_leftright>0)                   //Grande commented
	//	joint_state.position[0]=-1.7;     //Grande commented
	//else                                  //Grande commented
	//	joint_state.position[0]=1.7;      //Grande commented

 	joint_state.position[0]= -pan_pos;  //Grande decommented // MARCO ADDED A MINUS
	joint_state.velocity[0]=vel_leftright;

	joint_state.name[1]="head_tilt_joint";	//arriba-abajo

	//if(pos_updown>0)                   //Grande commented
	//	joint_state.position[1]=1.5;   //Grande commented
	//else                               //Grande commented
	//	joint_state.position[1]=-1.5;  //Grande commented


	joint_state.position[1]=-tilt_pos;   //Grande decommented // MARCO ADDED A MINUS

	joint_state.velocity[1]=vel_updown;

	joint_state.header.stamp = ros::Time::now();


	//publish
	joint_pub_.publish(joint_state);
}


void FaceFollower::setHeadPositionGlobal(float pos_updown, float pos_leftright, float vel_updown, float vel_leftright)
{
    if(pos_leftright<0)
        pos_leftright=-1.5;
    else if(pos_leftright>0)
        pos_leftright=1.5;
/*
    if(pos_updown<0)
        pos_updown=-1.5;
    else if(pos_updown>0)
        pos_updown=1.5;
*/
	if(vel_leftright<0)
		vel_leftright=-vel_leftright;
	if(vel_updown<0)
		vel_updown=-vel_updown;

	sensor_msgs::JointState joint_state;

	int servos_count=2;

	joint_state.name.resize(servos_count);
	joint_state.position.resize(servos_count);
	joint_state.velocity.resize(servos_count);

	joint_state.name[0]="head_pan_joint";
	joint_state.position[0]=pos_leftright;
	joint_state.velocity[0]=vel_leftright;


	joint_state.name[1]="head_tilt_joint";
	joint_state.position[1]=pos_updown;
	joint_state.velocity[1]=vel_updown;

	joint_state.header.stamp = ros::Time::now();
	//publish
	joint_pub_.publish(joint_state);
}

/*
 * Move Qbo's base according to the given linear and angular velocity
 */
void FaceFollower::sendVelocityBase(float linear_vel, float angular_vel)
{
	if (linear_vel > 0.2) linear_vel = 0.2;
	if (linear_vel < -0.2) linear_vel = -0.2;
	if (angular_vel > 0.5) angular_vel = 0.5;
	if (angular_vel < -0.5) angular_vel = -0.5;
	geometry_msgs::Twist velocidad_base;
	velocidad_base.linear.x=linear_vel;
	velocidad_base.linear.y=0;
	velocidad_base.linear.z=0;

	velocidad_base.angular.x=0;
	velocidad_base.angular.y=0;
	velocidad_base.angular.z=angular_vel;
	//publish
	base_control_pub_.publish(velocidad_base);   //Grande decommented
/*
	if (linear_vel!=linear_vel)
	{	
		 ROS_WARN("Send Velocity Base has linear NaN value!");
		velocidad_base.linear.x=0.0;
		velocidad_base.angular.z=0.0
		//base_control_pub_.publish(velocidad_base);
	}
	else
	{
		if(angular_vel!=angular_vel)
		{
			ROS_WARN("Send Velocity Base has angular NaN value!");
			velocidad_base.linear.x=0.0;
			velocidad_base.angular.z=0.0;
			//base_control_pub_.publish(velocidad_base);
		}
		else
		{
			ROS_INFO("Linear vel: %lg, Angular vel: %lg", linear_vel, angular_vel);
			//base_control_pub_.publish(velocidad_base);
		}
	} */

}

float FaceFollower::controlPID(float x, float ix, float dx, float Kp, float Ki, float Kd)
{
	float result;
	try
	{
		result=Kp*x+Ki*ix+Kd*dx;
		if (result!=result) ROS_WARN ("PID CONTROLLER VALUE IS NaN");
	}
	catch(std::exception& e)
	{
		ROS_ERROR("PID controller returned a NaN value!");

	}
	return result;
}


//Grande: new PID controller
float FaceFollower::control_PID(float x, float ix, float dx, float Kp, float Ki, float Kd)
{
	float result;
	try
	{
		result=Kp*x;
		//result=Kp*x+Ki*ix+Kd*dx;
		if (result!=result) ROS_WARN ("PID CONTROLLER VALUE IS NaN");
	}
	catch(std::exception& e)
	{
		ROS_ERROR("PID controller returned a NaN value!");

	}
	return result;
}


/*
 * This function is executed when the node is killed
 * It restores the head to its default position
 */
void FaceFollower::headToZeroPosition()
{
	printf("Moving head to its default position\n");

	ros::start();

	sensor_msgs::JointState joint_state;
	int servos_count=2;
	joint_state.name.resize(servos_count);
	joint_state.position.resize(servos_count);
	joint_state.velocity.resize(servos_count);

	joint_state.name[0]="head_pan_joint";	//izquierda-derecha

	joint_state.position[0]=0.;
	joint_state.velocity[0]= 0.2;


	joint_state.name[1]="head_tilt_joint";	//arriba-abajo
	joint_state.position[1]=0.;
	joint_state.velocity[1]=0.2;

	joint_state.header.stamp = ros::Time::now();
	ros::NodeHandle private_nh_2;
	yaw_from_joint_ = 1.0;
	pitch_from_joint_= 1.0;

	ros::Publisher joint_pub_2 = private_nh_2.advertise<sensor_msgs::JointState>("/cmd_joints", 1);
	ros::Subscriber joint_states_=private_nh_2.subscribe<sensor_msgs::JointState>("/joint_states",10,&FaceFollower::jointStateCallback, this);

	ros::Time time_saved = ros::Time::now();
	ros::Duration time_diff;

	while(1 && ros::ok())
	{
		ros::spinOnce();
		//printf("Publishing\n");
		joint_state.header.stamp = ros::Time::now();
		joint_pub_2.publish(joint_state);

		//printf("Published\n");
		if(yaw_from_joint_ == 0 && pitch_from_joint_ == 0)
			break;

		time_diff = ros::Time::now()- time_saved;

		if(time_diff.toSec() >= 4.0)
			break;

	}
	printf("Face Following Node Successfully Ended!\n");

	printf("Qbo face following node successfully ended\n");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "qbo_face_following");


  FaceFollower ff;
  ros::spin();
  ff.headToZeroPosition();

  return 0;
}
