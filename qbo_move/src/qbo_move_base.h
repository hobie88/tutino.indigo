#ifndef MOVE_BASE_H
#define MOVE_BASE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <string>
#include <boost/thread/mutex.hpp>

using namespace std;

#define WAITING 5

typedef struct {
	float x;
	float y;
	float z;
} InitialPosition;

typedef struct {
	float x;
	float z;
} took_vel;   //velocità speditami dal master

class MoveBase
{
private:
	void onInit();

	ros::NodeHandle private_nh_;

	/*
	 * ROS Subscribers
	 */
	//ros::Subscriber joint_sub_;
	ros::Subscriber twist_sub_;
	ros::Subscriber floor_sensor_sub_;
	ros::Subscriber right_sensor_sub_;
	ros::Subscriber left_sensor_sub_;
	ros::Subscriber wheel_sub;
	ros::Subscriber head_sub;

	/*
	 * ROS Publishers
	 */
	ros::Publisher base_control_pub_;

	/*
	 * Callbacks
	 */
	//void jointCallback(const sensor_msgs::JointStateConstPtr& joint_state);
	void twistCallback(const geometry_msgs::TwistConstPtr& twist_vel);
	void floorSensorCallback(const sensor_msgs::PointCloudConstPtr& floor_sensor);
	void rightSensorCallback(const sensor_msgs::PointCloudConstPtr& right_sensor);
	void leftSensorCallback(const sensor_msgs::PointCloudConstPtr& left_sensor);
	//void wheelCallback(const nav_msgs::OdometryConstPtr& wheel_pos);
	void wheelCallback(const geometry_msgs::Point32ConstPtr& wheel_pos);
	void headCallback(const std_msgs::Float32ConstPtr& head_pos);

	/*
	 * Functions to set and delete ROS parameters
	 */
	void setROSParams();
	void deleteROSParams();

	void checkOtherSensor(int obstacle, string other);
	void setInitialPosition();
	void putOrthogonal(float teta);
	float controlPID(float x, float ix, float dx, float Kp, float Ki, float Kd);

	bool is_obstacle;                                //ostacolo visto da entrambi i sensori a ultrasuoni
	bool was_obstacle;                               //non c'è ostacolo ma c'era fino alla iterazione precedente
	bool right_obstacle;                             //il sensore destro vede un ostacolo
	bool left_obstacle;                              //il sensore sinistro vede un ostacolo
	//float yaw_from_joint;
	//float yaw_stop;
	//float yaw_start;
	float x_base;                                   //coordinata x della base
	float y_base;                                   //coordinata y della base
	float z_base;                                   //rotazione attorno all'asse z della base
	float z_step;                                   //angolo che viene valorizzato mentre cerco il percorso per evitare l'ostacolo
	float z_obst;                                   //angolo che diventa il riferimento appena vedo l'ostacolo
	//float yaw_sum;
	float yaw_reset;                                //angolo di cui il robot si deve girare una volta superato l'ostacolo
	int counter;
	float kp_yaw_;
	float ki_yaw_;
	float kd_yaw_;
	float distance_from_obstacle;                    //distanza fra il robot e l'ostacolo
	float distance_from_head;                        //distanza fra il robot e il volto agganciato
	float obst_to_goal;                              //distanza fra l'ostacolo e il volto agganciato
	float route;                                     //segmento che il robot deve percorrere per aggirare l'ostacolo
	ros::Time t_begin;                               //istante in cui vedo l'ostacolo la prima volta
	ros::Duration t_end;                             //tempo necessario a percorrere il tratto che mi permette di evitare l'ostacolo
	//bool pos_initial;                                //variabile che controlla se deve essere chiamata la setInitialPosition()
	InitialPosition init_pos;
	took_vel speed;                                  //velocità presa (velocità che mi arriva)
	float right_distance;                            //distanza dell'ostacolo vista dal sensore destro
	float left_distance;                             //distanza dell'ostacolo vista dal sensore sinistro
	float sensor_distance;                           //distanza fra i sensori
	float difference_distance;                       //differenza della distanza vista dai sensori
	float teta;                                      //angolo di cui deve ruotare il robot per mettersi perpendicolare all'ostacolo
	bool subscribed;                                 //indica se ho già registrato la wheelCallback
	bool completed;                                  //indica se sono state eseguite entrambe le callback dei sensori
	int cycle_right;
	int cycle_left;
	bool reg_pos;
	bool floor_obstacle;                             //indica se c'è un ostacolo basso (sensore a infrarossi)
	boost::mutex mutex;


public:
	MoveBase();
	~MoveBase();
};

#endif

