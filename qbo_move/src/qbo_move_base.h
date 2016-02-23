/*
 *
 * Authors:  Giuseppe Grande <grande@turintech.it>, Ada Panarese <panarese@turintech.it>
 */

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

#define OBSTACLE_DISTANCE 0.5

typedef struct {
	float x;
	float z;
} took_vel;

class MoveBase
{
private:
	void onInit();

	ros::NodeHandle private_nh_;

	/*
	 * ROS Subscribers
	 */
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
	void twistCallback(const geometry_msgs::TwistConstPtr& twist_vel);
	void floorSensorCallback(const sensor_msgs::PointCloudConstPtr& floor_sensor);
	void rightSensorCallback(const sensor_msgs::PointCloudConstPtr& right_sensor);
	void leftSensorCallback(const sensor_msgs::PointCloudConstPtr& left_sensor);
	void wheelCallback(const geometry_msgs::Point32ConstPtr& wheel_pos);
	void headCallback(const std_msgs::Float32ConstPtr& head_pos);

	/*
	 * Functions to set and delete ROS parameters
	 */
	void setROSParams();
	void deleteROSParams();

	void checkOtherSensor(int obstacle, string other);
	void putOrthogonal(float teta);
	float controlPID(float x, float ix, float dx, float Kp, float Ki, float Kd);

	void safePublish(geometry_msgs::Twist);

	bool is_obstacle;                                //ostacolo visto da entrambi i sensori a ultrasuoni
	bool was_obstacle;                               //non c'è ostacolo ma c'era fino alla iterazione precedente
	bool right_obstacle;                             //il sensore destro vede un ostacolo
	bool left_obstacle;                              //il sensore sinistro vede un ostacolo
	float z_step;                                   //angolo che viene valorizzato mentre cerco il percorso per evitare l'ostacolo
	float z_obst;                                   //angolo che diventa il riferimento appena vedo l'ostacolo
	float yaw_reset;                                //angolo di cui il robot si deve girare una volta superato l'ostacolo
	float kp_yaw_;
	float ki_yaw_;
	float kd_yaw_;
	float distance_from_obstacle;                    //distanza fra il robot e l'ostacolo
	float distance_from_head;                        //distanza fra il robot e il volto agganciato
	float obst_to_goal;                              //distanza fra l'ostacolo e il volto agganciato
	float route1;                                    //segmento che il robot DOVREBBE percorrere per aggirare l'ostacolo a destra
	float route2;                                    //segmento che il robot DOVREBBE percorrere per aggirare l'ostacolo a sinistra
	float route;                                     //segmento che il robot deve percorrere per aggirare l'ostacolo
	took_vel speed;                                  //velocità presa (velocità che mi arriva)
	float right_distance;                            //distanza dell'ostacolo vista dal sensore destro
	float left_distance;                             //distanza dell'ostacolo vista dal sensore sinistro
	float sensor_distance;                           //distanza fra i sensori
	float difference_distance;                       //differenza della distanza vista dai sensori
	float teta;                                      //angolo di cui deve ruotare il robot per mettersi perpendicolare all'ostacolo
	bool completed;                                  //indica se sono state eseguite entrambe le callback dei sensori
	int cycle_right;
	int cycle_left;
	bool reg_pos;
	bool floor_obstacle;                             //indica se c'è un ostacolo basso (sensore a infrarossi)
	float x_prev;
	float y_prev;
	float z_prev;
	float x_flag_prev;
	float y_flag_prev;
	bool flag;
	float flag_distance;
	float first_call;
	bool stop;
	string rotation;
	bool rotation_right_left;
	bool rotation_check;
	float alfa;
	int sensor_cycle;
	boost::mutex mutex;
	bool semaforo;
	ros::Time begin;
	ros::Duration range;
	bool right_all_obstacle;
	bool left_all_obstacle;
	bool half_step;
	float well_threshold;
	float hill_threshold;
	float left_sensor_;
	float right_sensor_;
	float floor_sensor_;


public:
	MoveBase();
	~MoveBase();
};

#endif

