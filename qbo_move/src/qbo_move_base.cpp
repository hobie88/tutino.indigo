/*
 *
 * Authors:  Giuseppe Grande <grande@turintech.it>, Ada Panarese <panarese@turintech.it>
 */

#include "qbo_move_base.h"
#include <cmath>
#include <exception>
//#define DEBUG


MoveBase::MoveBase(): range(5.0)
{
	onInit();
}

MoveBase::~MoveBase()
{

}

void MoveBase::setROSParams()
{

}

void MoveBase::deleteROSParams()
{

}

void MoveBase::onInit()
{
	ROS_INFO("Initialization of qbo_move_base node");
	/*
	 * Set node's subscriptions
	 */
	//joint_sub_=private_nh_.subscribe<sensor_msgs::JointState>("/cmd_joints",10,&MoveBase::jointCallback, this);
	twist_sub_=private_nh_.subscribe<geometry_msgs::Twist>("/pre_cmd_vel",10,&MoveBase::twistCallback, this);
	floor_sensor_sub_=private_nh_.subscribe<sensor_msgs::PointCloud>("/distance_sensors_state/floor_sensor",10,&MoveBase::floorSensorCallback, this);
	right_sensor_sub_=private_nh_.subscribe<sensor_msgs::PointCloud>("/distance_sensors_state/front_right_srf10",10,&MoveBase::rightSensorCallback, this);

	left_sensor_sub_=private_nh_.subscribe<sensor_msgs::PointCloud>("/distance_sensors_state/front_left_srf10",10,&MoveBase::leftSensorCallback, this);
	head_sub = private_nh_.subscribe<std_msgs::Float32>("/head_distance",10,&MoveBase::headCallback, this);
	wheel_sub = private_nh_.subscribe<geometry_msgs::Point32>("/qbo_arduqbo/my_odom",10,&MoveBase::wheelCallback, this);

	/*
	 * Set node's publishers
	 */
	base_control_pub_ = private_nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000); //To move robot's base

	setROSParams();

	is_obstacle = false;
	was_obstacle = false;
	right_obstacle = false;
	left_obstacle = false;
	kp_yaw_= 0.0;
	ki_yaw_= 0.0;
	kd_yaw_= 0.4;
	sensor_distance = 0.22;
	difference_distance = 0.0;
	teta = 0.0;
	completed = false;
	cycle_right = 0;
	cycle_left = 0;
	reg_pos = false;
	floor_obstacle = false;
	first_call = true;
	rotation = "right_rotation";
	rotation_right_left = true;
	flag=false;
	stop = false;
	rotation_check = false;
	alfa = 0.0;
	z_step = 0.0;
	sensor_cycle = 0.0;
	semaforo = false;
	distance_from_head = 4*OBSTACLE_DISTANCE;
	right_all_obstacle = false;
	left_all_obstacle = false;
	half_step = false;
	well_threshold = 0.27;
	hill_threshold = 0.22;

}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


//la funzione checkOtherSensor verrà chiamata sia nella callback del sensore destro sia nella callback del sensore sinistro (sia per vedere se c'è un
//ostacolo visto da entrambi i sensori sia per vedere se nessuno dei due sensori vede un ostacolo): viene chiamata in entrambe le callback perché non
//so quale delle due callback viene chiamata per prima, quindi se viene chiamata prima la callback del sensore di destra quando questa callback
//chiamerà la checkOtherSensor non succederà niente, ma succederà qualcosa quando questa funzione verrà chiamata dalla callback del sensore di
//sinistra, dato che la callback del sensore di destra già è stata chiamata (e viceversa se viene chiamata prima la callback del sensore di sinistra)
void MoveBase::checkOtherSensor(int obstacle, string other) {

	if(semaforo == false) {
		if(obstacle == 1) {                  //uno dei due sensori ha visto l'ostacolo
			if(other == "right") {
				if(left_obstacle == true) {    //anche l'altro sensore ha visto l'ostacolo
					is_obstacle = true;
					if(was_obstacle == false) {
						difference_distance = right_distance - left_distance;
						teta = tan(difference_distance/sensor_distance);
						if((teta > 0.5) && (completed == true)) {
							putOrthogonal(teta);   //il robot si deve mettere perpendicolare all'ostacolo
							reg_pos = false;
							//checkOtherSensor(obstacle, other);
						} else if((teta <= 0.5) && (completed == true)) {
							reg_pos = true;
							//beta = z_now;
							distance_from_obstacle = right_distance;
						}
					}
				}
			} else if(other == "left") {
				if(right_obstacle == true) {    //anche l'altro sensore ha visto l'ostacolo
					is_obstacle = true;
					if(was_obstacle == false) {
						difference_distance = right_distance - left_distance;
						teta = tan(difference_distance/sensor_distance);
						if((teta > 0.5) && (completed == true)) {
							putOrthogonal(teta);   //il robot si deve mettere perpendicolare all'ostacolo
							reg_pos = false;
							//checkOtherSensor(obstacle, other);
						} else if((teta <= 0.5) && (completed == true)) {
							reg_pos = true;
							distance_from_obstacle = right_distance;
						}
					}
				}
			}
		} else if ((rotation == "right_rotation") && (sensor_cycle == 0.0)) {        //uno dei due sensori non vede più l'ostacolo (che aveva visto fino alla chiamata precedente)
			if(other == "right") {
				//left_obstacle = false;   //buttare: ho forzato lo spegnimento del sensore sinistro
				if(left_obstacle == false) {//anche l'altro sensore non vede più l'ostacolo ---> non c'è più l'ostacolo (commentata perché ostacolo vicino)
					is_obstacle = false;
					rotation_check = true;
					route1 = distance_from_obstacle/(cos(abs(z_step)));   //distanza da percorrere in linea retta per evitare l'ostacolo (provengo da una situazione di ostacolo, è implicito nel fatto che sono dentro questa funzione)
					alfa = z_step;
					rotation = "left_rotation";
					rotation_right_left = false;
					sensor_cycle = 1.0;
					//if(rotation_right_left == true) {
						//checkOtherSensor(0, "right");
					//}
				}
			} else if(other == "left") {
				if(right_obstacle == false) {        //anche l'altro sensore non vede più l'ostacolo ---> non c'è più l'ostacolo
					is_obstacle = false;
					rotation_check = true;
					route1 = distance_from_obstacle/(cos(abs(z_step)));   //distanza da percorrere in linea retta per evitare l'ostacolo (provengo da una situazione di ostacolo, è implicito nel fatto che sono dentro questa funzione)
					alfa = z_step;
					rotation = "left_rotation";
					rotation_right_left = false;
					sensor_cycle = 1.0;
					//if(rotation_right_left == true) {
						//checkOtherSensor(0, "left");
					//}
				}
			}
		} else if (rotation == "left_rotation") {                           //uno dei due sensori non vede più l'ostacolo (che aveva visto fino alla chiamata precedente)
			if(other == "right") {
				if(left_obstacle == false) {//anche l'altro sensore non vede più l'ostacolo ---> non c'è più l'ostacolo (commentata perché ostacolo vicino)
					is_obstacle = false;
					route2 = distance_from_obstacle/(abs(cos(z_step)));   //distanza da percorrere in linea retta per evitare l'ostacolo (provengo da una situazione di ostacolo, è implicito nel fatto che sono dentro questa funzione)
					rotation = "right_rotation";
					rotation_right_left = true;
					rotation_check = false;
					sensor_cycle = 2.0;
					if(route2 < route1) {
						route = route2;
						alfa = 0.0;
					} else {
						route = route1;
					}
				}
			} else if(other == "left") {
				//right_obstacle = false;    //buttare: forzo lo spegnimento del sensore destro
				if(right_obstacle == false) {        //anche l'altro sensore non vede più l'ostacolo ---> non c'è più l'ostacolo
					is_obstacle = false;
					route2 = distance_from_obstacle/(abs(cos(z_step)));   //distanza da percorrere in linea retta per evitare l'ostacolo (provengo da una situazione di ostacolo, è implicito nel fatto che sono dentro questa funzione)
					rotation = "right_rotation";
					rotation_right_left = true;
					rotation_check = false;
					sensor_cycle = 2.0;
					if(route2 < route1) {
						route = route2;
						alfa = 0.0;
					} else {
						route = route1;
					}
				}
			}
		}
	}
}

void MoveBase::headCallback(const std_msgs::Float32ConstPtr& head_pos)
{
	distance_from_head = head_pos->data;
}

void MoveBase::twistCallback(const geometry_msgs::TwistConstPtr& twist_vel)
{
	speed.x = (float)twist_vel->linear.x;
	speed.z = (float)twist_vel->angular.z;
}


void MoveBase::floorSensorCallback(const sensor_msgs::PointCloudConstPtr& floor_sensor)
{
	float floor_sensor_measure = floor_sensor->points[0].x;
	floor_sensor_ = floor_sensor->points[0].x;
	geometry_msgs::Twist base_vel;
	if((floor_sensor_measure < hill_threshold) || (floor_sensor_measure > well_threshold)) {
#ifdef DEBUG
		ROS_INFO("C'è un dosso o una cunetta");
#endif
		floor_obstacle = true;
		base_vel.linear.x = 0.0;
		base_vel.angular.z = -0.3;
		right_all_obstacle = false;
		left_all_obstacle = false;
		is_obstacle = false;
		was_obstacle = false;
		rotation_check = false;
		sensor_cycle = 0.0;
		right_obstacle = false;
		left_obstacle = false;
		rotation = "right_rotation";
		rotation_right_left = true;
		safePublish(base_vel);
		//base_control_pub_.publish(base_vel);
	} else {
		floor_obstacle = false;
	}
}



void MoveBase::rightSensorCallback(const sensor_msgs::PointCloudConstPtr& right_sensor)
{
	//ROS_ERROR("Chiamata la rightSensorCallback");
	right_sensor_ = right_sensor->points[0].x;
	mutex.lock();

	if(!floor_obstacle) {
		if(cycle_left == 1) {
			completed = true;
			cycle_left = 0;
			cycle_right = 0;
		} else {
			completed = false;
			cycle_right = 1;
		}
		right_distance = (float) right_sensor->points[0].x;

		if((right_distance != 0.0) && (right_distance < OBSTACLE_DISTANCE)) {
			//se c'è l'ostacolo eseguire il codice sotto
			right_obstacle = true;
			checkOtherSensor(1, "right"); //controlla se anche l'altro sensore vede l'ostacolo (gli passo 1 per dire che il destro vede l'ostacolo)
		} else {
			//se NON c'è ostacolo eseguire il codice qui sotto
			right_obstacle = false;
			if(is_obstacle == true) {    //se provengo da una situazione di ostacolo (ostacolo visto da entrambi i sensori)
				checkOtherSensor(0, "right"); //controlla se anche l'altro sensore ha smesso di vedere l'ostacolo (gli passo 0 per dire che il destro non vede più l'ostacolo)
			}
		}
	}

	mutex.unlock();
}

void MoveBase::leftSensorCallback(const sensor_msgs::PointCloudConstPtr& left_sensor)
{
	//ROS_ERROR("Chiamata la leftSensorCallback");
	left_sensor_=left_sensor->points[0].x;
	mutex.lock();

	if(!floor_obstacle) {
		if(cycle_right == 1) {
			completed = true;
			cycle_left = 0;
			cycle_right = 0;
		} else {
			completed = false;
			cycle_left = 1;
		}
		left_distance = (float) left_sensor->points[0].x;

		if((left_distance != 0.0) && (left_distance < OBSTACLE_DISTANCE)) {
			//se c'è l'ostacolo eseguire il codice sotto
			left_obstacle = true;
			checkOtherSensor(1, "left"); //controlla se anche l'altro sensore vede l'ostacolo (gli passo 1 per dire che il sinistro vede l'ostacolo)
			//fine
		} else {
			//se NON c'è ostacolo eseguire il codice qui sotto
			left_obstacle = false;
			if(is_obstacle == true) {    //se provengo da una situazione di ostacolo (ostacolo visto da entrambi i sensori)
				checkOtherSensor(0, "left"); //controlla se anche l'altro sensore ha smesso di vedere l'ostacolo (gli passo 0 per dire che il sinistro non vede più l'ostacolo)
			}
		}
	}

	mutex.unlock();
}

void MoveBase::wheelCallback(const geometry_msgs::Point32ConstPtr& wheel_pos)
{
	if(!floor_obstacle) {
		mutex.lock();
		float x_now = (float) wheel_pos->x;
		float y_now = (float) wheel_pos->y;
		float z_now = (float) wheel_pos->z;

		geometry_msgs::Twist base_vel;
		base_vel.linear.y = 0;
		base_vel.linear.z = 0;
		base_vel.angular.x = 0;
		base_vel.angular.y = 0;

		if(((is_obstacle || rotation_check) && (sensor_cycle < 2.0)) /*&& (rotation_recheck == 0) &&&& going_right*/) {         //se c'è un ostacolo
			if(!was_obstacle) {   //se è la prima volta che vedo un ostacolo fermo il robot
#ifdef DEBUG
				ROS_INFO("Vedo l'ostacolo per la prima volta");
#endif
				obst_to_goal = distance_from_head - distance_from_obstacle;
				if(reg_pos == true) {
					z_obst = z_now;
					was_obstacle = true;
					reg_pos = false;
					base_vel.linear.x = 0;
					base_vel.angular.z = 0;  //fermo il robot perché ho regolato la posizione ---> il robot non deve più ruotare per posizionarsi perpendicolarmente all'ostacolo
				}
			} else {                     //eseguo le operazioni per vedere il punto in cui finisce l'ostacolo
#ifdef DEBUG
					ROS_INFO("Eseguo le operazioni per vedere il punto in cui finisce l'ostacolo");
#endif
					z_step = z_now - z_obst;
					if(((z_step <= 1.57) && (z_step >= -1.57)) || half_step) {   //1,57 radianti (90°) è l'angolo oltre il quale non considero più l'ostacolo come presente
						base_vel.linear.x = 0;
						if(rotation_right_left == true) {
							base_vel.angular.z = -controlPID(0, 0, 1.3, kp_yaw_, ki_yaw_, kd_yaw_);
						} else {
							base_vel.angular.z = controlPID(0, 0, 1.7, kp_yaw_, ki_yaw_, kd_yaw_);
							half_step = false;
						}
					} else if(rotation_right_left == true) {
						rotation_right_left = false;
						half_step = true;
						right_all_obstacle = true;
						if(left_all_obstacle == true) {
							right_all_obstacle = false;
							left_all_obstacle = false;
							is_obstacle = false;
							was_obstacle = false;
							rotation_check = false;
							sensor_cycle = 0.0;
							right_obstacle = false;
							left_obstacle = false;
							rotation = "right_rotation";
						}
					} else if(rotation_right_left == false) {
						left_all_obstacle = true;
						if(right_all_obstacle == true) {
							if((z_now - z_obst) < 3.14) {
								base_vel.angular.z = 0.2;
								base_vel.linear.x = 0.0;
							} else {
#ifdef DEBUG
								ROS_INFO("Ci sono ostacoli dappertutto: mi volto e torno indietro");
#endif
								right_all_obstacle = false;
								left_all_obstacle = false;
								is_obstacle = false;
								was_obstacle = false;
								rotation_check = false;
								sensor_cycle = 0.0;
								right_obstacle = false;
								left_obstacle = false;
								rotation = "right_rotation";
								rotation_right_left = true;
							}
						}
					}

			}
		} else if(right_obstacle && (sensor_cycle == 0) && (completed == true)) {
#ifdef DEBUG
			ROS_INFO("Vedo un ostacolo solo a destra");
#endif
			base_vel.angular.z = 0.5;
			base_vel.linear.x = 0.2;
		} else if(left_obstacle && (sensor_cycle == 0) && (completed == true)){
#ifdef DEBUG
			ROS_INFO("Vedo un ostacolo solo a sinistra");
#endif
			base_vel.angular.z = -0.5;
			base_vel.linear.x = 0.2;
		} else {            //se NON c'è ostacolo
			if(!was_obstacle) {   //se non c'è ostacolo e non c'era neanche prima vuol dire che sono nella situazione normale di funzionamento ----> faccio
								  //solo da passacarte per la velocità da spedire calcolata secondo i criteri soliti da altri nodi
				if(flag){
					flag_distance = obst_to_goal/(sin(abs(yaw_reset)));
					if(((y_now - y_flag_prev)*(y_now - y_flag_prev) + (x_now - x_flag_prev)*(x_now - x_flag_prev)) < ((flag_distance-OBSTACLE_DISTANCE-0.1)*(flag_distance-OBSTACLE_DISTANCE-0.1))) {
						base_vel.linear.x= 0.5;
						base_vel.angular.z= 0;    //vado in linea retta
#ifdef DEBUG
						ROS_INFO("Vado verso il volto");
#endif
					}else {
#ifdef DEBUG
						ROS_INFO("Sono fermo vicino al volto");
#endif
						base_vel.linear.x= 0;
						flag=false; // sono ferma, sono vicino il volto
						stop = true;
						begin = ros::Time::now();
					}
				}else{
					if((stop == true) && ((ros::Time::now() - begin).operator<(range))) {
#ifdef DEBUG
						ROS_INFO("Sono fermo per cinque secondi");
#endif
						base_vel.linear.x = 0.0;
						base_vel.angular.z = 0.0;
					} else {
#ifdef DEBUG
						ROS_INFO("Riparto");
#endif
						base_vel.linear.x = speed.x;
						base_vel.angular.z = speed.z;
					}
				}
			} else {
				if((alfa != 0.0) && ((z_now - z_obst) > alfa)) {
#ifdef DEBUG
					ROS_INFO("Ritorno verso destra perché quello è il percorso minore");
#endif
					base_vel.angular.z = -1.5;
					base_vel.linear.x = 0.0;
					is_obstacle = false;
				} else if(alfa != 0.0) {
					//semaforo = true;
					alfa = 0.0;
				} else if (sensor_cycle != 0.0) {
					semaforo = true;
					sensor_cycle = 0.0;
				} else {
					//non c'è ostacolo ma NON sono nella situazione normale: sono nella situazione di evitare l'ostacolo, non lo vedo perché
					//lo sto evitando, ma l'ostacolo c'è
					semaforo = false;
					if(first_call == true) {
						x_prev = x_now;
						y_prev = y_now;
						z_prev = z_now;
						first_call = false;
					}
					if(((y_now - y_prev)*(y_now - y_prev) + (x_now - x_prev)*(x_now - x_prev)) < (route*route)) {
#ifdef DEBUG
						ROS_INFO("Percorro il tratto per evitare l'ostacolo");
#endif
						base_vel.linear.x= 0.4;
						base_vel.angular.z= 0;    //vado in linea retta
						yaw_reset = -atan(obst_to_goal/(distance_from_obstacle*tan(z_now - z_obst))); //deve ruotare per riposizionarsi verso la testa
						z_step = z_now - z_obst;

					} else {       //ho evitato l'ostacolo ---> setto was_obstacle=false e is_obstacle=false in modo che potrò riprendere il normale funzionamento
#ifdef DEBUG
						ROS_INFO("Ho evitato l'ostacolo");
#endif
						base_vel.linear.x= 0.0;   //fermo il robot
						//yaw_reset = -atan(obst_to_goal/(distance_from_obstacle*tan(z_now - z_obst))); //deve ruotare per riposizionarsi verso la testa
						if((abs(z_now - z_obst) > abs(1.57 - abs(yaw_reset))) && (((z_now - z_obst) * (yaw_reset)) > 0.0)) {
							base_vel.angular.z = 0.0;
							was_obstacle = false;
							is_obstacle = false;
							flag=true;
							x_flag_prev = x_now;
							y_flag_prev = y_now;   // salvo la posizione prima di raggiungere il volto, dopo aver ruotato
							first_call = true;
						} else {
							if((z_step) > 0) {
								base_vel.angular.z = -1.0;
							} else {
								base_vel.angular.z = 1.0;
							}
						}
					}
				}
			}
		}
		//publish
		safePublish(base_vel);
		//base_control_pub_.publish(base_vel);

		mutex.unlock();
		//ROS_ERROR("wheel callbacj mutex unlock");
	}
}

void MoveBase::putOrthogonal(float teta) {
	geometry_msgs::Twist base_vel;
	base_vel.linear.x = 0.0;
	base_vel.angular.z = controlPID(0, 0, teta, kp_yaw_, ki_yaw_, kd_yaw_);
	safePublish(base_vel);
	//base_control_pub_.publish(base_vel);
}


float MoveBase::controlPID(float x, float ix, float dx, float Kp, float Ki, float Kd)
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

void MoveBase::safePublish(geometry_msgs::Twist msg)
{
	if((left_sensor_>0.2 || left_sensor_==0) && (right_sensor_>0.2 || right_sensor_==0) && (floor_sensor_>0.16))
			{
				if (std::abs(msg.linear.x)>1.0)
				{
					msg.linear.x=sgn(msg.linear.x)*1.0;
				}
				if (std::abs(msg.angular.z)>1.0)
				{
					msg.angular.z=sgn(msg.angular.z)*1.0;
				}

				base_control_pub_.publish(msg);
			}
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "qbo_move_base");
  ROS_INFO("qbo_move_base node initialized");


  MoveBase mb;
  ros::spin();

  return 0;
}

