#include "qbo_move_base.h"
#include <cmath>
#include <exception>


MoveBase::MoveBase()
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
	ROS_INFO("Inizializzazione del nodo qbo_move_base");
	/*
	 * Set node's subscriptions
	 */
	//joint_sub_=private_nh_.subscribe<sensor_msgs::JointState>("/cmd_joints",10,&MoveBase::jointCallback, this);
	twist_sub_=private_nh_.subscribe<geometry_msgs::Twist>("/pre_cmd_vel",10,&MoveBase::twistCallback, this);
	floor_sensor_sub_=private_nh_.subscribe<sensor_msgs::PointCloud>("/distance_sensors_state/floor_sensor",10,&MoveBase::floorSensorCallback, this);
	right_sensor_sub_=private_nh_.subscribe<sensor_msgs::PointCloud>("/distance_sensors_state/front_right_srf10",10,&MoveBase::rightSensorCallback, this);

	left_sensor_sub_=private_nh_.subscribe<sensor_msgs::PointCloud>("/distance_sensors_state/front_left_srf10",10,&MoveBase::leftSensorCallback, this);
	head_sub = private_nh_.subscribe<std_msgs::Float32>("/head_distance",10,&MoveBase::headCallback, this);

	/*
	 * Set node's publishers
	 */
	base_control_pub_ = private_nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000); //To move robot's base

	setROSParams();

	is_obstacle = false;
	was_obstacle = false;
	right_obstacle = false;
	left_obstacle = false;
	counter = 0;
	kp_yaw_= 0.0;
	ki_yaw_= 0;
	kd_yaw_= 0.5;
	init_pos.x = 0.0;
	init_pos.y = 0.0;
	init_pos.z = 0.0;
	sensor_distance = 0.22;
	difference_distance = 0.0;
	teta = 0.0;
	subscribed = false;
	completed = false;
	cycle_right = 0;
	cycle_left = 0;
	reg_pos = false;
	floor_obstacle = false;
	//pos_initial = true;
}

//la funzione checkOtherSensor verrà chiamata sia nella callback del sensore destro sia nella callback del sensore sinistro (sia per vedere se c'è un
//ostacolo visto da entrambi i sensori sia per vedere se nessuno dei due sensori vede un ostacolo): viene chiamata in entrambe le callback perché non
//so quale delle due callback viene chiamata per prima, quindi se viene chiamata prima la callback del sensore di destra quando questa callback
//chiamerà la checkOtherSensor non succederà niente, ma succederà qualcosa quando questa funzione verrà chiamata dalla callback del sensore di
//sinistra, dato che la callback del sensore di destra già è stata chiamata (e viceversa se viene chiamata prima la callback del sensore di sinistra)
void MoveBase::checkOtherSensor(int obstacle, string other) {
	if(obstacle == 1) {                  //uno dei due sensori ha visto l'ostacolo
		if(other == "right") {
			if(left_obstacle == true) {    //anche l'altro sensore ha visto l'ostacolo
				is_obstacle = true;
				//wheel_sub = private_nh_.subscribe<nav_msgs::Odometry>("/odom",10,&MoveBase::wheelCallback, this);
				if(was_obstacle == false) {
					if(subscribed == false) {
						ROS_INFO("Mi sottoscrivo alla wheelCallback");
						//wheel_sub = private_nh_.subscribe<geometry_msgs::Point32>("/qbo_arduqbo/my_odom",10,&MoveBase::wheelCallback, this);
						subscribed = true;
					}
					difference_distance = right_distance - left_distance;
					teta = tan(difference_distance/sensor_distance);
					if((teta > 0.5) && (completed == true)) {
						putOrthogonal(teta);   //il robot si deve mettere perpendicolare all'ostacolo
						reg_pos = false;
						//checkOtherSensor(obstacle, other);
					} else if((teta <= 0.5) && (completed == true)) {
						ROS_INFO("Sono perpendicolare all'ostacolo");
						reg_pos = true;
						distance_from_obstacle = right_distance;
					}
				}
				//route = distance_from_obstacle/cos(yaw_start);
				//t_end = ros::Duration(route/1.0);             //dò 1 come velocità lineare costante
				//yaw_start = yaw_from_joint;
			}
		} else if(other == "left") {
			if(right_obstacle == true) {    //anche l'altro sensore ha visto l'ostacolo
				is_obstacle = true;
				//wheel_sub = private_nh_.subscribe<nav_msgs::Odometry>("/odom",10,&MoveBase::wheelCallback, this);
				if(was_obstacle == false) {
					if(subscribed == false) {
						wheel_sub = private_nh_.subscribe<geometry_msgs::Point32>("/qbo_arduqbo/my_odom",10,&MoveBase::wheelCallback, this);
						subscribed = true;
					}
					difference_distance = right_distance - left_distance;
					teta = tan(difference_distance/sensor_distance);
					if((teta > 0.2) && (completed == true)) {
						putOrthogonal(teta);   //il robot si deve mettere perpendicolare all'ostacolo
						reg_pos = false;
						//checkOtherSensor(obstacle, other);
					} else if((teta <= 0.2) && (completed == true)) {
						reg_pos = true;
						distance_from_obstacle = right_distance;
					}
				}
				//route = distance_from_obstacle/cos(yaw_start);
				//t_end = ros::Duration(route/1.0);              //dò 1 come velocità lineare costante
				//yaw_start = yaw_from_joint;
			}
		}
	} else {                           //uno dei due sensori non vede più l'ostacolo (che aveva visto fino alla chiamata precedente)
		if(other == "right") {
			if(left_obstacle == false) {         //anche l'altro sensore non vede più l'ostacolo ---> non c'è più l'ostacolo
				is_obstacle = false;
				route = distance_from_obstacle/(abs(cos(z_base)));   //distanza da percorrere in linea retta per evitare l'ostacolo (provengo da una situazione di ostacolo, è implicito nel fatto che sono dentro questa funzione)
				t_end = ros::Duration(route/0.2);              //dò 1 come velocità lineare costante
			}
		} else if(other == "left") {
			if(right_obstacle == false) {        //anche l'altro sensore non vede più l'ostacolo ---> non c'è più l'ostacolo
				is_obstacle = false;
				route = distance_from_obstacle/(abs(cos(z_base)));   //distanza da percorrere in linea retta per evitare l'ostacolo (provengo da una situazione di ostacolo, è implicito nel fatto che sono dentro questa funzione)
				t_end = ros::Duration(route/0.2);              //dò 1 come velocità lineare costante
			}
		}
	}
}

/*
void MoveBase::jointCallback(const sensor_msgs::JointStateConstPtr& joint_state)
{
	yaw_from_joint = 0;
}
*/

void MoveBase::headCallback(const std_msgs::Float32ConstPtr& head_pos)
{
	//ROS_INFO("Chiamata la headCallback");
	distance_from_head = head_pos->data;
}

void MoveBase::twistCallback(const geometry_msgs::TwistConstPtr& twist_vel)
{
	//ROS_INFO("Chiamata la twistCallback");
	geometry_msgs::Twist base_vel;
	speed.x = (float)twist_vel->linear.x;
	speed.z = (float)twist_vel->angular.z;
}


void MoveBase::floorSensorCallback(const sensor_msgs::PointCloudConstPtr& floor_sensor)
{
	//ROS_INFO("Chiamata la floorSensorCallback");
	floor_obstacle = false;
}



void MoveBase::rightSensorCallback(const sensor_msgs::PointCloudConstPtr& right_sensor)
{
	//ROS_ERROR("Chiamata la rightSensorCallback");
	mutex.lock();
	//ROS_INFO("Risght sensor callback mutex locked");

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

		if(right_distance != 0.0) {
			//se c'è l'ostacolo eseguire il codice sotto
			right_obstacle = true;
			checkOtherSensor(1, "right"); //controlla se anche l'altro sensore vede l'ostacolo (gli passo 1 per dire che il destro vede l'ostacolo)
			//fine
		} else {
			//se NON c'è ostacolo eseguire il codice qui sotto
			right_obstacle = false;
			if(is_obstacle == true) {    //se provengo da una situazione di ostacolo (ostacolo visto da entrambi i sensori)
				checkOtherSensor(0, "right"); //controlla se anche l'altro sensore ha smesso di vedere l'ostacolo (gli passo 0 per dire che il destro non vede più l'ostacolo)
			}
			//fine
		}
	}

	mutex.unlock();
	//ROS_INFO("Right sensor callback mutex unlock");
}

void MoveBase::leftSensorCallback(const sensor_msgs::PointCloudConstPtr& left_sensor)
{
	//ROS_ERROR("Chiamata la leftSensorCallback");
	mutex.lock();
	//ROS_INFO("Left sensor callback mutex lock");

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

		if(left_distance != 0.0) {
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
			//fine
		}
	}

	mutex.unlock();
	//ROS_INFO("Left sensor callback mutex unlock");
}

void MoveBase::wheelCallback(const geometry_msgs::Point32ConstPtr& wheel_pos)
{
	ROS_ERROR("Chiamata la wheelCallback");
	mutex.lock();
	ROS_ERROR("wheel callbacj mutex lock");
	float x_now = (float) wheel_pos->x;
	float y_now = (float) wheel_pos->y;
	float z_now = (float) wheel_pos->z;

	geometry_msgs::Twist base_vel;
	base_vel.linear.y = 0;
	base_vel.linear.z = 0;
	base_vel.angular.x = 0;
	base_vel.angular.y = 0;

	if(is_obstacle) {         //se c'è un ostacolo
	ROS_INFO("Dentro la wheelCallback: c'è un ostacolo e basta");
		if(!was_obstacle) {   //se è la prima volta che vedo un ostacolo fermo il robot
			ROS_INFO("Dentro la wheelCallback: c'è un ostacolo e prima non c'era");
			t_begin = ros::Time::now();
			obst_to_goal = distance_from_head - distance_from_obstacle;
			z_obst = z_now;
			//base_vel.linear.x = 0;
			//base_vel.angular.z = 0;
			//yaw_stop = yaw_from_joint;
			if(reg_pos == true) {
				ROS_INFO("Mi fermo prima di calcolare");
				was_obstacle = true;
				base_vel.linear.x = 0;
				base_vel.angular.z = 0;  //fermo il robot perché ho regolato la posizione ---> il robot non deve più ruotare per posizionarsi perpendicolarmente all'ostacolo
			}
		} else {
			ROS_INFO("Dentro la wheelCallback: c'è un ostacolo e c'era anche prima");
			if((counter % WAITING) != 0) {   //non eseguo operazioni ogni volta che viene chiamata la callback, ma una volta ogni WAITING chiamate
				base_vel.linear.x = 0;
				base_vel.angular.z = 0;
				counter++;
			} else {                        //eseguo le operazioni per vedere il punto in cui finisce l'ostacolo
				//yaw_start = yaw_from_joint - yaw_stop;
				z_step = z_now - z_obst;
				//yaw_sum += yaw_start;
				//base_vel.angular.z = -controlPID(0, 0, yaw_start, kp_yaw_, ki_yaw_, kd_yaw_);
				//yaw_stop = yaw_from_joint;
				if(z_step < 0.7) {   //0,7 radianti è l'angolo oltre il quale non considero più l'ostacolo come presente
					base_vel.linear.x = 0;
					base_vel.angular.z = -controlPID(0, 0, z_step, kp_yaw_, ki_yaw_, kd_yaw_);
				} else {
					is_obstacle = false;
					was_obstacle = false;
					counter = 0;
				}

			}
		}
	} else if(right_obstacle) {
		ROS_INFO("Dentro la wheelCallback: c'è un ostacolo solo a destra");
	} else if(left_obstacle){
		ROS_INFO("Dentro la wheelCallback: c'è un ostacolo solo a sinistra");
	} else {            //se NON c'è ostacolo
		if(!was_obstacle) {   //se non c'è ostacolo e non c'era neanche prima vuol dire che sono nella situazione normale di funzionamento ----> faccio
							  //solo da passacarte per la velocità da spedire calcolata secondo i criteri soliti da altri nodi
			ROS_INFO("Dentro la wheelCallback: non c'è un ostacolo e non c'era anche prima");
			//base_vel.linear.x=(float)twist_vel->linear.x;
			//base_vel.angular.z=(float)twist_vel->angular.z;
			base_vel.linear.x = speed.x;
			base_vel.angular.z = speed.z;
			ROS_INFO("Faccio solo da passacarte");
			//pos_initial = true;
		} else {      //non c'è ostacolo ma NON sono nella situazione normale: sono nella situazione di evitare l'ostacolo, non lo vedo perché
					  //lo sto evitando, ma l'ostacolo c'è
			ROS_INFO("Dentro la wheelCallback: sto cercando di evitare l'ostacolo");
			if((ros::Time::now() - t_begin).operator <(t_end)) {  //sto percorrendo il tratto necessario a evitare l'ostacolo
				base_vel.linear.x= 0.2;
				base_vel.angular.z= 0;    //vado in linea retta
			} else {       //ho evitato l'ostacolo ---> setto was_obstacle=false e is_obstacle=false in modo che potrò riprendere il normale funzionamento
				ROS_INFO("Dentro la wheelCallback: ho evitato l'ostacolo");
				base_vel.linear.x= 0.0;   //fermo il robot
				//yaw_reset = tan(obst_to_goal/(distance_from_obstacle*sin(yaw_sum))); //deve ruotare per riposizionarsi verso la testa
				yaw_reset = tan(obst_to_goal/(distance_from_obstacle*sin(z_step))); //deve ruotare per riposizionarsi verso la testa
				base_vel.angular.z = controlPID(0, 0, yaw_reset, kp_yaw_, ki_yaw_, kd_yaw_);
				was_obstacle = false;
				is_obstacle = false;
				counter = 0;
				wheel_sub.shutdown();
				subscribed = false;
			}
		}
	}
	//publish
	base_control_pub_.publish(base_vel);

	mutex.unlock();
	//ROS_ERROR("wheel callbacj mutex unlock");
}

void MoveBase::putOrthogonal(float teta) {
	ROS_INFO("Cerco di posizionarmi ortogonalmente all'ostacolo");
	ROS_INFO("%f\n", teta);
	geometry_msgs::Twist base_vel;
	base_vel.linear.x = 0.0;
	base_vel.angular.z = controlPID(0, 0, teta, kp_yaw_, ki_yaw_, kd_yaw_);
	ROS_INFO("%f\n", base_vel.angular.z);
	base_control_pub_.publish(base_vel);
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "qbo_move_base");
  ROS_INFO("qbo_move_base node initialized");


  MoveBase mb;
  ros::spin();

  return 0;
}

