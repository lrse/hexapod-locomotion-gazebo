#include <ros/ros.h>
#include <ros/console.h>
#include "gait_planning.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <random>

using namespace std;
using namespace ros;
using namespace std_msgs;

GaitPlanning::GaitPlanning(NodeHandle& node_handle):InverseKinematics(node_handle)
{
	frec_0 = 8.5;//1.25;//
	
	// Set up a random number generator with a uniform distribution
    std::random_device rd;  // Random device to seed the generator
    std::mt19937 generator(rd()); // Mersenne Twister engine with a 32-bit state size
    std::uniform_real_distribution<double> distr(0, 2*PI); // Uniform distribution between 0 and 2pi

	phi_init[right_front]  = 0.0;//distr(generator);
	phi_init[right_middle] = PI;//distr(generator);
	phi_init[right_rear]   = 0.0;//distr(generator);
	phi_init[left_rear]    = PI;//distr(generator);
	phi_init[left_middle]  = 0.0;//distr(generator);
	phi_init[left_front]   = PI;//distr(generator);

	// To change the color of text printed on the terminal:
	// GREEN = "\033[92m"  # Green text
	// RESET = "\033[0m"     # Reset text attributes to default (e.g., color)
	ROS_INFO_STREAM("\033[92m" << "Initial phases: " << phi_init[right_front] << ", " << phi_init[right_middle]<< ", " << phi_init[right_rear]<< ", " << phi_init[left_rear]<< ", " << phi_init[left_middle]<< ", " << phi_init[left_front] << '\n' << "\033[0m");
	
	initial_frec = node_handle.advertise<Float64>("/phantomx/base_frec",1);
	
	for(unsigned int i = 0; i < NUM_SIDES; i++)
	{
		phi[i]   = 0;
		frecs[i] = 0;
		dr[i]    << 0, 0, 0;

		Publisher new_publisher_phase = node_handle.advertise<Float64>(command_publisher_phases[i],1);
		Publisher new_publisher_r = node_handle.advertise<Float64MultiArray>(command_publisher_r[i],1);
		
		publishers_phases.push_back(new_publisher_phase);
		publishers_r.push_back(new_publisher_r);
	}

	// TODO no funciona loopeando, hay que hacerlo a mano:
	subscribers_phases.push_back(node_handle.subscribe<Float64>(subscriber_phases_topics[right_front], 
													queue_size, 
													[&](const Float64::ConstPtr& msg) 
													{frecs[right_front] = msg->data;}));

	subscribers_phases.push_back(node_handle.subscribe<Float64>(subscriber_phases_topics[right_middle], 
													queue_size, 
													[&](const Float64::ConstPtr& msg) 
													{frecs[right_middle] = msg->data;}));

	subscribers_phases.push_back(node_handle.subscribe<Float64>(subscriber_phases_topics[right_rear], 
													queue_size, 
													[&](const Float64::ConstPtr& msg) 
													{frecs[right_rear] = msg->data;}));

	subscribers_phases.push_back(node_handle.subscribe<Float64>(subscriber_phases_topics[left_front], 
													queue_size, 
													[&](const Float64::ConstPtr& msg) 
													{frecs[left_front] = msg->data;}));

	subscribers_phases.push_back(node_handle.subscribe<Float64>(subscriber_phases_topics[left_middle], 
													queue_size, 
													[&](const Float64::ConstPtr& msg) 
													{frecs[left_middle] = msg->data;}));

	subscribers_phases.push_back(node_handle.subscribe<Float64>(subscriber_phases_topics[left_rear], 
													queue_size, 
													[&](const Float64::ConstPtr& msg) 
													{frecs[left_rear] = msg->data;})); 													

	subscribers_delta_r.push_back(node_handle.subscribe<Float64MultiArray>(subscriber_delta_r_topics[left_front], 
													queue_size,
													[&](const Float64MultiArray::ConstPtr& msg) 
													{dr[left_front] = {msg->data[0], msg->data[1], msg->data[2]};}));

	subscribers_delta_r.push_back(node_handle.subscribe<Float64MultiArray>(subscriber_delta_r_topics[left_middle], 
													queue_size,
													[&](const Float64MultiArray::ConstPtr& msg) 
													{dr[left_middle] = {msg->data[0], msg->data[1], msg->data[2]};}));

	subscribers_delta_r.push_back(node_handle.subscribe<Float64MultiArray>(subscriber_delta_r_topics[left_rear], 
													queue_size,
													[&](const Float64MultiArray::ConstPtr& msg) 
													{dr[left_rear] = {msg->data[0], msg->data[1], msg->data[2]};}));

	subscribers_delta_r.push_back(node_handle.subscribe<Float64MultiArray>(subscriber_delta_r_topics[right_rear], 
													queue_size,
													[&](const Float64MultiArray::ConstPtr& msg) 
													{dr[right_rear] = {msg->data[0], msg->data[1], msg->data[2]};}));

	subscribers_delta_r.push_back(node_handle.subscribe<Float64MultiArray>(subscriber_delta_r_topics[right_middle], 
													queue_size,
													[&](const Float64MultiArray::ConstPtr& msg) 
													{dr[right_middle] = {msg->data[0], msg->data[1], msg->data[2]};}));

	subscribers_delta_r.push_back(node_handle.subscribe<Float64MultiArray>(subscriber_delta_r_topics[right_front], 
													queue_size,
													[&](const Float64MultiArray::ConstPtr& msg) 
													{dr[right_front] = {msg->data[0], msg->data[1], msg->data[2]};}));

	// subscribers_initial_phase.push_back(node_handle.subscribe<Float64>(subscriber_initial_phase_topics[left_front], 
	// 												queue_size,
	// 												[&](const Float64::ConstPtr& msg) 
	// 												{phi_init[left_front] = msg->data; ROS_INFO_STREAM("\033[92m" << "Initial phase for lf: " << phi_init[left_front] << '\n' << "\033[0m");}));

	// subscribers_initial_phase.push_back(node_handle.subscribe<Float64>(subscriber_initial_phase_topics[left_middle], 
	// 												queue_size,
	// 												[&](const Float64::ConstPtr& msg) 
	// 												{phi_init[left_middle] = msg->data; ROS_INFO_STREAM("\033[92m" << "Initial phase for lm: " << phi_init[left_middle] << '\n' << "\033[0m");}));

	// subscribers_initial_phase.push_back(node_handle.subscribe<Float64>(subscriber_initial_phase_topics[left_rear], 
	// 												queue_size,
	// 												[&](const Float64::ConstPtr& msg) 
	// 												{phi_init[left_rear] = msg->data; ROS_INFO_STREAM("\033[92m" << "Initial phase for lr: " << phi_init[left_rear] << '\n' << "\033[0m");}));

	// subscribers_initial_phase.push_back(node_handle.subscribe<Float64>(subscriber_initial_phase_topics[right_rear], 
	// 												queue_size,
	// 												[&](const Float64::ConstPtr& msg) 
	// 												{phi_init[right_rear] = msg->data; ROS_INFO_STREAM("\033[92m" << "Initial phase for rr: " << phi_init[right_rear] << '\n' << "\033[0m");}));

	// subscribers_initial_phase.push_back(node_handle.subscribe<Float64>(subscriber_initial_phase_topics[right_middle], 
	// 												queue_size,
	// 												[&](const Float64::ConstPtr& msg) 
	// 												{phi_init[right_middle] = msg->data; ROS_INFO_STREAM("\033[92m" << "Initial phase for rm: " << phi_init[right_middle] << '\n' << "\033[0m");}));

	// subscribers_initial_phase.push_back(node_handle.subscribe<Float64>(subscriber_initial_phase_topics[right_front], 
	// 												queue_size,
	// 												[&](const Float64::ConstPtr& msg) 
	// 												{phi_init[right_front] = msg->data; ROS_INFO_STREAM("\033[92m" << "Initial phase for rf: " << phi_init[right_front] << '\n' << "\033[0m");}));

	subscriber_f0.push_back(node_handle.subscribe<Float64>("phantomx/f0", 
													queue_size,
													[&](const Float64::ConstPtr& msg) 
													{frec_0 = msg->data;}));// ROS_INFO_STREAM("\033[92m" << "f0: " << frec_0 << '\n' << "\033[0m");}));
	subscriber_linear_vel_command.push_back(node_handle.subscribe<Float64MultiArray>("phantomx/linear_vel_command", 
													queue_size,
													[&](const Float64MultiArray::ConstPtr& msg) 
													{linear_vel_command = {msg->data[0], msg->data[1]}; ROS_INFO_STREAM("\033[92m" << "Linear vel command: " << linear_vel_command[0] << ", " << linear_vel_command[1] << '\n' << "\033[0m");}));
	subscriber_angular_vel_command.push_back(node_handle.subscribe<Float64>("phantomx/angular_vel_command", 
													queue_size,
													[&](const Float64::ConstPtr& msg) 
													{angular_vel_command = msg->data; ROS_INFO_STREAM("\033[92m" << "Angular vel command: " << angular_vel_command << '\n' << "\033[0m");}));
}

Vector3d GaitPlanning::footTrajectoryGenerator(double phi, hexapod_coxa_joints hc)
{
	Vector3d result;
	double x, y, z;
	const double z0 = 5;
	double k = 2 * (phi - PI)/PI;

	if      (0 <= k && k <= 1) {z = h * (-2*k*k*k + 3*k*k) - z0;}
	else if (1 < k && k <= 2)  {z = h * (2*k*k*k - 9*k*k + 12*k - 4) - z0;}
	else                       {z = - z0;}

	// To the vector we just calculated, we add the reference position for the leg (otherwise
	// the foot-tip would move just below the coxa, and we want the leg extended for the hexapod)
	// foottip_ref[hc] is in the robot's coordinate system, so we need to change it to Hi:
	extern double roll, pitch;
	double yaw=0;
	// if (0 <= k && k <= 2) {
	// 	yaw = -PI/18 * angular_vel_command; // <0 for positive turn
	// }
	// else{
	// 	yaw = PI/18 * angular_vel_command;
	// }
	result = InverseKinematics::passiveTransformation(yaw,
												-pitch, 
												-roll, 
												InverseKinematics::coxa_ref[hc],
												InverseKinematics::foottip_ref[hc]);
	
	// double u;
	// // u = 1-cos(k*PI/2);
	// // if      (0 <= k && k <= 2) {u = 2*k;}
	// // else                       {u = -2*k;}
	// x = u*1/sqrt(2);//linear_vel_command(0);
	// y = u*1/sqrt(2);//linear_vel_command(1);
	// result(0) += x;
	// result(1) += y;
	// // if (0 <= k && k <= 2) {
	// // 	result(0) += x;
	// // 	result(1) += y;
	// // }
	// // else{
	// // 	result(0) += -x;
	// // 	result(1) += -y;
	// // }
	result(2) += z;
	return result;
}

void GaitPlanning::walkingPattern(hexapod_coxa_joints hc, double t)
{
	extern double roll, pitch;
	Vector3d position_wrt_cj;
	phi[hc] =  fmod(phi_init[hc] + (frec_0 + frecs[hc]) * t, 2*PI);
	Float64 command_message_1;
    command_message_1.data = phi[hc];
    publishers_phases[hc].publish(command_message_1);
	Vector3d r = footTrajectoryGenerator(phi[hc], hc) + dr[hc];
	Vector3d origin;
	// With r[hc] on the coordinate system Hi, we calculate the position with respect to the coxa joint coordinate
	// system of the corresponding leg.
	position_wrt_cj = InverseKinematics::passiveTransformation(InverseKinematics::coxa_joints_orientations[hc],
												pitch, 
												roll, 
												origin,
												r);
	// Obs: I need only the values of pitch and roll of the base, since the yaw value afects both the base and the
	// horizontal coordinates, and hence the leg coordinate system is rotated wrt the horizontal system by
	// a fix angle in place of yaw, then pitch and roll of the base.
	double angles[3];
	InverseKinematics::getAngles(position_wrt_cj, angles);

	Float64 command_message;
	command_message.data = angles[0];
	Phantomx::publishers[get<0>(legs_coordenates[hc])].publish(command_message);
	command_message.data = angles[1];
	Phantomx::publishers[get<1>(legs_coordenates[hc])].publish(command_message);
	command_message.data = angles[2];
	Phantomx::publishers[get<2>(legs_coordenates[hc])].publish(command_message);

	// Publisher of r:
	Float64MultiArray r_message;
	r_message.data.push_back(r[0]);
	r_message.data.push_back(r[1]);
	r_message.data.push_back(r[2]);
	// r_message.data.push_back(position_wrt_cj[0]);
	// r_message.data.push_back(position_wrt_cj[1]);
	// r_message.data.push_back(position_wrt_cj[2]);
	r_message.data.push_back(t); //TODO: ver si esto no rompe todo
	publishers_r[hc].publish(r_message);
}

void GaitPlanning::startWalking(){

	double t; 			//Time
	double n = 0; 		//Time index

	double RATE = 400; 	// TODO: quitar este harcodeo

	while(ros::ok()) 	//do as long as the ROS node is still available
	{
		t = n / RATE; 	//Time in seconds
		n++;

		thread t_rr(&GaitPlanning::walkingPattern, this, right_rear,   t);
		thread t_rm(&GaitPlanning::walkingPattern, this, right_middle, t);
		thread t_rf(&GaitPlanning::walkingPattern, this, right_front,  t);
		thread t_lr(&GaitPlanning::walkingPattern, this, left_rear,    t);
		thread t_lm(&GaitPlanning::walkingPattern, this, left_middle,  t);
		thread t_lf(&GaitPlanning::walkingPattern, this, left_front,   t);

		t_rr.join();
		t_rm.join();
		t_rf.join();
		t_lr.join(); 
		t_lm.join();
		t_lf.join();


		Float64 initial_frec_message;
		initial_frec_message.data = frec_0;
		initial_frec.publish(initial_frec_message);
		ros::spinOnce(); 		// wait to the next callbacks 
		loop_rate.sleep(); 	// Sleep until next cycle (loop_rate = 50 Hz)
	}

}

GaitPlanning::~GaitPlanning(){}