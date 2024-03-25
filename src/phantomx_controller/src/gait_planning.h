#ifndef GAIT_PLANNING_H
#define GAIT_PLANNING_H

#include <thread>
#include <tf/tf.h>
#include <ros/ros.h>
#include "inverse_kinematics.h"

using namespace std;
using namespace ros;
using namespace std_msgs;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::MatrixBase;

class GaitPlanning: public InverseKinematics {

	private:

		
		double frecs[6];			  // frequency for each leg
		double phi_init[6];			  // initial phase for each leg
		double frec_0;				  // initial frequency
		double phi[6];  			  // phase for each leg
		Vector2d linear_vel_command;  // linear velocity command
		double angular_vel_command;   // angular velocity command

		Vector3d dr[6];				  // target foot position residual for each leg
		const double h = 5; 	      // maximum foot height (cm)

		vector<Subscriber> subscribers_phases;
		vector<Subscriber> subscribers_delta_r;
		vector<Subscriber> subscribers_initial_phase;
		vector<Subscriber> subscriber_f0;
		vector<Subscriber> subscriber_linear_vel_command;
		vector<Subscriber> subscriber_angular_vel_command;
		vector<Publisher> publishers_phases;
		vector<Publisher> publishers_r;
		Publisher initial_frec;
		string command_publisher_phases[6] = {"/phantomx/phase/right_front",    
											  "/phantomx/phase/right_middle",	 
											  "/phantomx/phase/right_rear",	
											  "/phantomx/phase/left_rear",	     
											  "/phantomx/phase/left_middle",	 
											  "/phantomx/phase/left_front"};

		string command_publisher_r[6] = {"/phantomx/r/right_front",    
										 "/phantomx/r/right_middle",	 
										 "/phantomx/r/right_rear",	
										 "/phantomx/r/left_rear",	     
										 "/phantomx/r/left_middle",	 
										 "/phantomx/r/left_front"};


		string subscriber_phases_topics[6] = {"/phantomx/frec/right_front",    
										 	  "/phantomx/frec/right_middle",	 
										 	  "/phantomx/frec/right_rear",	
										 	  "/phantomx/frec/left_rear",	     
										      "/phantomx/frec/left_middle",	 
										      "/phantomx/frec/left_front"};
											  
		string subscriber_delta_r_topics[6] = {"/phantomx/delta_r/right_front",  	
										  	   "/phantomx/delta_r/right_middle",		
										  	   "/phantomx/delta_r/right_rear",		
										  	   "/phantomx/delta_r/left_rear",		
										  	   "/phantomx/delta_r/left_middle",		
										  	   "/phantomx/delta_r/left_front"};	

		string subscriber_initial_phase_topics[6] = {"/phantomx/initial_phase/right_front",  	
										  	   		 "/phantomx/initial_phase/right_middle",		
										  	   		 "/phantomx/initial_phase/right_rear",		
										  	   		 "/phantomx/initial_phase/left_rear",		
										  	   		 "/phantomx/initial_phase/left_middle",		
										  	   		 "/phantomx/initial_phase/left_front"};		

		

		Vector3d footTrajectoryGenerator(double, hexapod_coxa_joints);

		map< hexapod_coxa_joints, tuple<int, int, int> > legs_coordenates {{right_front,  {0, 6, 12}},
																		   {right_middle, {1, 7, 13}},
																		   {right_rear,   {2, 8, 14}},
																		   {left_rear,    {3, 9, 15}},
																		   {left_middle,  {4, 10, 16}},
																		   {left_front,   {5, 11, 17}}};

		void walkingPattern(hexapod_coxa_joints, double);

	public:
		
		GaitPlanning(NodeHandle&);
		
		~GaitPlanning();

		void startWalking();
	
};

#endif