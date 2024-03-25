#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <map>
#include <math.h>
#include <iostream>
#include "phantomx.h"
#include <ros/ros.h>
#include <Eigen/Dense>
#include <std_msgs/Float64.h>

using namespace std;
using namespace ros;
using namespace std_msgs;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::MatrixBase;

/*
 Default coordenates of the hexapod:
#########################################################
#	  Right: 				  Left:						#
#				  |----| y1 = 10.34cm					#		
#Rear: (2)     \_____/        (3)          				#
#	   		   |     |									#	
#	   		  /   z   \ 								#	
#      (1)  -=----0----=---> y (4)    	+				#
# 	   		  \   |   /					| x1 = 12.48cm	#
# 	   		   |__|__|					+				#
#Front:(0)     /  |  \        (5)   	    			#		
#         	      |										#
#                 V x 									#
#				  |--| y2 = 6.164cm						#
#########################################################
*/


class InverseKinematics: public Phantomx {

	/* 
	This code will be following the convention used in Fernndez-Madrigal, J. A., & Claraco, J. L. B. (2012), Simultaneous 
	Localization and Mapping for Mobile Robots: Introduction and Methods, for the rotation (yaw-pitch-roll representation).
	*/

	protected:

		double* ik_angles[6];

		template <typename Derived1, typename Derived2>
		Vector3d passiveTransformation(const double, 
								   	 const double,
								     const double,
								     const MatrixBase<Derived1>&,
								     const MatrixBase<Derived2>&);

		template <typename Derived1, typename Derived2>
		Vector3d activeTransformation(const double, 
								   	 const double,
								     const double,
								     const MatrixBase<Derived1>&,
								     const MatrixBase<Derived2>&);
	public:

		InverseKinematics(NodeHandle&);

		template <typename Derived>
		void getAngles(const MatrixBase<Derived>&, double*);

		// TODO: solucionar esto del template que me rompe todo
		// template <typename Derived4>
		void orientBase(double, 
						double, 
						double,
						const Vector3d&//const MatrixBase<Derivedd>&);//
						);

		friend ostream& operator<<(ostream& os,const InverseKinematics& ik);

		~InverseKinematics();	
};

#endif