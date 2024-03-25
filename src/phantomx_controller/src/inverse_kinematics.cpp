#include "phantomx.h"
#include "inverse_kinematics.h"
#include <ros/console.h>

InverseKinematics::InverseKinematics(NodeHandle& node_handle):Phantomx(node_handle)
{
	coxa_joints   = new Vector3d [NUM_SIDES];
	foot_tips 	  = new Vector3d [NUM_SIDES];

	for(unsigned i=0; i<NUM_SIDES; i++)
	{
		ik_angles[i] = new double [3];
		for (unsigned int j=0; j<3; j++)
			ik_angles[i][j] = 0;

	}

	/* initialize coxa joints' positions */
  	coxa_joints[right_front]  = rf_c;
  	coxa_joints[right_middle] = rm_c;
  	coxa_joints[right_rear]   = rr_c;
  	coxa_joints[left_rear]    = lr_c;
  	coxa_joints[left_middle]  = lm_c;
  	coxa_joints[left_front]   = lf_c;

  	/* initialize foots */
  	foot_tips[right_front]  = rf_ref;
  	foot_tips[right_middle] = rm_ref;
  	foot_tips[right_rear]   = rr_ref;
  	foot_tips[left_rear]    = lr_ref;
  	foot_tips[left_middle]  = lm_ref;
  	foot_tips[left_front]   = lf_ref;
};

template <typename Derived>
void InverseKinematics::getAngles(const MatrixBase<Derived>& position, double* angles)
{
	try {
		// The following values were taken from experiments performed by lab interns from previous years:
		// const double offset_femur = -0.19*PI/180;//-0.19°
		// const double offset_tibia = -0.73*PI/180-PI/2;//-0.73°
		const double offset_femur = 0;
		const double offset_tibia = -PI/2;
		
		double x, y, z, u;
		double c1, s1, c2, s2;

		x = position(0);
		y = position(1);
		z = position(2);

		angles[0] =  atan2(y,x); //returns value in [-pi,pi]

		u = sqrt(x*x + y*y) - LC;

		if (sqrt(u*u + z*z) > LF + LT){ // This means the desired foot tip position is outside the leg workspace
			// Adjust desired foottip to have the maximum length allowed:
			const double k = (LF + LT)/sqrt(u*u + z*z);
			u = k * u * 0.9; // The 0.9 is to reduce numerical errors
			z = k * z * 0.9;
			ROS_INFO_STREAM("the desired foot tip position is outside the leg workspace and was adjuisted");
		}

		// IK equations come from the calculations seen on the course "Control de Robots" from the Universidad Nacional de San Juan, 
		// except for a sign error found on s1.
		c2 = (u*u + z*z - LF*LF - LT*LT) / (2*LF*LT);
		if (c2 > 1){ // This is to prevent numerical errors
			c2 = 1;
		}
		else if (c2 < -1)
		{
			c2 = -1;
		}
		
		s2 = -sqrt(1 - c2*c2);		

		c1 = (u * (LF + LT*c2) + z * LT*s2) / (u*u + z*z);
		s1 = (z * (LF + LT*c2) - u * LT*s2) / (u*u + z*z);

		angles[1] = - atan2(s1,c1) - offset_femur; //The orientation is the opposite of the one shown on the course, hence the -
		angles[2] = + atan2(s2,c2) - offset_tibia;

		// //TODO: borrar esta verificacion
		// u = LF * cos(-angles[1] - offset_femur) + LT * cos(-angles[1] - offset_femur + angles[2] + offset_tibia);
		// z = LF * sin(-angles[1] - offset_femur) + LT * sin(-angles[1] - offset_femur + angles[2] + offset_tibia);
		// y = (u + LC) * sin(angles[0]);
		// x = (u + LC) * cos(angles[0]);
		// ROS_INFO_STREAM("Original: " << position[0] << ',' << position[1] << ',' << position[2] << ", IK+DK = " << x << ',' << y << ',' << z <<  "\n");
	}
	catch (...){

		ROS_INFO_STREAM("The configuration: [" << position[0] << ',' << position[1] << ',' << position[2] << "] is outside of the robot's workspace!" << "\n");
	}
}


/**
 * Change vector's coordinate system from O_0 to O_1, given that O_1 is obtained rotating O_0 using the angles yaw, pitch and roll, 
 * in that order, and then translating it according to the translation vector.
 * Mathematically:
 * 					v_out = R^T * v_in - R^T *d = R^T (v_in - d)
 * where d is the translation vector and R is the rotation matrix:
 * 					R = R_z(yaw) * R_y(pitch) * R_x(roll)
 * 				R^T = [ cos(yaw)*cos(pitch)                                  sin(yaw)*cos(pitch)                                     -sin(pitch)
 *        				cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll)    sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll)        cos(pitch)*sin(roll)
 *         				cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll)     sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll)       cos(pitch)*cos(roll)]
 * This code follows the convention used in Fernndez-Madrigal, J. A., & Claraco, J. L. B. (2012), Simultaneous 
 * Localization and Mapping for Mobile Robots: Introduction and Methods, for the rotation (yaw-pitch-roll representation),
 * which implies first rotating yaw around z, then pitch around the new y, and finally roll around the new x.
*/
template <typename Derived1, typename Derived2>
Vector3d InverseKinematics::passiveTransformation(const double yaw,
										        const double pitch, 
										   	    const double roll,
										        const MatrixBase<Derived1>& translation_vector,
										        const MatrixBase<Derived2>& initial_vector)
{
	Matrix3d R;
	Vector3d result;
	R << cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll),
		 sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll),
		 -sin(pitch)        , cos(pitch)*sin(roll)                            , cos(pitch)*cos(roll);
 	
	result = R.transpose() * (initial_vector - translation_vector);
	return result;
}


/**
 * Geometrically rotate a vector using the angles yaw, pitch and roll, in that order, and then translate it according to the translation vector.
 * Mathematically:
 * 					v_out = R * v_in + d
 * where d is the translation vector and R is the rotation matrix:
 * 					R = R_z(yaw) * R_y(pitch) * R_x(roll)
 * 				R = [ cos(yaw)*cos(pitch)    cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll)     cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll)                                
 *        			  sin(yaw)*cos(pitch)    sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll)    sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll)
 *         			  -sin(pitch)            cos(pitch)*sin(roll)                                cos(pitch)*cos(roll)]
 * This code follows the convention used in Fernndez-Madrigal, J. A., & Claraco, J. L. B. (2012), Simultaneous 
 * Localization and Mapping for Mobile Robots: Introduction and Methods, for the rotation (yaw-pitch-roll representation),
 * which implies first rotating yaw around z, then pitch around the new y, and finally roll around the new x.
*/
template <typename Derived1, typename Derived2>
Vector3d InverseKinematics::activeTransformation(const double yaw,
										        const double pitch, 
										   	    const double roll,
										        const MatrixBase<Derived1>& translation_vector,
										        const MatrixBase<Derived2>& vector_to_rotate)
{

	Matrix3d R;
	Vector3d result;

	R << cos(yaw)*cos(pitch),  cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll),   cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll),
		 sin(yaw)*cos(pitch),  sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll),   sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll),
		 -sin(pitch)        ,  cos(pitch)*sin(roll)                            ,   cos(pitch)*cos(roll);
 	
	result = R * vector_to_rotate + translation_vector;
	return result;
}


/**
 * Adjust the base position and orientation without changing the foot tip position of any leg.
*/
// template <typename Derived4>
void InverseKinematics::orientBase(double yaw,  
								   double pitch,
								   double roll,
								   const Vector3d& T //const MatrixBase<Derived4>& Tt)//
								   )
{
	/* robot orientation */
	Float64 command_message;

	for (hc robot_side = right_front; robot_side <= left_front; robot_side = hc(robot_side+1))
	{
		//Calculation of the foot tip's coordinates in the new base system
		foot_tip_wrt_base = passiveTransformation(yaw, pitch, roll, T, foot_tips[robot_side]);
		
		//Calculation of the foot tip's coordinates in the new leg's coxa system
		foot_tip_wrt_cj = passiveTransformation(coxa_joints_orientations[robot_side], 0.0, 0.0, coxa_joints[robot_side], foot_tip_wrt_base);

		getAngles(foot_tip_wrt_cj, ik_angles[robot_side]);
		command_message.data = ik_angles[robot_side][0];
		publishers[get<0>(legs_coordenates[robot_side])].publish(command_message);
		command_message.data = ik_angles[robot_side][1];
		publishers[get<1>(legs_coordenates[robot_side])].publish(command_message);
		command_message.data = ik_angles[robot_side][2];
		publishers[get<2>(legs_coordenates[robot_side])].publish(command_message);
	}		
}

ostream& operator<<(ostream& os, const InverseKinematics& ik)
{

	os << "Corners' positions:" << endl;
	for (int i = 0; i < 6; ++i)
		os << "Corner " << i << ": " <<'[' << ik.coxa_joints[i][0] << ',' << ik.coxa_joints[i][1] << ',' << ik.coxa_joints[i][2] << ']' << endl;

	os << endl;

	os << "Foots tips' positions:" << endl;
	for (int i = 0; i < 6; ++i)
		os << "Foots tips " << i << ": " <<'[' << ik.foot_tips[i][0] << ',' << ik.foot_tips[i][1] << ',' << ik.foot_tips[i][2] << ']' << endl;

	os << endl;

	os << "IK's angles:"<< endl;
	for (int i = 0; i < 6; ++i)
	 	os << "IK angles " << i << ": " << '[' << ik.ik_angles[i][0] << ", " << ik.ik_angles[i][1] << ", " << ik.ik_angles[i][2] << ']' << endl;
	
    return os;
}

InverseKinematics::~InverseKinematics(){}
