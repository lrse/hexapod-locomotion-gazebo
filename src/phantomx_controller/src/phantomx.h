#ifndef PHANTOMX_H
#define PHANTOMX_H

#include <map>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <std_msgs/Float64.h>

using namespace std;
using namespace ros;
using namespace std_msgs;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::MatrixBase;

#define PI M_PI
 
class Phantomx {

public:
    
    //LC = length of coxa, LF = length of tibia, LT = length of thigh
    
    double LC               = 5.2;//5.053;  // coxa lenght (cm)
    double LF               = 6.6;//6.828;  // femur lenght (cm)
    double LT               = 13.8;//12.998; // tibia lenght (cm)
    Rate loop_rate          = 400; 
    unsigned int queue_size = 10;
    unsigned int NUM_SIDES  = 6;

    NodeHandle node_handle;

    Vector3d foot_tip_wrt_base, foot_tip_wrt_cj; 

    double x1 = 12.48;
    double y1 = 10.34;
    double y2 = 6.164;
    double z1 = 0.1116;
    double initial_z = -7;
    double displacement = 10.0 + 5.053;

    // coxa joint coordenates wrt the base
    const Vector3d rf_c = {x1 , -y2,  z1};  /* coxa joint of leg 0 */ 
    const Vector3d rm_c = {0  , -y1,  z1};  /* coxa joint of leg 1 */
    const Vector3d rr_c = {-x1, -y2,  z1};  /* coxa joint of leg 2 */
    const Vector3d lr_c = {-x1,  y2 , z1};  /* coxa joint of leg 3 */
    const Vector3d lm_c = {0  ,  y1 , z1};  /* coxa joint of leg 4 */
    const Vector3d lf_c = {x1 ,  y2 , z1};  /* coxa joint of leg 5 */ 

    // reference foot tip coordenates wrt the base 
    // Legs spread out:
    double cos_ang = cos(3*PI/8);
    double sin_ang = sin(3*PI/8);
    const Vector3d rf_ref = {x1 + (displacement*cos_ang) , -y2 - (displacement*sin_ang), initial_z};
    const Vector3d rm_ref = {0                             , -y1 - (displacement)*1        , initial_z};
    const Vector3d rr_ref = {-x1 - (displacement*cos_ang), -y2 - (displacement*sin_ang), initial_z};
    const Vector3d lr_ref = {-x1 - (displacement*cos_ang), y2 + (displacement*sin_ang) , initial_z};
    const Vector3d lm_ref = {0                             , y1 + (displacement)*1         , initial_z};
    const Vector3d lf_ref = {x1 + (displacement*cos_ang) , y2 + (displacement*sin_ang) , initial_z};


    typedef enum LP {rf_coxa,  rm_coxa,  rr_coxa,
		              lr_coxa,  lm_coxa,  lf_coxa,
				      rf_thigh, rm_thigh, rr_thigh,
				      lr_thigh, lm_thigh, lf_thigh,
				      rf_tibia, rm_tibia, rr_tibia,
				      lr_tibia, lm_tibia, lf_tibia } LegPosition;

    typedef enum hc {right_front = 0, 
                     right_middle, 
                     right_rear, 
                     left_rear, 
                     left_middle, 
                     left_front} hexapod_coxa_joints;

    /*Angle of the legs with respect to the base coordinate system. The values may not be correct, 
    need to be looked up in documentation (TODO)*/
    map<hexapod_coxa_joints, double> coxa_joints_orientations{{right_front,     -PI/4},
                                                                {right_middle,  -PI/2},
                                                                {right_rear,    -3*PI/4},
                                                                {left_front,     PI/4},
                                                                {left_middle,    PI/2},
                                                                {left_rear,      3*PI/4}};
    
    map < hexapod_coxa_joints, const Vector3d > foottip_ref{{right_front,  rf_ref},
                                                            {right_middle, rm_ref},
                                                            {right_rear,   rr_ref},
                                                            {left_front,   lf_ref},
                                                            {left_middle,  lm_ref},
                                                            {left_rear,    lr_ref}};
    map < hexapod_coxa_joints, const Vector3d > coxa_ref{{right_front,  rf_c},
                                                         {right_middle, rm_c},
                                                         {right_rear,   rr_c},
                                                         {left_front,   lf_c},
                                                         {left_middle,  lm_c},
                                                         {left_rear,    lr_c}};
    
    map< hexapod_coxa_joints, tuple<int, int, int> > legs_coordenates {{right_front,  {0, 6, 12}},
                                                                       {right_middle, {1, 7, 13}},
                                                                       {right_rear,   {2, 8, 14}},
                                                                       {left_front,   {5, 11, 17}},
                                                                       {left_middle,  {4, 10, 16}},
                                                                       {left_rear,    {3, 9, 15}}};

    string command_topics[18] = { // coxas 
                                "/phantomx/j_c1_rf_position_controller/command",		 //0
                                "/phantomx/j_c1_rm_position_controller/command",         //1
                                "/phantomx/j_c1_rr_position_controller/command",         //2
                                "/phantomx/j_c1_lr_position_controller/command",         //3
                                "/phantomx/j_c1_lm_position_controller/command",         //4
                                "/phantomx/j_c1_lf_position_controller/command",         //5
                                // thighs 
                                "/phantomx/j_thigh_rf_position_controller/command",      //6
                                "/phantomx/j_thigh_rm_position_controller/command",      //7
                                "/phantomx/j_thigh_rr_position_controller/command",      //8
                                "/phantomx/j_thigh_lr_position_controller/command",      //9
                                "/phantomx/j_thigh_lm_position_controller/command",      //10
                                "/phantomx/j_thigh_lf_position_controller/command",      //11
                                // tibas 
                                "/phantomx/j_tibia_rf_position_controller/command",      //12
                                "/phantomx/j_tibia_rm_position_controller/command",      //13
                                "/phantomx/j_tibia_rr_position_controller/command",      //14
                                "/phantomx/j_tibia_lr_position_controller/command",	     //15	  
                                "/phantomx/j_tibia_lm_position_controller/command",      //16
                                "/phantomx/j_tibia_lf_position_controller/command",		 //17	
                                };     

	vector<Publisher> publishers;
    Vector3d* coxa_joints;
	Vector3d* foot_tips;
	

public:
    // Constructor
    Phantomx(NodeHandle&);
    ~Phantomx();
};

#endif