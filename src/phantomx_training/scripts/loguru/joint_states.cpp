#include "joint_states.h"


JointStates::JointStates(ros::NodeHandle& nh, ros::Rate& r):rate(r) 
{
    NHistory = 3;
    NSides = 6;
    JointPerLeg = 3;

    //Positions:
    tibia_positions["rf"] = 0.0; thigh_positions["rf"] = 0.0; c1_positions["rf"] = 0.0;
    tibia_positions["rm"] = 0.0; thigh_positions["rm"] = 0.0; c1_positions["rm"] = 0.0;
    tibia_positions["rr"] = 0.0; thigh_positions["rr"] = 0.0; c1_positions["rr"] = 0.0;
    tibia_positions["lf"] = 0.0; thigh_positions["lf"] = 0.0; c1_positions["lf"] = 0.0;
    tibia_positions["lm"] = 0.0; thigh_positions["lm"] = 0.0; c1_positions["lm"] = 0.0;
    tibia_positions["lr"] = 0.0; thigh_positions["lr"] = 0.0; c1_positions["lr"] = 0.0;
    //Velocities:
    tibia_velocities["rf"] = 0.0; thigh_velocities["rf"] = 0.0; c1_velocities["rf"] = 0.0;
    tibia_velocities["rm"] = 0.0; thigh_velocities["rm"] = 0.0; c1_velocities["rm"] = 0.0;
    tibia_velocities["rr"] = 0.0; thigh_velocities["rr"] = 0.0; c1_velocities["rr"] = 0.0;
    tibia_velocities["lf"] = 0.0; thigh_velocities["lf"] = 0.0; c1_velocities["lf"] = 0.0;
    tibia_velocities["lm"] = 0.0; thigh_velocities["lm"] = 0.0; c1_velocities["lm"] = 0.0;
    tibia_velocities["lr"] = 0.0; thigh_velocities["lr"] = 0.0; c1_velocities["lr"] = 0.0;

    // initialize queues: errror and velocity
    for (int i = 0; i < NHistory; i++)
    {   
        // errors
        q_data_rf_tibia_error.push(0); q_data_rf_thigh_error.push(0); q_data_rf_c1_error.push(0);
        q_data_rm_tibia_error.push(0); q_data_rm_thigh_error.push(0); q_data_rm_c1_error.push(0);
        q_data_rr_tibia_error.push(0); q_data_rr_thigh_error.push(0); q_data_rr_c1_error.push(0);
        q_data_lf_tibia_error.push(0); q_data_lf_thigh_error.push(0); q_data_lf_c1_error.push(0);
        q_data_lm_tibia_error.push(0); q_data_lm_thigh_error.push(0); q_data_lm_c1_error.push(0);
        q_data_lr_tibia_error.push(0); q_data_lr_thigh_error.push(0); q_data_lr_c1_error.push(0);
        // velocities
        q_data_rf_tibia_velocity.push(0); q_data_rf_thigh_velocity.push(0); q_data_rf_c1_velocity.push(0);
        q_data_rm_tibia_velocity.push(0); q_data_rm_thigh_velocity.push(0); q_data_rm_c1_velocity.push(0);
        q_data_rr_tibia_velocity.push(0); q_data_rr_thigh_velocity.push(0); q_data_rr_c1_velocity.push(0);
        q_data_lf_tibia_velocity.push(0); q_data_lf_thigh_velocity.push(0); q_data_lf_c1_velocity.push(0);
        q_data_lm_tibia_velocity.push(0); q_data_lm_thigh_velocity.push(0); q_data_lm_c1_velocity.push(0);
        q_data_lr_tibia_velocity.push(0); q_data_lr_thigh_velocity.push(0); q_data_lr_c1_velocity.push(0);
    }
    // ros topics handle
    //Right Front callbacks:
    sub_rf_c1    = nh.subscribe<control_msgs::JointControllerState>("/phantomx/j_c1_rf_position_controller/state", 1,     &JointStates::rf_c1_callback, this);
    sub_rf_thigh = nh.subscribe<control_msgs::JointControllerState>("/phantomx/j_thigh_rf_position_controller/state", 1,  &JointStates::rf_thigh_callback, this);
    sub_rf_tibia = nh.subscribe<control_msgs::JointControllerState>("/phantomx/j_tibia_rf_position_controller/state", 1,  &JointStates::rf_tibia_callback, this);
    //Right Middle callbacks:
    sub_rm_c1    = nh.subscribe<control_msgs::JointControllerState>("/phantomx/j_c1_rm_position_controller/state", 1,     &JointStates::rm_c1_callback, this);
    sub_rm_thigh = nh.subscribe<control_msgs::JointControllerState>("/phantomx/j_thigh_rm_position_controller/state", 1,  &JointStates::rm_thigh_callback, this);
    sub_rm_tibia = nh.subscribe<control_msgs::JointControllerState>("/phantomx/j_tibia_rm_position_controller/state", 1,  &JointStates::rm_tibia_callback, this);
    //Right Rear callbacks:
    sub_rr_c1    = nh.subscribe<control_msgs::JointControllerState>("/phantomx/j_c1_rr_position_controller/state", 1,     &JointStates::rr_c1_callback, this);
    sub_rr_thigh = nh.subscribe<control_msgs::JointControllerState>("/phantomx/j_thigh_rr_position_controller/state", 1,  &JointStates::rr_thigh_callback, this);
    sub_rr_tibia = nh.subscribe<control_msgs::JointControllerState>("/phantomx/j_tibia_rr_position_controller/state", 1,  &JointStates::rr_tibia_callback, this);
    //Left Front callbacks:
    sub_lf_c1    = nh.subscribe<control_msgs::JointControllerState>("/phantomx/j_c1_lf_position_controller/state", 1,     &JointStates::lf_c1_callback, this);
    sub_lf_thigh = nh.subscribe<control_msgs::JointControllerState>("/phantomx/j_thigh_lf_position_controller/state", 1,  &JointStates::lf_thigh_callback, this);
    sub_lf_tibia = nh.subscribe<control_msgs::JointControllerState>("/phantomx/j_tibia_lf_position_controller/state", 1,  &JointStates::lf_tibia_callback, this);
    //Left Middle callbacks:
    sub_lm_c1    = nh.subscribe<control_msgs::JointControllerState>("/phantomx/j_c1_lm_position_controller/state", 1,     &JointStates::lm_c1_callback, this);
    sub_lm_thigh = nh.subscribe<control_msgs::JointControllerState>("/phantomx/j_thigh_lm_position_controller/state", 1,  &JointStates::lm_thigh_callback, this);
    sub_lm_tibia = nh.subscribe<control_msgs::JointControllerState>("/phantomx/j_tibia_lm_position_controller/state", 1,  &JointStates::lm_tibia_callback, this);
    //Left Rear callbacks:
    sub_lr_c1    = nh.subscribe<control_msgs::JointControllerState>("/phantomx/j_c1_lr_position_controller/state", 1,     &JointStates::lr_c1_callback, this);
    sub_lr_thigh = nh.subscribe<control_msgs::JointControllerState>("/phantomx/j_thigh_lr_position_controller/state", 1,  &JointStates::lr_thigh_callback, this);
    sub_lr_tibia = nh.subscribe<control_msgs::JointControllerState>("/phantomx/j_tibia_lr_position_controller/state", 1,  &JointStates::lr_tibia_callback, this);
}
// Roght Front callbacks:
// c1
void JointStates::rf_c1_callback(const control_msgs::JointControllerState::ConstPtr& msg)
{
    c1_positions["rf"] = msg->process_value;
    c1_velocities["rf"] = msg->process_value_dot;
    q_data_rf_c1_error.push(msg->error);
    q_data_rf_c1_velocity.push(msg->process_value_dot);
    return;
}
// thigh
void JointStates::rf_thigh_callback(const control_msgs::JointControllerState::ConstPtr& msg)
{
    thigh_positions["rf"] = msg->process_value;
    thigh_velocities["rf"] = msg->process_value_dot;
    q_data_rf_thigh_error.push(msg->error);
    q_data_rf_thigh_velocity.push(msg->process_value_dot);
    return;
}
// tiba
void JointStates::rf_tibia_callback(const control_msgs::JointControllerState::ConstPtr& msg)
{
    tibia_positions["rf"] = msg->process_value;
    tibia_velocities["rf"] = msg->process_value_dot;
    q_data_rf_tibia_error.push(msg->error);
    q_data_rf_tibia_velocity.push(msg->process_value_dot);
    return;
}

// Right Middle callbacks:
// c1
void JointStates::rm_c1_callback(const control_msgs::JointControllerState::ConstPtr& msg)
{
    c1_positions["rm"] = msg->process_value;
    c1_velocities["rm"] = msg->process_value_dot;
    q_data_rm_c1_error.push(msg->error);
    q_data_rm_c1_velocity.push(msg->process_value_dot);
    return;
}
// thigh
void JointStates::rm_thigh_callback(const control_msgs::JointControllerState::ConstPtr& msg)
{
    thigh_positions["rm"] = msg->process_value;
    thigh_velocities["rm"] = msg->process_value_dot;
    q_data_rm_thigh_error.push(msg->error);
    q_data_rm_thigh_velocity.push(msg->process_value_dot);
    return;
}
// tiba
void JointStates::rm_tibia_callback(const control_msgs::JointControllerState::ConstPtr& msg)
{
    tibia_positions["rm"] = msg->process_value;
    tibia_velocities["rm"] = msg->process_value_dot;
    q_data_rm_tibia_error.push(msg->error);
    q_data_rm_tibia_velocity.push(msg->process_value_dot);
    return;
}

// Right Rear callbacks:
// c1
void JointStates::rr_c1_callback(const control_msgs::JointControllerState::ConstPtr& msg)
{
    c1_positions["rr"] = msg->process_value;
    c1_velocities["rr"] = msg->process_value_dot;
    q_data_rr_c1_error.push(msg->error);
    q_data_rr_c1_velocity.push(msg->process_value_dot);
    return;
}
// thigh
void JointStates::rr_thigh_callback(const control_msgs::JointControllerState::ConstPtr& msg)
{
    thigh_positions["rr"] = msg->process_value;
    thigh_velocities["rr"] = msg->process_value_dot;
    q_data_rr_thigh_error.push(msg->error);
    q_data_rr_thigh_velocity.push(msg->process_value_dot);
    return;
}
// tiba
void JointStates::rr_tibia_callback(const control_msgs::JointControllerState::ConstPtr& msg)
{
    tibia_positions["rr"] = msg->process_value;
    tibia_velocities["rr"] = msg->process_value_dot;
    q_data_rr_tibia_error.push(msg->error);
    q_data_rr_tibia_velocity.push(msg->process_value_dot);
    return;
}

// Left Front callbacks:
// c1
void JointStates::lf_c1_callback(const control_msgs::JointControllerState::ConstPtr& msg)
{
    c1_positions["lf"] = msg->process_value;
    c1_velocities["lf"] = msg->process_value_dot;
    q_data_lf_c1_error.push(msg->error);
    q_data_lf_c1_velocity.push(msg->process_value_dot);
    return;
}
// thigh
void JointStates::lf_thigh_callback(const control_msgs::JointControllerState::ConstPtr& msg)
{
    thigh_positions["lf"] = msg->process_value;
    thigh_velocities["lf"] = msg->process_value_dot;
    q_data_lf_thigh_error.push(msg->error);
    q_data_lf_thigh_velocity.push(msg->process_value_dot);
    return;
}
// tiba
void JointStates::lf_tibia_callback(const control_msgs::JointControllerState::ConstPtr& msg)
{
    tibia_positions["lf"] = msg->process_value;
    tibia_velocities["lf"] = msg->process_value_dot;
    q_data_lf_tibia_error.push(msg->error);
    q_data_lf_tibia_velocity.push(msg->process_value_dot);
    return;
}
// Left Middle callbacks:
// c1
void JointStates::lm_c1_callback(const control_msgs::JointControllerState::ConstPtr& msg)
{
    c1_positions["lm"] = msg->process_value;
    c1_velocities["lm"] = msg->process_value_dot;
    q_data_lm_c1_error.push(msg->error);
    q_data_lm_c1_velocity.push(msg->process_value_dot);
    return;
}
// thigh
void JointStates::lm_thigh_callback(const control_msgs::JointControllerState::ConstPtr& msg)
{
    thigh_positions["lm"] = msg->process_value;
    thigh_velocities["lm"] = msg->process_value_dot;
    q_data_lm_thigh_error.push(msg->error);
    q_data_lm_thigh_velocity.push(msg->process_value_dot);
    return;
}
// tiba
void JointStates::lm_tibia_callback(const control_msgs::JointControllerState::ConstPtr& msg)
{
    tibia_positions["lm"] = msg->process_value;
    tibia_velocities["lm"] = msg->process_value_dot;
    q_data_lm_tibia_error.push(msg->error);
    q_data_lm_tibia_velocity.push(msg->process_value_dot);
    return;
}
// Left Rear callbacks:
// c1
void JointStates::lr_c1_callback(const control_msgs::JointControllerState::ConstPtr& msg)
{
    c1_positions["lr"] = msg->process_value;
    c1_velocities["lr"] = msg->process_value_dot;
    q_data_lr_c1_error.push(msg->error);
    q_data_lr_c1_velocity.push(msg->process_value_dot);
    return;
}
// thigh
void JointStates::lr_thigh_callback(const control_msgs::JointControllerState::ConstPtr& msg)
{
    thigh_positions["lr"] = msg->process_value;
    thigh_velocities["lr"] = msg->process_value_dot;
    q_data_lr_thigh_error.push(msg->error);
    q_data_lr_thigh_velocity.push(msg->process_value_dot);
    return;
}
// tiba
void JointStates::lr_tibia_callback(const control_msgs::JointControllerState::ConstPtr& msg)
{
    tibia_positions["lr"] = msg->process_value;
    tibia_velocities["lr"] = msg->process_value_dot;
    q_data_lr_tibia_error.push(msg->error);
    q_data_lr_tibia_velocity.push(msg->process_value_dot);

    //queue content
    /*std::cout << "q_data_lr_tibia_error: ";
    for (int i = 0; i < NHistory; i++)
    {
        std::cout << q_data_lr_tibia_error[i] << " ";
    }
    std::cout << "\n";*/
    return;
}

JointStates::~JointStates(){}