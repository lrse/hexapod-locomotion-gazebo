#ifndef PHYSICS_CONTACTS_H
#define PHYSICS_CONTACTS_H

#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>

class PhysicsContactForces
{

protected:

    ros::NodeHandle nh;
    ros::Rate rate;
    //Foot's contact state of the robot 
    float contact_state_rf=0;
    float contact_state_rm=0;
    float contact_state_rr=0;
    float contact_state_lf=0;
    float contact_state_lm=0;
    float contact_state_lr=0;
    // thigh contact state
    float thigh_contact_state_rf=0;
    float thigh_contact_state_rm=0;
    float thigh_contact_state_rr=0;
    float thigh_contact_state_lf=0;
    float thigh_contact_state_lm=0;
    float thigh_contact_state_lr=0;
    // shank contact state
    float shank_contact_state_rf=0;
    float shank_contact_state_rm=0;
    float shank_contact_state_rr=0;
    float shank_contact_state_lf=0;
    float shank_contact_state_lm=0;
    float shank_contact_state_lr=0;
    // Foot's contact normal vector of the robot
    std::vector<float> terreain_normal_rf;
    std::vector<float> terreain_normal_rm;
    std::vector<float> terreain_normal_rr;
    std::vector<float> terreain_normal_lf;
    std::vector<float> terreain_normal_lm;
    std::vector<float> terreain_normal_lr;
    // contact forces
    float contact_force_rf=0;
    float contact_force_rm=0;
    float contact_force_rr=0;
    float contact_force_lf=0;
    float contact_force_lm=0;
    float contact_force_lr=0;
    // Subscribers Foot's contact forces per side of the robot
    ros::Subscriber sub_foot_contact_rf;
    ros::Subscriber sub_foot_contact_rm;
    ros::Subscriber sub_foot_contact_rr;
    ros::Subscriber sub_foot_contact_lf;
    ros::Subscriber sub_foot_contact_lm;
    ros::Subscriber sub_foot_contact_lr;
    // Subscribers thigh contact forces per side of the robot
    ros::Subscriber sub_thigh_contact_rf;
    ros::Subscriber sub_thigh_contact_rm;
    ros::Subscriber sub_thigh_contact_rr;
    ros::Subscriber sub_thigh_contact_lf;
    ros::Subscriber sub_thigh_contact_lm;
    ros::Subscriber sub_thigh_contact_lr;
    // Subscribers shank contact forces per side of the robot
    ros::Subscriber sub_shank_contact_rf;
    ros::Subscriber sub_shank_contact_rm;
    ros::Subscriber sub_shank_contact_rr;
    ros::Subscriber sub_shank_contact_lf;
    ros::Subscriber sub_shank_contact_lm;
    ros::Subscriber sub_shank_contact_lr;

public:

    // Constructor
    PhysicsContactForces(ros::NodeHandle& nh, ros::Rate& r);
    // Callbacks for the foot contact forces:
    void foot_contact_rf_callback(const geometry_msgs::Vector3::ConstPtr& msg);
    void foot_contact_rm_callback(const geometry_msgs::Vector3::ConstPtr& msg);
    void foot_contact_rr_callback(const geometry_msgs::Vector3::ConstPtr& msg);
    void foot_contact_lf_callback(const geometry_msgs::Vector3::ConstPtr& msg);
    void foot_contact_lm_callback(const geometry_msgs::Vector3::ConstPtr& msg);
    void foot_contact_lr_callback(const geometry_msgs::Vector3::ConstPtr& msg);
    // Callbacks for thigh contacts
    void thigh_contact_rf_callback(const std_msgs::Float64::ConstPtr& msg);
    void thigh_contact_rm_callback(const std_msgs::Float64::ConstPtr& msg);
    void thigh_contact_rr_callback(const std_msgs::Float64::ConstPtr& msg);
    void thigh_contact_lf_callback(const std_msgs::Float64::ConstPtr& msg);
    void thigh_contact_lm_callback(const std_msgs::Float64::ConstPtr& msg);
    void thigh_contact_lr_callback(const std_msgs::Float64::ConstPtr& msg);
    // Callbacks Shank contacts
    void shank_contact_rf_callback(const std_msgs::Float64::ConstPtr& msg);
    void shank_contact_rm_callback(const std_msgs::Float64::ConstPtr& msg);
    void shank_contact_rr_callback(const std_msgs::Float64::ConstPtr& msg);
    void shank_contact_lf_callback(const std_msgs::Float64::ConstPtr& msg);
    void shank_contact_lm_callback(const std_msgs::Float64::ConstPtr& msg);
    void shank_contact_lr_callback(const std_msgs::Float64::ConstPtr& msg);
    // Destructor
    ~PhysicsContactForces();
};


#endif // ENV_H