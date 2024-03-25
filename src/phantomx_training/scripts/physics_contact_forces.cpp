#include <assert.h> 
#include "physics_contact_forces.h"

// Constructor
PhysicsContactForces::PhysicsContactForces(ros::NodeHandle& nh, ros::Rate& r):rate(r)
{
    // Foot's contact state of the robot
    contact_state_rf=contact_state_rm=contact_state_rr=0;
    contact_state_lf=contact_state_lm=contact_state_lr=0; 
    // contact forces initialization
    contact_force_rf=contact_force_rm=contact_force_rr=0;
    contact_force_lf=contact_force_lm=contact_force_lr=0;
    // shank contact state
    shank_contact_state_rf=shank_contact_state_rm=shank_contact_state_rr=0;
    shank_contact_state_lf=shank_contact_state_lm=shank_contact_state_lr=0;
    // thigh contact state
    thigh_contact_state_rf=thigh_contact_state_rm=thigh_contact_state_rr=0;
    thigh_contact_state_lf=thigh_contact_state_lm=thigh_contact_state_lr=0;
    // Normal vector 
    terreain_normal_rf.resize(3);
    terreain_normal_rm.resize(3);
    terreain_normal_rr.resize(3);
    terreain_normal_lf.resize(3);
    terreain_normal_lm.resize(3);
    terreain_normal_lr.resize(3);
    // initializaze normal:
    terreain_normal_rf[0]=terreain_normal_rf[1]=terreain_normal_rf[2]=0;
    terreain_normal_rm[0]=terreain_normal_rm[1]=terreain_normal_rm[2]=0;
    terreain_normal_rr[0]=terreain_normal_rr[1]=terreain_normal_rr[2]=0;
    terreain_normal_lf[0]=terreain_normal_lf[1]=terreain_normal_lf[2]=0;
    terreain_normal_lm[0]=terreain_normal_lm[1]=terreain_normal_lm[2]=0;
    terreain_normal_lr[0]=terreain_normal_lr[1]=terreain_normal_lr[2]=0;
    // Subscribers for the foot contact forces:
    sub_foot_contact_rf = nh.subscribe<geometry_msgs::Vector3>("/physics/contacts_forces_rf", 1, &PhysicsContactForces::foot_contact_rf_callback, this);
    sub_foot_contact_rm = nh.subscribe<geometry_msgs::Vector3>("/physics/contacts_forces_rm", 1, &PhysicsContactForces::foot_contact_rm_callback, this);
    sub_foot_contact_rr = nh.subscribe<geometry_msgs::Vector3>("/physics/contacts_forces_rr", 1, &PhysicsContactForces::foot_contact_rr_callback, this);
    sub_foot_contact_lf = nh.subscribe<geometry_msgs::Vector3>("/physics/contacts_forces_lf", 1, &PhysicsContactForces::foot_contact_lf_callback, this);
    sub_foot_contact_lm = nh.subscribe<geometry_msgs::Vector3>("/physics/contacts_forces_lm", 1, &PhysicsContactForces::foot_contact_lm_callback, this);
    sub_foot_contact_lr = nh.subscribe<geometry_msgs::Vector3>("/physics/contacts_forces_lr", 1, &PhysicsContactForces::foot_contact_lr_callback, this);
    // Subscribers for the thigh contact forces:
    sub_thigh_contact_rf = nh.subscribe<std_msgs::Float64>("/physics/contacts_forces_thigh_rf", 1, &PhysicsContactForces::thigh_contact_rf_callback, this);
    sub_thigh_contact_rm = nh.subscribe<std_msgs::Float64>("/physics/contacts_forces_thigh_rm", 1, &PhysicsContactForces::thigh_contact_rm_callback, this);
    sub_thigh_contact_rr = nh.subscribe<std_msgs::Float64>("/physics/contacts_forces_thigh_rr", 1, &PhysicsContactForces::thigh_contact_rr_callback, this);
    sub_thigh_contact_lf = nh.subscribe<std_msgs::Float64>("/physics/contacts_forces_thigh_lf", 1, &PhysicsContactForces::thigh_contact_lf_callback, this);
    sub_thigh_contact_lm = nh.subscribe<std_msgs::Float64>("/physics/contacts_forces_thigh_lm", 1, &PhysicsContactForces::thigh_contact_lm_callback, this);
    sub_thigh_contact_lr = nh.subscribe<std_msgs::Float64>("/physics/contacts_forces_thigh_lr", 1, &PhysicsContactForces::thigh_contact_lr_callback, this);
    // Subscribers for the shank contact forces:
    sub_shank_contact_rf = nh.subscribe<std_msgs::Float64>("/physics/contacts_forces_shank_rf", 1, &PhysicsContactForces::shank_contact_rf_callback, this);
    sub_shank_contact_rm = nh.subscribe<std_msgs::Float64>("/physics/contacts_forces_shank_rm", 1, &PhysicsContactForces::shank_contact_rm_callback, this);
    sub_shank_contact_rr = nh.subscribe<std_msgs::Float64>("/physics/contacts_forces_shank_rr", 1, &PhysicsContactForces::shank_contact_rr_callback, this);
    sub_shank_contact_lf = nh.subscribe<std_msgs::Float64>("/physics/contacts_forces_shank_lf", 1, &PhysicsContactForces::shank_contact_lf_callback, this);
    sub_shank_contact_lm = nh.subscribe<std_msgs::Float64>("/physics/contacts_forces_shank_lm", 1, &PhysicsContactForces::shank_contact_lm_callback, this);
    sub_shank_contact_lr = nh.subscribe<std_msgs::Float64>("/physics/contacts_forces_shank_lr", 1, &PhysicsContactForces::shank_contact_lr_callback, this);
}

//Callbacks for the foot contact forces:
// Right front:
void PhysicsContactForces::foot_contact_rf_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    contact_force_rf = std::sqrt(msg->x * msg->x + msg->y * msg->y + msg->z * msg->z);
    contact_state_rf = (contact_force_rf > 0) ? 1 : 0;
    terreain_normal_rf.clear();
    if (abs(contact_force_rf) > 0) 
    {
        terreain_normal_rf.push_back(msg->x / contact_force_rf);
        terreain_normal_rf.push_back(msg->y / contact_force_rf);
        terreain_normal_rf.push_back(msg->z / contact_force_rf);
    }
    else
    {
        terreain_normal_rf.push_back(0);
        terreain_normal_rf.push_back(0);
        terreain_normal_rf.push_back(0);
        contact_force_rf = 0;
    }
    assert(terreain_normal_rf.size() == 3);  
}
// Right middle:
void PhysicsContactForces::foot_contact_rm_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    contact_force_rm = std::sqrt(msg->x * msg->x + msg->y * msg->y + msg->z * msg->z);
    contact_state_rm = (contact_force_rm > 0) ? 1 : 0;

    terreain_normal_rm.clear();
    if (abs(contact_force_rm) > 0) 
    {
        terreain_normal_rm.push_back(msg->x / contact_force_rm);
        terreain_normal_rm.push_back(msg->y / contact_force_rm);
        terreain_normal_rm.push_back(msg->z / contact_force_rm);
    }
    else
    {
        terreain_normal_rm.push_back(0);
        terreain_normal_rm.push_back(0);
        terreain_normal_rm.push_back(0);
        contact_force_rm = 0;
    }
    assert(terreain_normal_rm.size() == 3);
}
// Right rear:
void PhysicsContactForces::foot_contact_rr_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    
    contact_force_rr = std::sqrt(msg->x * msg->x + msg->y * msg->y + msg->z * msg->z);
    contact_state_rr = (contact_force_rr > 0) ? 1 : 0;
    terreain_normal_rr.clear();
    if (abs(contact_force_rr) > 0) 
    {
        terreain_normal_rr.push_back(msg->x / contact_force_rr);
        terreain_normal_rr.push_back(msg->y / contact_force_rr);
        terreain_normal_rr.push_back(msg->z / contact_force_rr);
    }
    else
    {
        terreain_normal_rr.push_back(0);
        terreain_normal_rr.push_back(0);
        terreain_normal_rr.push_back(0);
        contact_force_rr = 0;
    }
    assert(terreain_normal_rr.size() == 3);
}
// Left front:
void PhysicsContactForces::foot_contact_lf_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    contact_force_lf = std::sqrt(msg->x * msg->x + msg->y * msg->y + msg->z * msg->z);
    contact_state_lf = (contact_force_lf > 0) ? 1 : 0;

    terreain_normal_lf.clear();
    if (abs(contact_force_lf) > 0) 
    {
        terreain_normal_lf.push_back(msg->x / contact_force_lf);
        terreain_normal_lf.push_back(msg->y / contact_force_lf);
        terreain_normal_lf.push_back(msg->z / contact_force_lf);
    }
    else
    {
        terreain_normal_lf.push_back(0);
        terreain_normal_lf.push_back(0);
        terreain_normal_lf.push_back(0);
        contact_force_lf = 0;
    }
    assert(terreain_normal_lf.size() == 3);

}
// Left middle:
void PhysicsContactForces::foot_contact_lm_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    contact_force_lm = std::sqrt(msg->x * msg->x + msg->y * msg->y + msg->z * msg->z);
    terreain_normal_lm.clear();
    contact_state_lm = (contact_force_lm > 0) ? 1 : 0;

    if (abs(contact_force_lm) > 0) 
    {
        terreain_normal_lm.push_back(msg->x / contact_force_lm);
        terreain_normal_lm.push_back(msg->y / contact_force_lm);
        terreain_normal_lm.push_back(msg->z / contact_force_lm);
    }
    else
    {
        terreain_normal_lm.push_back(0);
        terreain_normal_lm.push_back(0);
        terreain_normal_lm.push_back(0);
        contact_force_lm = 0;
    }
    assert(terreain_normal_lm.size() == 3);
}
// Left rear:
void PhysicsContactForces::foot_contact_lr_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    contact_force_lr = std::sqrt(msg->x * msg->x + msg->y * msg->y + msg->z * msg->z);
    terreain_normal_lr.clear();
    contact_state_lr = (contact_force_lr > 0) ? 1 : 0;

    if (abs(contact_force_lr) > 0) 
    {
        terreain_normal_lr.push_back(msg->x / contact_force_lr);
        terreain_normal_lr.push_back(msg->y / contact_force_lr);
        terreain_normal_lr.push_back(msg->z / contact_force_lr);
    }
    else
    {
        terreain_normal_lr.push_back(0);
        terreain_normal_lr.push_back(0);
        terreain_normal_lr.push_back(0);
        contact_force_lr = 0;
    }
    assert(terreain_normal_lr.size() == 3);
}
// Callbacks for the thigh contact forces:
//Right front:
void PhysicsContactForces::thigh_contact_rf_callback(const std_msgs::Float64::ConstPtr& msg)
{
    thigh_contact_state_rf = (msg->data > 1e-3) ? 1 : 0;
}
//Right middle:
void PhysicsContactForces::thigh_contact_rm_callback(const std_msgs::Float64::ConstPtr& msg)
{
    thigh_contact_state_rm = (msg->data > 1e-3) ? 1 : 0;
}
//Right rear:
void PhysicsContactForces::thigh_contact_rr_callback(const std_msgs::Float64::ConstPtr& msg)
{
    thigh_contact_state_rr = (msg->data > 1e-3) ? 1 : 0;
}
//Left front:
void PhysicsContactForces::thigh_contact_lf_callback(const std_msgs::Float64::ConstPtr& msg)
{
    thigh_contact_state_lf = (msg->data > 1e-3) ? 1 : 0;
}
//Left middle:
void PhysicsContactForces::thigh_contact_lm_callback(const std_msgs::Float64::ConstPtr& msg)
{
    thigh_contact_state_lm = (msg->data > 1e-3) ? 1 : 0;
}
//Left rear:
void PhysicsContactForces::thigh_contact_lr_callback(const std_msgs::Float64::ConstPtr& msg)
{
    thigh_contact_state_lr = (msg->data > 1e-3) ? 1 : 0;
}

// Callbacks for the shank contact forces:
//Right front:
void PhysicsContactForces::shank_contact_rf_callback(const std_msgs::Float64::ConstPtr& msg)
{
    shank_contact_state_rf = (msg->data > 1e-3) ? 1 : 0;
}
//Right middle:
void PhysicsContactForces::shank_contact_rm_callback(const std_msgs::Float64::ConstPtr& msg)
{
    shank_contact_state_rm = (msg->data > 1e-3) ? 1 : 0;
}
//Right rear:
void PhysicsContactForces::shank_contact_rr_callback(const std_msgs::Float64::ConstPtr& msg)
{
    shank_contact_state_rr = (msg->data > 1e-3) ? 1 : 0;
}
//Left front:
void PhysicsContactForces::shank_contact_lf_callback(const std_msgs::Float64::ConstPtr& msg)
{
    shank_contact_state_lf = (msg->data > 1e-3) ? 1 : 0;
}
//Left middle:
void PhysicsContactForces::shank_contact_lm_callback(const std_msgs::Float64::ConstPtr& msg)
{
    shank_contact_state_lm = (msg->data > 1e-3) ? 1 : 0;
}
//Left rear:
void PhysicsContactForces::shank_contact_lr_callback(const std_msgs::Float64::ConstPtr& msg)
{
    shank_contact_state_lr = (msg->data > 1e-3) ? 1 : 0;
}
// Destructor
PhysicsContactForces::~PhysicsContactForces(){}

