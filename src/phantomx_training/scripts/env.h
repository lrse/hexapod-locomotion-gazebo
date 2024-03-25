#ifndef PHANTOMX_ENV_H
#define PHANTOMX_ENV_H

#include <vector>
#include <cmath>
#include <memory>
#include "loguru/loguru.hpp"
#include <ros/ros.h>
#include "joint_states.h"
#include "physics_contact_forces.h"
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#define PI M_PI

class PhantomxEnv : public JointStates, public PhysicsContactForces {

private:

     // ros topics handle
    ros::Subscriber sub_ground_truth;
    // PHASE subscribers
    /*LEFT SIDE*/
    ros::Subscriber sub_phase_lf;
    ros::Subscriber sub_phase_lm;
    ros::Subscriber sub_phase_lr;
    /*RIGHT SIDE*/
    ros::Subscriber sub_phase_rf;
    ros::Subscriber sub_phase_rm;
    ros::Subscriber sub_phase_rr;

    // R subscribers
    /*LEFT SIDE*/
    ros::Subscriber sub_r_lf;
    ros::Subscriber sub_r_lm;
    ros::Subscriber sub_r_lr;
    /*RIGHT SIDE*/
    ros::Subscriber sub_r_rf;
    ros::Subscriber sub_r_rm;
    ros::Subscriber sub_r_rr;

    // HEIGHTMAPS subscribers
    ros::Subscriber sub_heightmap_lf;
    ros::Subscriber sub_heightmap_lm;
    ros::Subscriber sub_heightmap_lr;
    ros::Subscriber sub_heightmap_rf;
    ros::Subscriber sub_heightmap_rm;
    ros::Subscriber sub_heightmap_rr;
    // BASE FREQUENCY subscriber
    ros::Subscriber sub_base_frec;

    // ROS publishers (delta_r)
    ros::Publisher pub_delta_r_rf;
    ros::Publisher pub_delta_r_rm;
    ros::Publisher pub_delta_r_rr;
    ros::Publisher pub_delta_r_lf;
    ros::Publisher pub_delta_r_lm;
    ros::Publisher pub_delta_r_lr;
    // ROS publishers (frec)
    ros::Publisher pub_freq_rf;
    ros::Publisher pub_freq_rm;
    ros::Publisher pub_freq_rr;
    ros::Publisher pub_freq_lf;
    ros::Publisher pub_freq_lm;
    ros::Publisher pub_freq_lr;
    // ROS publishers (initial phase)
    ros::Publisher pub_phi0_rf;
    ros::Publisher pub_phi0_rm;
    ros::Publisher pub_phi0_rr;
    ros::Publisher pub_phi0_lf;
    ros::Publisher pub_phi0_lm;
    ros::Publisher pub_phi0_lr;

    // service client
    ros::ServiceClient resetSimulationClient;
    std_srvs::Empty pause_req;
    std_srvs::Empty unpause_req;
    ros::ServiceClient pauseClient;
    ros::ServiceClient unpauseClient;

    // Robot state
    ros::Rate rate;
    tf2::Quaternion q;
    /* ROBOT POSITION */
    float position_x;
    float position_y;
    float position_z;
    /* ROBOT VELOCITY */
    float velocity_x;
    float velocity_y;
    float velocity_z;
    /* ROBOT */
    float angular_rate_x;
    float angular_rate_y;
    float angular_rate_z;

    float base_frec=0;
    double roll, pitch, yaw;
    /* ROBOT'S PHASES */
    FixedQueue<float, 2> phase_rf;
    FixedQueue<float, 2> phase_rm;
    FixedQueue<float, 2> phase_rr;
    FixedQueue<float, 2> phase_lf;
    FixedQueue<float, 2> phase_lm;
    FixedQueue<float, 2> phase_lr;
    /*ROBOTS's targets R historial */
    FixedQueue<std::vector<float>, 3> r_rf_targets;
    FixedQueue<std::vector<float>, 3> r_rm_targets;
    FixedQueue<std::vector<float>, 3> r_rr_targets;
    FixedQueue<std::vector<float>, 3> r_lf_targets;
    FixedQueue<std::vector<float>, 3> r_lm_targets;
    FixedQueue<std::vector<float>, 3> r_lr_targets;
    /* HEIGHTMAPS  */
    std::vector<float> heightmap_rf;
    std::vector<float> heightmap_rm;
    std::vector<float> heightmap_rr;
    std::vector<float> heightmap_lf;
    std::vector<float> heightmap_lm;
    std::vector<float> heightmap_lr;
    
    
public:
    // constructor
    PhantomxEnv(ros::NodeHandle& nh, ros::Rate& r);
    // ground truth base link callback
    void groundTruthBaseLinkCallback(const nav_msgs::Odometry::ConstPtr& msg);
    // step simulation
    std::shared_ptr<std::vector<float>> step(std::vector<float>& action);
    // nn_return
    std::shared_ptr<std::vector<float>> getObsvector(void);
    // phase callback
    /* LEFT SIDE */
    void phase_lf_callback(const std_msgs::Float64::ConstPtr& msg);
    void phase_lm_callback(const std_msgs::Float64::ConstPtr& msg);
    void phase_lr_callback(const std_msgs::Float64::ConstPtr& msg);
    /* RIGHT SIDE */
    void phase_rf_callback(const std_msgs::Float64::ConstPtr& msg);
    void phase_rm_callback(const std_msgs::Float64::ConstPtr& msg);
    void phase_rr_callback(const std_msgs::Float64::ConstPtr& msg);
    // R callback
    /* LEFT SIDE */
    void r_lf_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void r_lm_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void r_lr_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    /* RIGHT SIDE */
    void r_rf_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void r_rm_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void r_rr_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    // heightmaps callback
    void heightmap_lf_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void heightmap_lm_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void heightmap_lr_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void heightmap_rf_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void heightmap_rm_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void heightmap_rr_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    // base frequency
    void base_frec_callback(const std_msgs::Float64::ConstPtr& msg);
    // destructor
    ~PhantomxEnv();
};

#endif //PHANTOMX_ENV_H