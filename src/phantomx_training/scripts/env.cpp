#include "env.h"
#include <assert.h> 
#include <random>

using namespace std;
using namespace ros;
using namespace std_msgs;

PhantomxEnv::PhantomxEnv(ros::NodeHandle& nh, ros::Rate& r):
JointStates(nh, r), 
PhysicsContactForces(nh, r),
rate(r)
{
    /*Base frec*/
    base_frec = 0.0;
    /* Init phases: */
    phase_rf.push(0.0); phase_rf.push(0.0);
    phase_rm.push(0.0); phase_rm.push(0.0);
    phase_rr.push(0.0); phase_rr.push(0.0);
    phase_lf.push(0.0); phase_lf.push(0.0);
    phase_lm.push(0.0); phase_lm.push(0.0);
    phase_lr.push(0.0); phase_lr.push(0.0);
    /* Init R targets: */
    std::vector<float> nullVector = {0.0, 0.0, 0.0};
    r_rf_targets.push(nullVector); r_rf_targets.push(nullVector); r_rf_targets.push(nullVector);
    r_rm_targets.push(nullVector); r_rm_targets.push(nullVector); r_rm_targets.push(nullVector);
    r_rr_targets.push(nullVector); r_rr_targets.push(nullVector); r_rr_targets.push(nullVector);
    r_lf_targets.push(nullVector); r_lf_targets.push(nullVector); r_lf_targets.push(nullVector);
    r_lm_targets.push(nullVector); r_lm_targets.push(nullVector); r_lm_targets.push(nullVector);
    r_lr_targets.push(nullVector); r_lr_targets.push(nullVector); r_lr_targets.push(nullVector);
    // clear vector:
    heightmap_rf.clear(); heightmap_rm.clear(); heightmap_rr.clear();
    heightmap_lf.clear(); heightmap_lm.clear(); heightmap_lr.clear();
    heightmap_rf.resize(9); heightmap_rm.resize(9); heightmap_rr.resize(9);
    heightmap_lf.resize(9); heightmap_lm.resize(9); heightmap_lr.resize(9);
    
    for(unsigned int i=0; i<heightmap_rf.size(); i++)
    {
        heightmap_rf[i]=0.0; heightmap_rm[i]=0.0; heightmap_rr[i]=0.0;
        heightmap_lf[i]=0.0; heightmap_lm[i]=0.0; heightmap_lr[i]=0.0;
    }

    assert(heightmap_rf.size() == 9);
    assert(heightmap_rm.size() == 9);
    assert(heightmap_rr.size() == 9);
    assert(heightmap_lf.size() == 9);
    assert(heightmap_lm.size() == 9);
    assert(heightmap_lr.size() == 9);
    /* Init R error history: */
    /*  pub delta R */
    pub_delta_r_rf = nh.advertise<std_msgs::Float64MultiArray>("/phantomx/delta_r/right_front", 1);
    pub_delta_r_rm = nh.advertise<std_msgs::Float64MultiArray>("/phantomx/delta_r/right_middle", 1);
    pub_delta_r_rr = nh.advertise<std_msgs::Float64MultiArray>("/phantomx/delta_r/right_rear", 1);
    pub_delta_r_lf = nh.advertise<std_msgs::Float64MultiArray>("/phantomx/delta_r/left_front", 1);
    pub_delta_r_lm = nh.advertise<std_msgs::Float64MultiArray>("/phantomx/delta_r/left_middle", 1);
    pub_delta_r_lr = nh.advertise<std_msgs::Float64MultiArray>("/phantomx/delta_r/left_rear", 1);
    /*  pub frec */
    pub_freq_rf = nh.advertise<std_msgs::Float64>("/phantomx/frec/right_front", 1);
    pub_freq_rm = nh.advertise<std_msgs::Float64>("/phantomx/frec/right_middle", 1);
    pub_freq_rr = nh.advertise<std_msgs::Float64>("/phantomx/frec/right_rear", 1);
    pub_freq_lf = nh.advertise<std_msgs::Float64>("/phantomx/frec/left_front", 1);
    pub_freq_lm = nh.advertise<std_msgs::Float64>("/phantomx/frec/left_middle", 1);
    pub_freq_lr = nh.advertise<std_msgs::Float64>("/phantomx/frec/left_rear", 1);
    /*  pub initial phase */
    pub_phi0_rf = nh.advertise<std_msgs::Float64>("/phantomx/initial_phase/right_front", 1);
    pub_phi0_rm = nh.advertise<std_msgs::Float64>("/phantomx/initial_phase/right_middle", 1);
    pub_phi0_rr = nh.advertise<std_msgs::Float64>("/phantomx/initial_phase/right_rear", 1);
    pub_phi0_lf = nh.advertise<std_msgs::Float64>("/phantomx/initial_phase/left_rear", 1);
    pub_phi0_lm = nh.advertise<std_msgs::Float64>("/phantomx/initial_phase/left_middle", 1);
    pub_phi0_lr = nh.advertise<std_msgs::Float64>("/phantomx/initial_phase/left_front", 1);
    // Constructor
    sub_ground_truth = nh.subscribe<nav_msgs::Odometry>("/ground_truth/base_link/state", 1, 
                                                        &PhantomxEnv::groundTruthBaseLinkCallback, 
                                                        this);
    /*PHASE SUBSCRIBERS: */
    /* LEFT SIDE */
    sub_phase_lf = nh.subscribe<std_msgs::Float64>("/phantomx/phase/left_front", 1,  &PhantomxEnv::phase_lf_callback, this);
    sub_phase_lm = nh.subscribe<std_msgs::Float64>("/phantomx/phase/left_middle", 1, &PhantomxEnv::phase_lm_callback, this);
    sub_phase_lr = nh.subscribe<std_msgs::Float64>("/phantomx/phase/left_rear", 1,   &PhantomxEnv::phase_lr_callback, this);
    /* RIGHT SIDE */
    sub_phase_rf = nh.subscribe<std_msgs::Float64>("/phantomx/phase/right_front", 1,  &PhantomxEnv::phase_rf_callback, this);
    sub_phase_rm = nh.subscribe<std_msgs::Float64>("/phantomx/phase/right_middle", 1, &PhantomxEnv::phase_rm_callback, this);
    sub_phase_rr = nh.subscribe<std_msgs::Float64>("/phantomx/phase/right_rear", 1,   &PhantomxEnv::phase_rr_callback, this);
    /* R SUBSCRIBERS: */
    /* LEFT SIDE */
    sub_r_lf = nh.subscribe<std_msgs::Float64MultiArray>("/phantomx/r/left_front", 1,  &PhantomxEnv::r_lf_callback, this);
    sub_r_lm = nh.subscribe<std_msgs::Float64MultiArray>("/phantomx/r/left_middle", 1, &PhantomxEnv::r_lm_callback, this);
    sub_r_lr = nh.subscribe<std_msgs::Float64MultiArray>("/phantomx/r/left_rear", 1,   &PhantomxEnv::r_lr_callback, this);
    /* RIGHT SIDE */
    sub_r_rf = nh.subscribe<std_msgs::Float64MultiArray>("/phantomx/r/right_front", 1,  &PhantomxEnv::r_rf_callback, this);
    sub_r_rm = nh.subscribe<std_msgs::Float64MultiArray>("/phantomx/r/right_middle", 1, &PhantomxEnv::r_rm_callback, this);
    sub_r_rr = nh.subscribe<std_msgs::Float64MultiArray>("/phantomx/r/right_rear", 1,   &PhantomxEnv::r_rr_callback, this);
    /*HEIGHTMAPS SUBSCRIBERS */
    sub_heightmap_lf = nh.subscribe<std_msgs::Float64MultiArray>("/heightmaps/lf_leg", 1, &PhantomxEnv::heightmap_lf_callback, this);
    sub_heightmap_lm = nh.subscribe<std_msgs::Float64MultiArray>("/heightmaps/lm_leg", 1, &PhantomxEnv::heightmap_lm_callback, this);
    sub_heightmap_lr = nh.subscribe<std_msgs::Float64MultiArray>("/heightmaps/lr_leg", 1, &PhantomxEnv::heightmap_lr_callback, this);
    sub_heightmap_rf = nh.subscribe<std_msgs::Float64MultiArray>("/heightmaps/rf_leg", 1, &PhantomxEnv::heightmap_rf_callback, this);
    sub_heightmap_rm = nh.subscribe<std_msgs::Float64MultiArray>("/heightmaps/rm_leg", 1, &PhantomxEnv::heightmap_rm_callback, this);
    sub_heightmap_rr = nh.subscribe<std_msgs::Float64MultiArray>("/heightmaps/rr_leg", 1, &PhantomxEnv::heightmap_rr_callback, this);
    /*BASE FREQUENCY SUBSCRIBER */
    sub_base_frec = nh.subscribe<std_msgs::Float64>("/phantomx/base_frec", 1, &PhantomxEnv::base_frec_callback, this);
    /* Reset the simulation service client */
    resetSimulationClient = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
    /* Pause and unpause the simulation */
    pauseClient   = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    unpauseClient = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    // Reset the simulation
    //LOG_F(INFO,"C++'s Phantomx environment initialized");
}

void PhantomxEnv::base_frec_callback(const std_msgs::Float64::ConstPtr& msg)
{
    base_frec = msg->data;
}

void PhantomxEnv::groundTruthBaseLinkCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Position of the robot
    position_x = msg->pose.pose.position.x;
    position_y = msg->pose.pose.position.y;
    position_z = msg->pose.pose.position.z;
    // Linear velocity of the robot
    velocity_x = msg->twist.twist.linear.x;
    velocity_y = msg->twist.twist.linear.y;
    velocity_z = msg->twist.twist.linear.z;
    // Angular rate of the robot
    angular_rate_x = msg->twist.twist.angular.x;
    angular_rate_y = msg->twist.twist.angular.y;
    angular_rate_z = msg->twist.twist.angular.z;
    // Quaternion to Euler angles
    tf2::Quaternion _q(msg->pose.pose.orientation.x, 
                       msg->pose.pose.orientation.y, 
                       msg->pose.pose.orientation.z, 
                       msg->pose.pose.orientation.w);
    q=_q;
}

/**
 * Takes a step in the environment using the given action.
 * @param action The action to take in the environment.
 * @return A shared pointer to a vector of floats representing the new state of the environment.
 */

std::shared_ptr<std::vector<float>> PhantomxEnv::step(std::vector<float>& action) 
{
    float delta_r_scale = 5;//2.43;//5.0;
    float freq_scale = 1;//50;//1.0;
    //unpause physics:
    unpauseClient.call(unpause_req);
    //ros::Time start_time = ros::Time::now();
    // Front Right
    std_msgs::Float64MultiArray rf_delta_r_msg;
    rf_delta_r_msg.data.push_back(action[0]*delta_r_scale);
    rf_delta_r_msg.data.push_back(action[1]*delta_r_scale);
    rf_delta_r_msg.data.push_back(action[2]*delta_r_scale);
    assert(rf_delta_r_msg.data.size() == 3);
    // Middle Right
    std_msgs::Float64MultiArray rm_delta_r_msg;
    rm_delta_r_msg.data.push_back(action[3]*delta_r_scale);
    rm_delta_r_msg.data.push_back(action[4]*delta_r_scale);
    rm_delta_r_msg.data.push_back(action[5]*delta_r_scale);
    assert(rm_delta_r_msg.data.size() == 3);
    // Rear Right
    std_msgs::Float64MultiArray rr_delta_r_msg;
    rr_delta_r_msg.data.push_back(action[6]*delta_r_scale);
    rr_delta_r_msg.data.push_back(action[7]*delta_r_scale);
    rr_delta_r_msg.data.push_back(action[8]*delta_r_scale);
    assert(rr_delta_r_msg.data.size() == 3);
    // Front Left
    std_msgs::Float64MultiArray lf_delta_r_msg;
    lf_delta_r_msg.data.push_back(action[9]*delta_r_scale);
    lf_delta_r_msg.data.push_back(action[10]*delta_r_scale);
    lf_delta_r_msg.data.push_back(action[11]*delta_r_scale);
    assert(lf_delta_r_msg.data.size() == 3);
    // Middle Left
    std_msgs::Float64MultiArray lm_delta_r_msg;
    lm_delta_r_msg.data.push_back(action[12]*delta_r_scale);
    lm_delta_r_msg.data.push_back(action[13]*delta_r_scale);
    lm_delta_r_msg.data.push_back(action[14]*delta_r_scale);
    assert(lm_delta_r_msg.data.size() == 3);
    // Rear Left
    std_msgs::Float64MultiArray lr_delta_r_msg;
    lr_delta_r_msg.data.push_back(action[15]*delta_r_scale);
    lr_delta_r_msg.data.push_back(action[16]*delta_r_scale);
    lr_delta_r_msg.data.push_back(action[17]*delta_r_scale);
    assert(lr_delta_r_msg.data.size() == 3);
    // Frequencies
    std_msgs::Float64 rf_frec_msg;
    rf_frec_msg.data = action[18]*freq_scale;
    std_msgs::Float64 rm_frec_msg;
    rm_frec_msg.data = action[19]*freq_scale;
    std_msgs::Float64 rr_frec_msg;
    rr_frec_msg.data = action[20]*freq_scale;
    std_msgs::Float64 lf_frec_msg;
    lf_frec_msg.data = action[21]*freq_scale;
    std_msgs::Float64 lm_frec_msg;
    lm_frec_msg.data = action[22]*freq_scale;
    std_msgs::Float64 lr_frec_msg;
    lr_frec_msg.data = action[23]*freq_scale;
    // Publish delta R and frecuency
    pub_delta_r_rf.publish(rf_delta_r_msg);
    pub_freq_rf.publish(rf_frec_msg);
    pub_delta_r_rm.publish(rm_delta_r_msg);
    pub_freq_rm.publish(rm_frec_msg);
    pub_delta_r_rr.publish(rr_delta_r_msg);
    pub_freq_rr.publish(rr_frec_msg);
    pub_delta_r_lf.publish(lf_delta_r_msg);
    pub_freq_lf.publish(lf_frec_msg);
    pub_delta_r_lm.publish(lm_delta_r_msg);
    pub_freq_lm.publish(lm_frec_msg);
    pub_delta_r_lr.publish(lr_delta_r_msg);
    pub_freq_lr.publish(lr_frec_msg);
    
    if(action[24] > 0.0){
        // Set up a random number generator with a uniform distribution
        std::random_device rd;  // Random device to seed the generator
        std::mt19937 generator(rd()); // Mersenne Twister engine with a 32-bit state size
        std::uniform_real_distribution<double> distr(0, 2*PI); // Uniform distribution between 0 and 2pi

        std_msgs::Float64 phi0_msg;
        phi0_msg.data = distr(generator);
        pub_phi0_rf.publish(phi0_msg);
        phi0_msg.data = distr(generator);
        pub_phi0_rm.publish(phi0_msg);
        phi0_msg.data = distr(generator);
        pub_phi0_rr.publish(phi0_msg);
        phi0_msg.data = distr(generator);
        pub_phi0_lf.publish(phi0_msg);
        phi0_msg.data = distr(generator);
        pub_phi0_lm.publish(phi0_msg);
        phi0_msg.data = distr(generator);
        pub_phi0_lr.publish(phi0_msg);
    }
    // state(t) 
    ros::Duration(0.01).sleep(); // joints states published at 100Hz (0.01s)
    ros::spinOnce();
    // state(t + 1) update the robot state, now update the observation
    rate.sleep();
    //ros::Duration elapsed_time = ros::Time::now() - start_time;
    //pause physics:
    pauseClient.call(pause_req);
    // log time step:
    //LOG_F(INFO,"Time step: %.6f s", elapsed_time.toSec());
    //Prepare the observation state(t+1):
    std::shared_ptr<std::vector<float>> obs = getObsvector();
    return obs;
}

std::shared_ptr<std::vector<float>> PhantomxEnv::getObsvector(void)
{
    //return the observation:
    std::shared_ptr<std::vector<float>> obs = std::make_shared<std::vector<float>>();
    obs->push_back((float)q.x());   // 0
    obs->push_back((float)q.y());   // 1
    obs->push_back((float)q.z());   // 2
    obs->push_back((float)q.w());   // 3
    //  base velocity
    obs->push_back(velocity_x);      // 4 
    obs->push_back(velocity_y);      // 5
    obs->push_back(velocity_z);      // 6
    // angular rate:
    obs->push_back(angular_rate_x);  // 7
    obs->push_back(angular_rate_y);  // 8
    obs->push_back(angular_rate_z);  // 9
    /* Position and velocities of the joints: */
    // Tibias
    obs->push_back(tibia_positions["rf"]); obs->push_back(tibia_velocities["rf"]);  // 10, 11
    obs->push_back(tibia_positions["rm"]); obs->push_back(tibia_velocities["rm"]);  // 12, 13
    obs->push_back(tibia_positions["rr"]); obs->push_back(tibia_velocities["rr"]);  // 14, 15
    obs->push_back(tibia_positions["lf"]); obs->push_back(tibia_velocities["lf"]);  // 16, 17
    obs->push_back(tibia_positions["lm"]); obs->push_back(tibia_velocities["lm"]);  // 18, 19
    obs->push_back(tibia_positions["lr"]); obs->push_back(tibia_velocities["lr"]);  // 20, 21
    // Thigh
    obs->push_back(thigh_positions["rf"]); obs->push_back(thigh_velocities["rf"]);  // 22, 23
    obs->push_back(thigh_positions["rm"]); obs->push_back(thigh_velocities["rm"]);  // 24, 25
    obs->push_back(thigh_positions["rr"]); obs->push_back(thigh_velocities["rr"]);  // 26, 27
    obs->push_back(thigh_positions["lf"]); obs->push_back(thigh_velocities["lf"]);  // 28, 29
    obs->push_back(thigh_positions["lm"]); obs->push_back(thigh_velocities["lm"]);  // 30, 31
    obs->push_back(thigh_positions["lr"]); obs->push_back(thigh_velocities["lr"]);  // 32, 33
    // Coxas
    obs->push_back(c1_positions["rf"]); obs->push_back(c1_velocities["rf"]);        // 34, 35
    obs->push_back(c1_positions["rm"]); obs->push_back(c1_velocities["rm"]);        // 36, 37
    obs->push_back(c1_positions["rr"]); obs->push_back(c1_velocities["rr"]);        // 38, 39
    obs->push_back(c1_positions["lf"]); obs->push_back(c1_velocities["lf"]);        // 40, 41
    obs->push_back(c1_positions["lm"]); obs->push_back(c1_velocities["lm"]);        // 42, 43
    obs->push_back(c1_positions["lr"]); obs->push_back(c1_velocities["lr"]);        // 44, 45
    // FTG phase
    obs->push_back(std::sin(phase_rf[1])); obs->push_back(std::cos(phase_rf[1]));   // 46, 47
    obs->push_back(std::sin(phase_rm[1])); obs->push_back(std::cos(phase_rm[1]));   // 48, 49
    obs->push_back(std::sin(phase_rr[1])); obs->push_back(std::cos(phase_rr[1]));   // 50, 51
    obs->push_back(std::sin(phase_lf[1])); obs->push_back(std::cos(phase_lf[1]));   // 52, 53
    obs->push_back(std::sin(phase_lm[1])); obs->push_back(std::cos(phase_lm[1]));   // 54, 55
    obs->push_back(std::sin(phase_lr[1])); obs->push_back(std::cos(phase_lr[1]));   // 56, 57
    // Frecuency (derivates)
    obs->push_back(phase_rf[1] - phase_rf[0]);  // 58
    obs->push_back(phase_rm[1] - phase_rm[0]);  // 59
    obs->push_back(phase_rr[1] - phase_rr[0]);  // 60
    obs->push_back(phase_lf[1] - phase_lf[0]);  // 61
    obs->push_back(phase_lm[1] - phase_lm[0]);  // 62
    obs->push_back(phase_lr[1] - phase_lr[0]);  // 63
    // Joint position error history: 
    // tibias
    obs->push_back(q_data_rf_tibia_error[0]); obs->push_back(q_data_rf_tibia_error[1]);  // t-2, t-1  // 64, 65   
    obs->push_back(q_data_rm_tibia_error[0]); obs->push_back(q_data_rm_tibia_error[1]);  // t-2, t-1  // 66, 67  
    obs->push_back(q_data_rr_tibia_error[0]); obs->push_back(q_data_rr_tibia_error[1]);  // t-2, t-1  // 68, 69  
    obs->push_back(q_data_lf_tibia_error[0]); obs->push_back(q_data_lf_tibia_error[1]);  // t-2, t-1  // 70, 71  
    obs->push_back(q_data_lm_tibia_error[0]); obs->push_back(q_data_lm_tibia_error[1]);  // t-2, t-1  // 72, 73
    obs->push_back(q_data_lr_tibia_error[0]); obs->push_back(q_data_lr_tibia_error[1]);  // t-2, t-1  // 74, 75
    // thighs
    obs->push_back(q_data_rf_thigh_error[0]); obs->push_back(q_data_rf_thigh_error[1]);  // t-2, t-1  // 76, 77 
    obs->push_back(q_data_rm_thigh_error[0]); obs->push_back(q_data_rm_thigh_error[1]);  // t-2, t-1  // 78, 79
    obs->push_back(q_data_rr_thigh_error[0]); obs->push_back(q_data_rr_thigh_error[1]);  // t-2, t-1  // 80, 81
    obs->push_back(q_data_lf_thigh_error[0]); obs->push_back(q_data_lf_thigh_error[1]);  // t-2, t-1  // 82, 83
    obs->push_back(q_data_lm_thigh_error[0]); obs->push_back(q_data_lm_thigh_error[1]);  // t-2, t-1  // 84, 85
    obs->push_back(q_data_lr_thigh_error[0]); obs->push_back(q_data_lr_thigh_error[1]);  // t-2, t-1  // 86, 87
    // c1
    obs->push_back(q_data_rf_c1_error[0]); obs->push_back(q_data_rf_c1_error[1]);        // t-2, t-1        // 88, 89
    obs->push_back(q_data_rm_c1_error[0]); obs->push_back(q_data_rm_c1_error[1]);        // t-2, t-1        // 90, 91
    obs->push_back(q_data_rr_c1_error[0]); obs->push_back(q_data_rr_c1_error[1]);        // t-2, t-1        // 92, 93
    obs->push_back(q_data_lf_c1_error[0]); obs->push_back(q_data_lf_c1_error[1]);        // t-2, t-1        // 94, 95
    obs->push_back(q_data_lm_c1_error[0]); obs->push_back(q_data_lm_c1_error[1]);        // t-2, t-1        // 96, 97
    obs->push_back(q_data_lr_c1_error[0]); obs->push_back(q_data_lr_c1_error[1]);        // t-2, t-1        // 98, 99
    /* Joint velocity history */ 
    // tibas
    obs->push_back(q_data_rf_tibia_velocity[0]); obs->push_back(q_data_rf_tibia_velocity[1]);   // t-2, t-1  // 100, 101
    obs->push_back(q_data_rm_tibia_velocity[0]); obs->push_back(q_data_rm_tibia_velocity[1]);   // t-2, t-1  // 102, 103
    obs->push_back(q_data_rr_tibia_velocity[0]); obs->push_back(q_data_rr_tibia_velocity[1]);   // t-2, t-1  // 104, 105
    obs->push_back(q_data_lf_tibia_velocity[0]); obs->push_back(q_data_lf_tibia_velocity[1]);   // t-2, t-1  // 106, 107
    obs->push_back(q_data_lm_tibia_velocity[0]); obs->push_back(q_data_lm_tibia_velocity[1]);   // t-2, t-1  // 108, 109
    obs->push_back(q_data_lr_tibia_velocity[0]); obs->push_back(q_data_lr_tibia_velocity[1]);   // t-2, t-1  // 110, 111
    // thighs
    obs->push_back(q_data_rf_thigh_velocity[0]); obs->push_back(q_data_rf_thigh_velocity[1]);  // t-2, t-1  // 112, 113 
    obs->push_back(q_data_rm_thigh_velocity[0]); obs->push_back(q_data_rm_thigh_velocity[1]);  // t-2, t-1  // 114, 115
    obs->push_back(q_data_rr_thigh_velocity[0]); obs->push_back(q_data_rr_thigh_velocity[1]);  // t-2, t-1  // 116, 117
    obs->push_back(q_data_lf_thigh_velocity[0]); obs->push_back(q_data_lf_thigh_velocity[1]);  // t-2, t-1  // 118, 119
    obs->push_back(q_data_lm_thigh_velocity[0]); obs->push_back(q_data_lm_thigh_velocity[1]);  // t-2, t-1  // 120, 121
    obs->push_back(q_data_lr_thigh_velocity[0]); obs->push_back(q_data_lr_thigh_velocity[1]);  // t-2, t-1  // 122, 123
    // c1
    obs->push_back(q_data_rf_c1_velocity[0]); obs->push_back(q_data_rf_c1_velocity[1]);  // t-2, t-1  // 124, 125 
    obs->push_back(q_data_rm_c1_velocity[0]); obs->push_back(q_data_rm_c1_velocity[1]);  // t-2, t-1  // 126, 127
    obs->push_back(q_data_rr_c1_velocity[0]); obs->push_back(q_data_rr_c1_velocity[1]);  // t-2, t-1  // 128, 129
    obs->push_back(q_data_lf_c1_velocity[0]); obs->push_back(q_data_lf_c1_velocity[1]);  // t-2, t-1  // 130, 131
    obs->push_back(q_data_lm_c1_velocity[0]); obs->push_back(q_data_lm_c1_velocity[1]);  // t-2, t-1  // 132, 133
    obs->push_back(q_data_lr_c1_velocity[0]); obs->push_back(q_data_lr_c1_velocity[1]);  // t-2, t-1  // 134, 135
    // R targets
    // Right Front:
    obs->push_back(r_rf_targets[0][0]); obs->push_back(r_rf_targets[0][1]); obs->push_back(r_rf_targets[0][2]); // t-2  // 136, 137, 138
    obs->push_back(r_rf_targets[1][0]); obs->push_back(r_rf_targets[1][1]); obs->push_back(r_rf_targets[1][2]); // t-1  // 139, 140, 141
    // Right Middle:
    obs->push_back(r_rm_targets[0][0]); obs->push_back(r_rm_targets[0][1]); obs->push_back(r_rm_targets[0][2]); // t-2  // 142, 143, 144
    obs->push_back(r_rm_targets[1][0]); obs->push_back(r_rm_targets[1][1]); obs->push_back(r_rm_targets[1][2]); // t-1  // 145, 146, 147
    // Right Rear:
    obs->push_back(r_rr_targets[0][0]); obs->push_back(r_rr_targets[0][1]); obs->push_back(r_rr_targets[0][2]); // t-2  // 148, 149, 150
    obs->push_back(r_rr_targets[1][0]); obs->push_back(r_rr_targets[1][1]); obs->push_back(r_rr_targets[1][2]); // t-1  // 151, 152, 153
    // Left Front:
    obs->push_back(r_lf_targets[0][0]); obs->push_back(r_lf_targets[0][1]); obs->push_back(r_lf_targets[0][2]); // t-2  // 154, 155, 156
    obs->push_back(r_lf_targets[1][0]); obs->push_back(r_lf_targets[1][1]); obs->push_back(r_lf_targets[1][2]); // t-1  // 157, 158, 159
    // Left Middle:
    obs->push_back(r_lm_targets[0][0]); obs->push_back(r_lm_targets[0][1]); obs->push_back(r_lm_targets[0][2]); // t-2  // 160, 161, 162
    obs->push_back(r_lm_targets[1][0]); obs->push_back(r_lm_targets[1][1]); obs->push_back(r_lm_targets[1][2]); // t-1  // 163, 164, 165
    // Left Rear:
    obs->push_back(r_lr_targets[0][0]); obs->push_back(r_lr_targets[0][1]); obs->push_back(r_lr_targets[0][2]); // t-2  // 166, 167, 168
    obs->push_back(r_lr_targets[1][0]); obs->push_back(r_lr_targets[1][1]); obs->push_back(r_lr_targets[1][2]); // t-1  // 169, 170, 171
    // Privilage information:
    // Terrain normal vector
    assert(terreain_normal_rf.size() == 3);
    assert(terreain_normal_rm.size() == 3);
    assert(terreain_normal_rr.size() == 3);
    assert(terreain_normal_lf.size() == 3);
    assert(terreain_normal_lm.size() == 3);
    assert(terreain_normal_lr.size() == 3);
    obs->push_back(terreain_normal_rf[0]);obs->push_back(terreain_normal_rf[1]);obs->push_back(terreain_normal_rf[2]);  // 172, 173, 174
    obs->push_back(terreain_normal_rm[0]);obs->push_back(terreain_normal_rm[1]);obs->push_back(terreain_normal_rm[2]);  // 175, 176, 177
    obs->push_back(terreain_normal_rr[0]);obs->push_back(terreain_normal_rr[1]);obs->push_back(terreain_normal_rr[2]);  // 178, 179, 180
    obs->push_back(terreain_normal_lf[0]);obs->push_back(terreain_normal_lf[1]);obs->push_back(terreain_normal_lf[2]);  // 181, 182, 183
    obs->push_back(terreain_normal_lm[0]);obs->push_back(terreain_normal_lm[1]);obs->push_back(terreain_normal_lm[2]);  // 184, 185, 186
    obs->push_back(terreain_normal_lr[0]);obs->push_back(terreain_normal_lr[1]);obs->push_back(terreain_normal_lr[2]);  // 187, 188, 189
    // Height scan around each foot
    assert(heightmap_rf.size() == 9);
    assert(heightmap_rm.size() == 9);
    assert(heightmap_rr.size() == 9);
    assert(heightmap_lf.size() == 9);
    assert(heightmap_lm.size() == 9);
    assert(heightmap_lr.size() == 9);
    //
    bool all_zeros_rf = false;
    bool all_zeros_rm = false;
    bool all_zeros_rr = false;
    bool all_zeros_lf = false;
    bool all_zeros_lm = false;
    bool all_zeros_lr = false;

    for(int i=0; i<heightmap_rf.size(); i++) 
    {
        obs->push_back(heightmap_rf[i]); // 190, 191, 192, 193, 194, 195, 196, 197, 198
        obs->push_back(heightmap_rm[i]); // 199, 200, 201, 202, 203, 204, 205, 206, 207
        obs->push_back(heightmap_rr[i]); // 208, 209, 210, 211, 212, 213, 214, 215, 216
        obs->push_back(heightmap_lf[i]); // 217, 218, 219, 220, 221, 222, 223, 224, 225
        obs->push_back(heightmap_lm[i]); // 226, 227, 228, 229, 230, 231, 232, 233, 234
        obs->push_back(heightmap_lr[i]); // 235, 236, 237, 238, 239, 240, 241, 242, 243
        all_zeros_rf |= (heightmap_rf[i] == 0.0);
        all_zeros_rm |= (heightmap_rm[i] == 0.0);
        all_zeros_rr |= (heightmap_rr[i] == 0.0);
        all_zeros_lf |= (heightmap_lf[i] == 0.0);
        all_zeros_lm |= (heightmap_lm[i] == 0.0);
        all_zeros_lr |= (heightmap_lr[i] == 0.0);
    }
    // if all heightmaps are zero, log
    if (all_zeros_rf || all_zeros_rm || all_zeros_rr || all_zeros_lf || all_zeros_lm || all_zeros_lr)
    {
        LOG_F(WARNING, "All the heightmaps are zero");
    }
    // Foot contact forces
    // Right Front:
    obs->push_back(contact_force_rf);  // 244
    obs->push_back(contact_force_rm);  // 245
    obs->push_back(contact_force_rr);  // 246
    obs->push_back(contact_force_lf);  // 247
    obs->push_back(contact_force_lm);  // 248 
    obs->push_back(contact_force_lr);  // 249
     // foot contact states
    obs->push_back(contact_state_rf);  // 250
    obs->push_back(contact_state_rm);  // 251
    obs->push_back(contact_state_rr);  // 252
    obs->push_back(contact_state_lf);  // 253
    obs->push_back(contact_state_lm);  // 254
    obs->push_back(contact_state_lr);  // 255
    // if all contact states are zero, log
    if ((contact_state_rf == 0.0) &&
        (contact_state_rm == 0.0) && 
        (contact_state_rr == 0.0) && 
        (contact_state_lf == 0.0) && 
        (contact_state_lm == 0.0) && 
        (contact_state_lr == 0.0))
    {
        LOG_F(WARNING, "All the contact states are zero");
    }
    // thigh contact states
    obs->push_back(thigh_contact_state_rf);  // 256
    obs->push_back(thigh_contact_state_rm);  // 257
    obs->push_back(thigh_contact_state_rr);  // 258
    obs->push_back(thigh_contact_state_lf);  // 259
    obs->push_back(thigh_contact_state_lm);  // 260
    obs->push_back(thigh_contact_state_lr);  // 261
    // shank contact states
    obs->push_back(shank_contact_state_rf);  // 262
    obs->push_back(shank_contact_state_rm);  // 263
    obs->push_back(shank_contact_state_rr);  // 264
    obs->push_back(shank_contact_state_lf);  // 265
    obs->push_back(shank_contact_state_lm);  // 266
    obs->push_back(shank_contact_state_lr);  // 267
    // Target Foots
    obs->push_back(r_rf_targets[2][0]); obs->push_back(r_rf_targets[2][1]); obs->push_back(r_rf_targets[2][2]); // 268, 269, 270
    obs->push_back(r_rm_targets[2][0]); obs->push_back(r_rm_targets[2][1]); obs->push_back(r_rm_targets[2][2]); // 271, 272, 273 
    obs->push_back(r_rr_targets[2][0]); obs->push_back(r_rr_targets[2][1]); obs->push_back(r_rr_targets[2][2]); // 274, 275, 276
    obs->push_back(r_lf_targets[2][0]); obs->push_back(r_lf_targets[2][1]); obs->push_back(r_lf_targets[2][2]); // 277, 278, 279
    obs->push_back(r_lm_targets[2][0]); obs->push_back(r_lm_targets[2][1]); obs->push_back(r_lm_targets[2][2]); // 280, 281, 282
    obs->push_back(r_lr_targets[2][0]); obs->push_back(r_lr_targets[2][1]); obs->push_back(r_lr_targets[2][2]); // 283, 284, 285
    // base frequency
    obs->push_back(base_frec);    // 286
    obs->push_back(phase_rf[1]);  // 287
    obs->push_back(phase_rm[1]);  // 288
    obs->push_back(phase_rr[1]);  // 289
    obs->push_back(phase_lf[1]);  // 290
    obs->push_back(phase_lm[1]);  // 291
    obs->push_back(phase_lr[1]);  // 292

    obs->push_back(tibia_efforts["rf"]); // 293
    obs->push_back(tibia_efforts["rm"]); // 294
    obs->push_back(tibia_efforts["rr"]); // 295
    obs->push_back(tibia_efforts["lf"]); // 296
    obs->push_back(tibia_efforts["lm"]); // 297
    obs->push_back(tibia_efforts["lr"]); // 298
    obs->push_back(thigh_efforts["rf"]); // 299
    obs->push_back(thigh_efforts["rm"]); // 300
    obs->push_back(thigh_efforts["rr"]); // 301
    obs->push_back(thigh_efforts["lf"]); // 302
    obs->push_back(thigh_efforts["lm"]); // 303
    obs->push_back(thigh_efforts["lr"]); // 304
    obs->push_back(c1_efforts["rf"]); //305
    obs->push_back(c1_efforts["rm"]); //306
    obs->push_back(c1_efforts["rr"]); //307
    obs->push_back(c1_efforts["lf"]); //308
    obs->push_back(c1_efforts["lm"]); //309
    obs->push_back(c1_efforts["lr"]); //310
    assert(obs->size() == 311);

    return obs;
}   

/* PHASE LEFT CALLBACKS */
//Left front:
void PhantomxEnv::phase_lf_callback(const std_msgs::Float64::ConstPtr& msg)
{
    phase_lf.push(msg->data);
}
//Left middle:
void PhantomxEnv::phase_lm_callback(const std_msgs::Float64::ConstPtr& msg)
{
    phase_lm.push(msg->data);
}
//Left rear:
void PhantomxEnv::phase_lr_callback(const std_msgs::Float64::ConstPtr& msg)
{
    phase_lr.push(msg->data);
}
/* PHASE RIGHT CALLBACKS */
//Right front:
void PhantomxEnv::phase_rf_callback(const std_msgs::Float64::ConstPtr& msg)
{
    phase_rf.push(msg->data);
}
//Right middle:
void PhantomxEnv::phase_rm_callback(const std_msgs::Float64::ConstPtr& msg)
{
    phase_rm.push(msg->data);
}
//Right rear:
void PhantomxEnv::phase_rr_callback(const std_msgs::Float64::ConstPtr& msg)
{
    phase_rr.push(msg->data);
}   

/* R CALLBACKS */
//Right front:
void PhantomxEnv::r_rf_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    std::vector<float> r_rf;
    r_rf.clear();
    for (int i=0; i<3; i++)
        r_rf.push_back((msg->data)[i]);

    assert(r_rf.size() == 3);
    r_rf_targets.push(r_rf);
}
//Right middle:
void PhantomxEnv::r_rm_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    std::vector<float> r_rm;
    r_rm.clear();
    for (int i=0; i<3; i++)
        r_rm.push_back((msg->data)[i]);

    assert(r_rm.size() == 3);
    r_rm_targets.push(r_rm);
}
//Right rear:
void PhantomxEnv::r_rr_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    std::vector<float> r_rr;
    r_rr.clear();
    for (int i=0; i<3; i++)
        r_rr.push_back((msg->data)[i]);
    
    assert(r_rr.size() == 3);
    r_rr_targets.push(r_rr);
}
//Left front:
void PhantomxEnv::r_lf_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    std::vector<float> r_lf;
    r_lf.clear();
    for (int i=0; i<3; i++)
        r_lf.push_back((msg->data)[i]);
    assert(r_lf.size() == 3);
    r_lf_targets.push(r_lf);
}
//Left middle:
void PhantomxEnv::r_lm_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    std::vector<float> r_lm;
    r_lm.clear();
    for (int i=0; i<3; i++)
        r_lm.push_back((msg->data)[i]);
    
    assert(r_lm.size() == 3);
    r_lm_targets.push(r_lm);
}
//Left rear:
void PhantomxEnv::r_lr_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    std::vector<float> r_lr;
    r_lr.clear();
    for (int i=0; i<3; i++)
        r_lr.push_back((msg->data)[i]);
    assert(r_lr.size() == 3);
    r_lr_targets.push(r_lr);
}

/* HEIGHTMAPS CALLBACKS */
//Left front:
void PhantomxEnv::heightmap_lf_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    heightmap_lf.clear();
    for (int i=0; i<9; i++)
        heightmap_lf.push_back((msg->data)[i]);
    assert(heightmap_lf.size() == 9);

    std::vector<float> r_lf;
    r_lf.clear();
    r_lf.push_back((msg->data)[9]);  // x
    r_lf.push_back((msg->data)[10]); // y
    r_lf.push_back((msg->data)[11]); // z
    assert(r_lf.size() == 3);
    r_lf_targets.push(r_lf);
}
//Left middle:
void PhantomxEnv::heightmap_lm_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    heightmap_lm.clear();
    for (int i=0; i<9; i++)
        heightmap_lm.push_back((float)(msg->data)[i]);
    assert(heightmap_lm.size() == 9);

    std::vector<float> r_lm;
    r_lm.clear();
    r_lm.push_back((float)(msg->data)[9]);  // x
    r_lm.push_back((float)(msg->data)[10]); // y
    r_lm.push_back((float)(msg->data)[11]); // z
    assert(r_lm.size() == 3);
    r_lm_targets.push(r_lm);

}
//Left rear:
void PhantomxEnv::heightmap_lr_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    heightmap_lr.clear();
    for (int i=0; i<9; i++)
        heightmap_lr.push_back((float)(msg->data)[i]);
    assert(heightmap_lr.size() == 9);

    std::vector<float> r_lr;
    r_lr.clear();
    r_lr.push_back((float)(msg->data)[9]);  // x
    r_lr.push_back((float)(msg->data)[10]); // y
    r_lr.push_back((float)(msg->data)[11]); // z
    assert(r_lr.size() == 3);
    r_lr_targets.push(r_lr);
}
//Right front:
void PhantomxEnv::heightmap_rf_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    heightmap_rf.clear();
    for (int i=0; i<9; i++)
        heightmap_rf.push_back((float)(msg->data)[i]);
    assert(heightmap_rf.size() == 9);

    std::vector<float> r_rf;
    r_rf.clear();
    r_rf.push_back((float)(msg->data)[9]);  // x
    r_rf.push_back((float)(msg->data)[10]); // y
    r_rf.push_back((float)(msg->data)[11]); // z
    assert(r_rf.size() == 3);
    r_rf_targets.push(r_rf);
}
//Right middle:
void PhantomxEnv::heightmap_rm_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    heightmap_rm.clear();
    for (int i=0; i<9; i++)
        heightmap_rm.push_back((float)(msg->data)[i]);
    assert(heightmap_rm.size() == 9);

    std::vector<float> r_rm;
    r_rm.clear();
    r_rm.push_back((float)(msg->data)[9]);  // x
    r_rm.push_back((float)(msg->data)[10]); // y
    r_rm.push_back((float)(msg->data)[11]); // z
    assert(r_rm.size() == 3);
    r_rm_targets.push(r_rm);
}
//Right rear:
void PhantomxEnv::heightmap_rr_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    heightmap_rr.clear();
    for (int i=0; i<9; i++)
        heightmap_rr.push_back((float)(msg->data)[i]);
    assert(heightmap_rr.size() == 9);

    std::vector<float> r_rr;
    r_rr.clear();
    r_rr.push_back((float)(msg->data)[9]);  // x
    r_rr.push_back((float)(msg->data)[10]); // y
    r_rr.push_back((float)(msg->data)[11]); // z
    assert(r_rr.size() == 3);
    r_rr_targets.push(r_rr);
}
//destructor
PhantomxEnv::~PhantomxEnv()
{
    // Destructor
}
