#ifndef JOINT_STATES_H
#define JOINT_STATES_H

#include <queue>
#include <deque>
#include <vector>
#include <ros/ros.h>
#include <control_msgs/JointControllerState.h>
#include <sensor_msgs/JointState.h>


template <typename T, int MaxLen, typename Container=std::deque<T>>
class FixedQueue : public std::queue<T, Container> {
public:
    void push(const T& value) {
        if (this->size() == MaxLen) {
           this->c.pop_front();
        }
        std::queue<T, Container>::push(value);
    }

    T& operator[](size_t index) {
        if (index < this->size()) {
            // Convert index to iterator and access element
            auto it = this->c.begin();
            std::advance(it, index);
            return *it;
        }
        throw std::out_of_range("Index out of range.");
    }
};

class JointStates
{

protected:

    unsigned int NHistory;
    unsigned int NSides;
    unsigned int JointPerLeg;
    //positions:
    std::map<std::string, float> tibia_positions;
    std::map<std::string, float> thigh_positions;
    std::map<std::string, float> c1_positions;
    //velocities:
    std::map<std::string, float> tibia_velocities;
    std::map<std::string, float> thigh_velocities;
    std::map<std::string, float> c1_velocities;
    //efforts:
    std::map<std::string, float> tibia_efforts;
    std::map<std::string, float> thigh_efforts;
    std::map<std::string, float> c1_efforts;

    // Robot joints states error with the set points:
    // Right Front:
    float q_data_rf_tibia_err;
    float q_data_rf_thigh_err;
    float q_data_rf_c1_err;
    FixedQueue<float, 3> q_data_rf_tibia_error;
    FixedQueue<float, 3> q_data_rf_thigh_error;
    FixedQueue<float, 3> q_data_rf_c1_error;
    // Right Middle:
    float q_data_rm_tibia_err;
    float q_data_rm_thigh_err;
    float q_data_rm_c1_err;
    FixedQueue<float, 3> q_data_rm_tibia_error;
    FixedQueue<float, 3> q_data_rm_thigh_error;
    FixedQueue<float, 3> q_data_rm_c1_error;
    // Right Rear:
    float q_data_rr_tibia_err;
    float q_data_rr_thigh_err;
    float q_data_rr_c1_err;
    FixedQueue<float, 3> q_data_rr_tibia_error;
    FixedQueue<float, 3> q_data_rr_thigh_error;
    FixedQueue<float, 3> q_data_rr_c1_error;
    // Left Front:
    float q_data_lf_tibia_err;
    float q_data_lf_thigh_err;
    float q_data_lf_c1_err;
    FixedQueue<float, 3> q_data_lf_tibia_error;
    FixedQueue<float, 3> q_data_lf_thigh_error;
    FixedQueue<float, 3> q_data_lf_c1_error;
     // Left Middle:
    float q_data_lm_tibia_err;
    float q_data_lm_thigh_err;
    float q_data_lm_c1_err;
    FixedQueue<float, 3> q_data_lm_tibia_error;
    FixedQueue<float, 3> q_data_lm_thigh_error;
    FixedQueue<float, 3> q_data_lm_c1_error;
    // Left Rear:
    float q_data_lr_tibia_err;
    float q_data_lr_thigh_err;
    float q_data_lr_c1_err;
    FixedQueue<float, 3> q_data_lr_tibia_error;
    FixedQueue<float, 3> q_data_lr_thigh_error;
    FixedQueue<float, 3> q_data_lr_c1_error;

    // Robot joints states velocity
    // Right Front:
    FixedQueue<float, 3> q_data_rf_tibia_velocity;
    FixedQueue<float, 3> q_data_rf_thigh_velocity;
    FixedQueue<float, 3> q_data_rf_c1_velocity;
    // Right Middle:
    FixedQueue<float, 3> q_data_rm_tibia_velocity;
    FixedQueue<float, 3> q_data_rm_thigh_velocity;
    FixedQueue<float, 3> q_data_rm_c1_velocity;
    // Right Rear:
    FixedQueue<float, 3> q_data_rr_tibia_velocity;
    FixedQueue<float, 3> q_data_rr_thigh_velocity;
    FixedQueue<float, 3> q_data_rr_c1_velocity;
    // Left Front:
    FixedQueue<float, 3> q_data_lf_tibia_velocity;
    FixedQueue<float, 3> q_data_lf_thigh_velocity;
    FixedQueue<float, 3> q_data_lf_c1_velocity;
    // Left Middle:
    FixedQueue<float, 3> q_data_lm_tibia_velocity;
    FixedQueue<float, 3> q_data_lm_thigh_velocity;
    FixedQueue<float, 3> q_data_lm_c1_velocity;
    // Left Rear:
    FixedQueue<float, 3> q_data_lr_tibia_velocity;
    FixedQueue<float, 3> q_data_lr_thigh_velocity;
    FixedQueue<float, 3> q_data_lr_c1_velocity;

    ros::Rate rate;
    ros::Subscriber sub_states;
    //Right Front:
    ros::Subscriber sub_rf_tibia;
    ros::Subscriber sub_rf_thigh;
    ros::Subscriber sub_rf_c1;

    ros::Subscriber sub_rm_tibia;
    ros::Subscriber sub_rm_thigh;
    ros::Subscriber sub_rm_c1;

    ros::Subscriber sub_rr_tibia;
    ros::Subscriber sub_rr_thigh;
    ros::Subscriber sub_rr_c1;

    ros::Subscriber sub_lf_tibia;
    ros::Subscriber sub_lf_thigh;
    ros::Subscriber sub_lf_c1;

    ros::Subscriber sub_lm_tibia;
    ros::Subscriber sub_lm_thigh;
    ros::Subscriber sub_lm_c1;

    ros::Subscriber sub_lr_tibia;
    ros::Subscriber sub_lr_thigh;
    ros::Subscriber sub_lr_c1;

public:
    JointStates(ros::NodeHandle& nh, ros::Rate& r);
    ~JointStates();
    void states_callback(const sensor_msgs::JointState::ConstPtr& msg);
    //Right Front callbacks:
    void rf_c1_callback(const control_msgs::JointControllerState::ConstPtr& msg);
    void rf_thigh_callback(const control_msgs::JointControllerState::ConstPtr& msg);
    void rf_tibia_callback(const control_msgs::JointControllerState::ConstPtr& msg);
    //Right Middle callbacks:
    void rm_c1_callback(const control_msgs::JointControllerState::ConstPtr& msg);
    void rm_thigh_callback(const control_msgs::JointControllerState::ConstPtr& msg);
    void rm_tibia_callback(const control_msgs::JointControllerState::ConstPtr& msg);
    //Right Rear callbacks:
    void rr_c1_callback(const control_msgs::JointControllerState::ConstPtr& msg);
    void rr_thigh_callback(const control_msgs::JointControllerState::ConstPtr& msg);
    void rr_tibia_callback(const control_msgs::JointControllerState::ConstPtr& msg);
    //Left Front callbacks:
    void lf_c1_callback(const control_msgs::JointControllerState::ConstPtr& msg);
    void lf_thigh_callback(const control_msgs::JointControllerState::ConstPtr& msg);
    void lf_tibia_callback(const control_msgs::JointControllerState::ConstPtr& msg);
    //Left Middle callbacks:
    void lm_c1_callback(const control_msgs::JointControllerState::ConstPtr& msg);
    void lm_thigh_callback(const control_msgs::JointControllerState::ConstPtr& msg);
    void lm_tibia_callback(const control_msgs::JointControllerState::ConstPtr& msg);
    //Left Rear callbacks:
    void lr_c1_callback(const control_msgs::JointControllerState::ConstPtr& msg);
    void lr_thigh_callback(const control_msgs::JointControllerState::ConstPtr& msg);
    void lr_tibia_callback(const control_msgs::JointControllerState::ConstPtr& msg);
};

#endif // ENV_H