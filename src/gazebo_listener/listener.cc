#include <iostream>
#include <assert.h> 
#include <chrono>
#include <sstream>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <geometry_msgs/Vector3.h>
#include <gazebo/transport/transport.hh>

#define RATE 200

ros::Publisher contacts_pub_lm;
ros::Publisher contacts_pub_lf;
ros::Publisher contacts_pub_lr;
ros::Publisher contacts_pub_rm;
ros::Publisher contacts_pub_rf;
ros::Publisher contacts_pub_rr;

/*ros::Publisher contacts_pub_shank_lm;
ros::Publisher contacts_pub_shank_lf;
ros::Publisher contacts_pub_shank_lr;
ros::Publisher contacts_pub_shank_rm;
ros::Publisher contacts_pub_shank_rf;
ros::Publisher contacts_pub_shank_rr;*/

ros::Publisher contacts_pub_thigh_lm;
ros::Publisher contacts_pub_thigh_lf;
ros::Publisher contacts_pub_thigh_lr;
ros::Publisher contacts_pub_thigh_rm;
ros::Publisher contacts_pub_thigh_rf;
ros::Publisher contacts_pub_thigh_rr;

unsigned int contacts_foot[6];
unsigned int contacts_shank[6];
unsigned int contacts_thigh[6];

float rf_force_foot[3];
float rr_force_foot[3];
float rm_force_foot[3];
float lr_force_foot[3];
float lm_force_foot[3];
float lf_force_foot[3];

using namespace std;

std::vector<std::string> parseString(const std::string& input, const char delimiter) {
    std::vector<std::string> result;
    std::istringstream stream(input);
    std::string token;

    while (std::getline(stream, token, delimiter)) {
        result.push_back(token);
    }

    return result;
}

std::string strip(const std::string& input) {
    size_t start = input.find_first_not_of(" \t\n\r");
    size_t end = input.find_last_not_of(" \t\n\r");

    if (start != std::string::npos && end != std::string::npos) {
        return input.substr(start, end - start + 1);
    }

    return "";  // Empty string if input is all whitespace
}
// 0 rf
// 1 rm
// 2 rr
// 3 lf
// 4 lm
// 5 lr
void cb(ConstContactsPtr& contacts)
{
	for(unsigned int i = 0; i < 6; i++){
		contacts_foot[i]  = 0;
		contacts_thigh[i] = 0;
		contacts_shank[i] = 0;
	}

	for(unsigned int i = 0; i< 3; i++)
	{
		rf_force_foot[i] = 0.0;
		rr_force_foot[i] = 0.0;
		rm_force_foot[i] = 0.0;
		lr_force_foot[i] = 0.0;
		lm_force_foot[i] = 0.0;
		lf_force_foot[i] = 0.0;
	}
	
	//std::cout << "Contactos: " << contacts->contact_size() << "\t";
	for (int i = 0; i < contacts->contact_size(); i++) 
	{	
		const gazebo::msgs::Contact& contact = contacts->contact(i);
		const std::string& col = contact.collision1();
		
		std::vector<std::string> res = parseString(col, ':');
		std::string input = strip(res[4]);
		
		// Foots
		if (input=="tibia_rf_collision_1")
		{
			contacts_foot[0] = 1;
			rf_force_foot[0] = contact.wrench(0).body_1_wrench().force().x();
			rf_force_foot[1] = contact.wrench(0).body_1_wrench().force().y();
			rf_force_foot[2] = contact.wrench(0).body_1_wrench().force().z();
		}
		else if (input=="tibia_rm_collision_1")
		{
			contacts_foot[1] = 1;
			rm_force_foot[0] = contact.wrench(0).body_1_wrench().force().x();
			rm_force_foot[1] = contact.wrench(0).body_1_wrench().force().y();
			rm_force_foot[2] = contact.wrench(0).body_1_wrench().force().z();
		}
		else if (input=="tibia_rr_collision_1")
		{
			contacts_foot[2] = 1;
			rr_force_foot[0] = contact.wrench(0).body_1_wrench().force().x();
			rr_force_foot[1] = contact.wrench(0).body_1_wrench().force().y();
			rr_force_foot[2] = contact.wrench(0).body_1_wrench().force().z();
		}
		else if (input=="tibia_lf_collision_1")
		{
			contacts_foot[3] = 1;
			lf_force_foot[0] = contact.wrench(0).body_1_wrench().force().x();
			lf_force_foot[1] = contact.wrench(0).body_1_wrench().force().y();
			lf_force_foot[2] = contact.wrench(0).body_1_wrench().force().z();
		}
		else if (input=="tibia_lm_collision_1")
		{	
			contacts_foot[4] = 1;
			lm_force_foot[0] = contact.wrench(0).body_1_wrench().force().x();
			lm_force_foot[1] = contact.wrench(0).body_1_wrench().force().y();
			lm_force_foot[2] = contact.wrench(0).body_1_wrench().force().z();
		}
		else if (input=="tibia_lr_collision_1")
		{	
			contacts_foot[5] = 1;
			lr_force_foot[0] = contact.wrench(0).body_1_wrench().force().x();
			lr_force_foot[1] = contact.wrench(0).body_1_wrench().force().y();
			lr_force_foot[2] = contact.wrench(0).body_1_wrench().force().z();
		}

		//thighs
		if (input=="tibia_rf_collision")
		{
			contacts_thigh[0] = 1;
		}
		else if (input=="tibia_rm_collision")
		{
			contacts_thigh[1] = 1;
		}
		else if (input=="tibia_rr_collision")
		{
			contacts_thigh[2] = 1;
		}
		else if (input=="tibia_lf_collision")
		{
			contacts_thigh[3] = 1;
		}
		else if (input=="tibia_lm_collision")
		{	
			contacts_thigh[4] = 1;
		}
		else if (input=="tibia_lr_collision")
		{	
			contacts_thigh[5] = 1;
		}
	}
	// publish foot's forces
	if(contacts->contact_size() > 0)
	{
		//std::cout << std::endl;
		/* RF */
		if (contacts_foot[0] == 1) {
			geometry_msgs::Vector3 pubMsg; 
			pubMsg.x = rf_force_foot[0];
			pubMsg.y = rf_force_foot[1];
			pubMsg.z = rf_force_foot[2];
			contacts_pub_rf.publish(pubMsg);
		}
		else
		{
			geometry_msgs::Vector3 pubMsg; 
			pubMsg.x = 0.0;
			pubMsg.y = 0.0;
			pubMsg.z = 0.0;
			contacts_pub_rf.publish(pubMsg);
		}
		/* RM */
		if (contacts_foot[1] == 1) {
			geometry_msgs::Vector3 pubMsg; 
			pubMsg.x = rm_force_foot[0];
			pubMsg.y = rm_force_foot[1];
			pubMsg.z = rm_force_foot[2];
			contacts_pub_rm.publish(pubMsg);
		}
		else
		{
			geometry_msgs::Vector3 pubMsg; 
			pubMsg.x = 0.0;
			pubMsg.y = 0.0;
			pubMsg.z = 0.0;
			contacts_pub_rm.publish(pubMsg);
		}
		/* RR */
		if (contacts_foot[2] == 1) {
			geometry_msgs::Vector3 pubMsg; 
			pubMsg.x = rr_force_foot[0];
			pubMsg.y = rr_force_foot[1];
			pubMsg.z = rr_force_foot[2];
			contacts_pub_rr.publish(pubMsg);
		}
		else
		{
			geometry_msgs::Vector3 pubMsg; 
			pubMsg.x = 0.0;
			pubMsg.y = 0.0;
			pubMsg.z = 0.0;
			contacts_pub_rr.publish(pubMsg);
		}
		/* LF */
		if (contacts_foot[3] == 1) {
			geometry_msgs::Vector3 pubMsg; 
			pubMsg.x = lf_force_foot[0];
			pubMsg.y = lf_force_foot[1];
			pubMsg.z = lf_force_foot[2];
			contacts_pub_lf.publish(pubMsg);
		} 
		else
		{
			geometry_msgs::Vector3 pubMsg; 
			pubMsg.x = 0.0;
			pubMsg.y = 0.0;
			pubMsg.z = 0.0;
			contacts_pub_lf.publish(pubMsg);
		}
		/* LM */
		if (contacts_foot[4] == 1) {
			geometry_msgs::Vector3 pubMsg; 
			pubMsg.x = lm_force_foot[0];
			pubMsg.y = lm_force_foot[1];
			pubMsg.z = lm_force_foot[2];
			contacts_pub_lm.publish(pubMsg);
		} 
		else
		{
			geometry_msgs::Vector3 pubMsg; 
			pubMsg.x = 0.0;
			pubMsg.y = 0.0;
			pubMsg.z = 0.0;
			contacts_pub_lm.publish(pubMsg);
		}
		/* LR */
		if (contacts_foot[5] == 1) {
			geometry_msgs::Vector3 pubMsg; 
			pubMsg.x = lr_force_foot[0];
			pubMsg.y = lr_force_foot[1];
			pubMsg.z = lr_force_foot[2];
			contacts_pub_lr.publish(pubMsg);
		}
		else
		{
			geometry_msgs::Vector3 pubMsg;
			pubMsg.x = 0.0;
			pubMsg.y = 0.0;
			pubMsg.z = 0.0;
			contacts_pub_lr.publish(pubMsg);
		}  
	} 
	// publish thighs's contacts
	if(contacts->contact_size() > 0)
	{
		if (contacts_thigh[0] == 1) {

			std_msgs::Float64 pubMsg;
  			pubMsg.data = 1.0;
			contacts_pub_thigh_rf.publish(pubMsg);
		}
		else
		{
			std_msgs::Float64 pubMsg;
  			pubMsg.data = 0.0;
			contacts_pub_thigh_rf.publish(pubMsg);
		}

		if(contacts_thigh[1] == 1){

			std_msgs::Float64 pubMsg;
  			pubMsg.data = 1.0;
			contacts_pub_thigh_rm.publish(pubMsg);

		}
		else
		{
			std_msgs::Float64 pubMsg;
  			pubMsg.data = 0.0;
			contacts_pub_thigh_rm.publish(pubMsg);
		}

		if(contacts_thigh[2] == 1){
			std_msgs::Float64 pubMsg;
  			pubMsg.data = 1.0;
			contacts_pub_thigh_rr.publish(pubMsg);
		}
		else
		{
			std_msgs::Float64 pubMsg;
  			pubMsg.data = 0.0;
			contacts_pub_thigh_rr.publish(pubMsg);
		}

		if(contacts_thigh[3] == 1){
			std_msgs::Float64 pubMsg;
  			pubMsg.data = 1.0;
			contacts_pub_thigh_lf.publish(pubMsg);
		}
		else
		{
			std_msgs::Float64 pubMsg;
  			pubMsg.data = 0.0;
			contacts_pub_thigh_lf.publish(pubMsg);
		}

		if(contacts_thigh[4] == 1){
			std_msgs::Float64 pubMsg;
  			pubMsg.data = 1.0;
			contacts_pub_thigh_lm.publish(pubMsg);
		}
		else
		{
			std_msgs::Float64 pubMsg;
  			pubMsg.data = 0.0;
			contacts_pub_thigh_lm.publish(pubMsg);
		}

		if(contacts_thigh[5] == 1){
			std_msgs::Float64 pubMsg;
  			pubMsg.data = 1.0;
			contacts_pub_thigh_lr.publish(pubMsg);
		}
		else
		{
			std_msgs::Float64 pubMsg;
  			pubMsg.data = 0.0;
			contacts_pub_thigh_lr.publish(pubMsg);
		}
	}
}

int main(int _argc, char **_argv)
{
  ROS_INFO("Starting the listener node...");
  ros::init(_argc, _argv, "gazebo_listener"); 
  ros::NodeHandle n;
  ros::Rate loop_rate(RATE);
  // Shank contacs
  /*contacts_pub_shank_rr = n.advertise<std_msgs::Float64>("physics/contacts_forces_shank_rr", 1);
  contacts_pub_shank_rf = n.advertise<std_msgs::Float64>("physics/contacts_forces_shank_rf", 1);
  contacts_pub_shank_rm = n.advertise<std_msgs::Float64>("physics/contacts_forces_shank_rm", 1);
  contacts_pub_shank_lr = n.advertise<std_msgs::Float64>("physics/contacts_forces_shank_lr", 1);
  contacts_pub_shank_lf = n.advertise<std_msgs::Float64>("physics/contacts_forces_shank_lf", 1);
  contacts_pub_shank_lm = n.advertise<std_msgs::Float64>("physics/contacts_forces_shank_lm", 1);*/
  // Thigh contacs
  contacts_pub_thigh_rf = n.advertise<std_msgs::Float64>("physics/contacts_forces_thigh_rf", 1);
  contacts_pub_thigh_rm = n.advertise<std_msgs::Float64>("physics/contacts_forces_thigh_rm", 1);
  contacts_pub_thigh_rr = n.advertise<std_msgs::Float64>("physics/contacts_forces_thigh_rr", 1);
  contacts_pub_thigh_lr = n.advertise<std_msgs::Float64>("physics/contacts_forces_thigh_lr", 1);
  contacts_pub_thigh_lf = n.advertise<std_msgs::Float64>("physics/contacts_forces_thigh_lf", 1);
  contacts_pub_thigh_lm = n.advertise<std_msgs::Float64>("physics/contacts_forces_thigh_lm", 1);
  // Foot Forces contacts publishers
  contacts_pub_rf = n.advertise<geometry_msgs::Vector3>("physics/contacts_forces_rf", 1);
  contacts_pub_rr = n.advertise<geometry_msgs::Vector3>("physics/contacts_forces_rr", 1);
  contacts_pub_rm = n.advertise<geometry_msgs::Vector3>("physics/contacts_forces_rm", 1);
  contacts_pub_lr = n.advertise<geometry_msgs::Vector3>("physics/contacts_forces_lr", 1);
  contacts_pub_lm = n.advertise<geometry_msgs::Vector3>("physics/contacts_forces_lm", 1);
  contacts_pub_lf = n.advertise<geometry_msgs::Vector3>("physics/contacts_forces_lf", 1);

  gazebo::client::setup(_argc, _argv);
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  // Conecto al topic de gazebo physics/contacts
  gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/physics/contacts", cb);

  while (true) gazebo::common::Time::MSleep(10);

  gazebo::client::shutdown();

}
