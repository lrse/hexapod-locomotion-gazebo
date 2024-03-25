#include <ros/ros.h>
#include "gait_planning.h"

using namespace std;
using namespace ros;

double yaw, roll, pitch;    //angles in rad
// Parameters for the motion generation 
int main(int argc, char* argv[])
{	
	init(argc, argv, "phantomx_controller"); // ROS node inicialization
	NodeHandle node_handle; // ROS node handle
	GaitPlanning gait(node_handle);
	gait.startWalking();
	return 0;
}