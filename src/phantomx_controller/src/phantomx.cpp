#include "phantomx.h"


Phantomx::Phantomx(NodeHandle& node_handle) {


        /* initialize coxa joints and foots */
        coxa_joints   = new Vector3d [NUM_SIDES];
	    foot_tips 	  = new Vector3d [NUM_SIDES];
        node_handle = node_handle;

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

        for(unsigned int i = 0; i < 3*NUM_SIDES; i++)
        {
            Publisher new_publisher = node_handle.advertise<Float64>(command_topics[i],1);
            publishers.push_back(new_publisher);
        }
    }

Phantomx::~Phantomx() {

    delete [] coxa_joints;
	delete [] foot_tips;
    
}