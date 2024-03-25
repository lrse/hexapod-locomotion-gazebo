#include <gazebo/gazebo.hh>
#include "gazebo/physics/World.hh"
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Heightmap.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/HeightmapShape.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <gazebo/transport/transport.hh>
#include <std_msgs/Float64MultiArray.h>
#include <gazebo_msgs/LinkStates.h>
#include <map>
#include <vector>


/*rf_foot = -np.array([-0.08827736795607508, 0.0834015636632274, 0.10644186234158473])
rm_foot = -np.array([0.00178225, 0.1272474,  0.09333618])
rr_foot = -np.array([0.10109539, 0.08784227, 0.09125327])

lf_foot = -np.array([-0.0909871,  -0.09317667,  0.08935056])
lm_foot = -np.array([ 0.00181821, -0.1232117,   0.09100944])
lr_foot = -np.array([ 0.08731719, -0.09387418,  0.09359773])*/

float rf_foot[3] = {0.08827736795607508, -0.0834015636632274, -0.10644186234158473};
float rm_foot[3] = {-0.00178225, -0.1272474,  -0.09333618};
float rr_foot[3] = {-0.10109539, -0.08784227, -0.09125327};

float lf_foot[3] = {+0.0909871,  +0.09317667,  -0.08935056};
float lm_foot[3] = {-0.00181821, +0.1232117,   -0.09100944};
float lr_foot[3] = {-0.08731719, +0.09387418,  -0.09359773};


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


namespace gazebo
{
  class HeightmapsPlugin : public ModelPlugin
  {
    public: HeightmapsPlugin() : ModelPlugin()
            {

               if (!ros::isInitialized())
              {
                  int argc = 0;
                  char **argv = NULL;
                  ros::init(argc, argv, "heightmaps");
              }

              // Create a ROS node handle
              rosNode.reset(new ros::NodeHandle("heightmaps_node"));

              heightmaps_pub_rf = rosNode->advertise<std_msgs::Float64MultiArray>("/heightmaps/rf_leg", 1);
              heightmaps_pub_rm = rosNode->advertise<std_msgs::Float64MultiArray>("/heightmaps/rm_leg", 1);
              heightmaps_pub_rr = rosNode->advertise<std_msgs::Float64MultiArray>("/heightmaps/rr_leg", 1);
              heightmaps_pub_lf = rosNode->advertise<std_msgs::Float64MultiArray>("/heightmaps/lf_leg", 1);
              heightmaps_pub_lm = rosNode->advertise<std_msgs::Float64MultiArray>("/heightmaps/lm_leg", 1);
              heightmaps_pub_lr = rosNode->advertise<std_msgs::Float64MultiArray>("/heightmaps/lr_leg", 1);

              sub_positions = rosNode->subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1,  &HeightmapsPlugin::linkStatesCallback, this);
            }

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
            {
              physics::Link_V links = _model->GetLinks();
              physics::LinkPtr link = links[0];
              ROS_INFO("Size of links %ld", links.size());
              physics::Collision_V collisions = link->GetCollisions();
              physics::CollisionPtr collision = collisions[0];
              physics::ShapePtr shape         = collision->GetShape();
                
              hm = dynamic_cast<physics::HeightmapShape*>(shape.get()); 
              std::string name= hm->GetURI();
              ROS_INFO("Map name URI:%s", name.c_str());

              size  = hm->Size();
              vc = hm->VertexCount();
              
              ROS_INFO("Size: %f, %f, %f", size.X(), size.Y(), size.Z());
              ROS_INFO("VertexCount: %d, %d", vc.X(), vc.Y());
              double x,y;
              // get the size of a grid
              ROS_INFO("Max Height: %f", hm->GetMaxHeight());
              ROS_INFO("Min Height: %f", hm->GetMinHeight());

              // Initialize the transport system
              //transport::NodePtr node(new transport::Node());
              //node->Init();

              // Subscribe to the ground truth topic
              //subscriber_ = node->Subscribe("/gazebo/default/pose/info", &HeightmapsPlugin::OnGroundTruthMsg, this);

            }
    
    public: void linkStatesCallback(gazebo_msgs::LinkStates link_states)
    {
      
      linkPositionsIdxDict["tibia_rf"] = 0;
      linkPositionsIdxDict["tibia_rm"] = 0;
      linkPositionsIdxDict["tibia_rr"] = 0;
      linkPositionsIdxDict["tibia_lf"] = 0;
      linkPositionsIdxDict["tibia_lm"] = 0;
      linkPositionsIdxDict["tibia_lr"] = 0;

      for(int i = 0; i < link_states.name.size(); i++)
      {
          std::vector<std::string> res = parseString(link_states.name[i], ':');
		      std::string currentName = strip(res[2]);
          //std::cout << currentName << std::endl;

          if(linkPositionsIdxDict.find(currentName) != linkPositionsIdxDict.end())
            linkPositionsIdxDict[currentName] = i;
      }
      const geometry_msgs::Pose& poseTibiaRf = link_states.pose[linkPositionsIdxDict["tibia_rf"]];
      const geometry_msgs::Pose& poseTibiaRm = link_states.pose[linkPositionsIdxDict["tibia_rm"]];
      const geometry_msgs::Pose& poseTibiaRr = link_states.pose[linkPositionsIdxDict["tibia_rr"]];
      const geometry_msgs::Pose& poseTibiaLf = link_states.pose[linkPositionsIdxDict["tibia_lf"]];
      const geometry_msgs::Pose& poseTibiaLm = link_states.pose[linkPositionsIdxDict["tibia_lm"]];
      const geometry_msgs::Pose& poseTibiaLr = link_states.pose[linkPositionsIdxDict["tibia_lr"]];
      // Access the position and orientation of the model
      const geometry_msgs::Point& positionTibiaRf = poseTibiaRf.position;
      //std::cout << positionTibiaRf.x << " " << positionTibiaRf.y << " " << positionTibiaRf.z << std::endl;
      const geometry_msgs::Point& positionTibiaRm = poseTibiaRm.position;
      const geometry_msgs::Point& positionTibiaRr = poseTibiaRr.position;
      const geometry_msgs::Point& positionTibiaLf = poseTibiaLf.position;
      const geometry_msgs::Point& positionTibiaLm = poseTibiaLm.position;
      const geometry_msgs::Point& positionTibiaLr = poseTibiaLr.position;

      double radio = 0.1;
      double numPoints = 9;

      std_msgs::Float64MultiArray* arrayMsg; 
      arrayMsg = GetHeightInParametrizeCircle(positionTibiaRf.x + rf_foot[0], 
                                              positionTibiaRf.y + rf_foot[1],
                                              positionTibiaRf.z + rf_foot[2],
                                              radio, numPoints);
      heightmaps_pub_rf.publish(*arrayMsg); 
      delete arrayMsg;

      arrayMsg = GetHeightInParametrizeCircle(positionTibiaRm.x + rm_foot[0], 
                                              positionTibiaRm.y + rm_foot[1],
                                              positionTibiaRm.z + rm_foot[2], 
                                              radio, numPoints);
      heightmaps_pub_rm.publish(*arrayMsg);
      delete arrayMsg; 
      
      arrayMsg = GetHeightInParametrizeCircle(positionTibiaRr.x + rr_foot[0], 
                                              positionTibiaRr.y + rr_foot[1],
                                              positionTibiaRr.z + rr_foot[2], 
                                              radio, numPoints);
      heightmaps_pub_rr.publish(*arrayMsg);
      delete arrayMsg;
  
      arrayMsg = GetHeightInParametrizeCircle(positionTibiaLf.x + lf_foot[0], 
                                              positionTibiaLf.y + lf_foot[1],
                                              positionTibiaLf.z + lf_foot[2], 
                                              radio, numPoints);
      heightmaps_pub_lf.publish(*arrayMsg);
      delete arrayMsg; 
  
      arrayMsg = GetHeightInParametrizeCircle(positionTibiaLm.x + lm_foot[0], 
                                              positionTibiaLm.y + lm_foot[1],
                                              positionTibiaLm.z + lm_foot[2], 
                                              radio, numPoints);
      heightmaps_pub_lm.publish(*arrayMsg);
      delete arrayMsg;
  
      arrayMsg = GetHeightInParametrizeCircle(positionTibiaLr.x + lr_foot[0], 
                                              positionTibiaLr.y + lr_foot[1],
                                              positionTibiaLr.z + lr_foot[2], 
                                              radio, numPoints);
      heightmaps_pub_lr.publish(*arrayMsg);
      delete arrayMsg;
     
  }
    /*  
    public: void OnGroundTruthMsg(ConstPosesStampedPtr& msg) 
            {
                for (int i = 0; i < msg->pose_size(); ++i)
                {

                  const msgs::Pose& poseMsg = msg->pose(i);
                  std::string link_name = poseMsg.name();
                  //printf("Link name: %s\n", link_name.c_str());
                  // Access the position and orientation of the model
                  const msgs::Vector3d& position = poseMsg.position();
                  // Convert position and orientation to appropriate types (e.g., ignition::math::Vector3d and ignition::math::Quaterniond)
                 // std::cout << "link name  " << link_name << "x:"<<position.x() << " y:"<<position.y() << std::endl;
                  if (link_name == "phantomx_training::tibia_rr")
                  {
                   std_msgs::Float64MultiArray* arrayMsg; 
                   arrayMsg = GetHeightInParametrizeCircle(position.x(), position.y(), 0.1, 9);

                   //std::cout << "link name  " << link_name << " x: "<<position.x() << " y: "<<position.y() << std::endl;
                   heightmaps_pub_rr.publish(*arrayMsg);
                   delete arrayMsg;
                  }
                  if (link_name == "phantomx_training::tibia_rf")
                  {
                   std_msgs::Float64MultiArray* arrayMsg; 
                   arrayMsg = GetHeightInParametrizeCircle(position.x(), position.y(), 0.1, 9);
                   heightmaps_pub_rf.publish(*arrayMsg); 
                   delete arrayMsg;
                  }
                  if (link_name == "phantomx_training::tibia_rm")
                  {
                   std_msgs::Float64MultiArray* arrayMsg; 
                   arrayMsg = GetHeightInParametrizeCircle(position.x(), position.y(), 0.1, 9);
                   heightmaps_pub_rm.publish(*arrayMsg);
                   delete arrayMsg; 
                  }
                  if (link_name == "phantomx_training::tibia_lr")
                  {
                   std_msgs::Float64MultiArray* arrayMsg; 
                   arrayMsg = GetHeightInParametrizeCircle(position.x(), position.y(), 0.1, 9);
                   heightmaps_pub_lr.publish(*arrayMsg);
                   delete arrayMsg;
                  }
                  if (link_name == "phantomx_training::tibia_lf")
                  {
                   std_msgs::Float64MultiArray* arrayMsg; 
                   arrayMsg = GetHeightInParametrizeCircle(position.x(), position.y(), 0.1, 9);
                   heightmaps_pub_lf.publish(*arrayMsg);
                   delete arrayMsg;
                  }
                  if (link_name == "phantomx_training::tibia_lm")
                  {
                   std_msgs::Float64MultiArray* arrayMsg; 
                   arrayMsg = GetHeightInParametrizeCircle(position.x(), position.y(), 0.1, 9);
                   heightmaps_pub_lm.publish(*arrayMsg);
                   delete arrayMsg;
                  }  

                   
                }
            }
    */

    private: double GetHeight(double x, double y)
             {
                return hm->GetHeight( (x + size.X()/2)/size.X()*vc.X() - 1, (y + size.Y()/2)/size.Y()*vc.Y() - 1);
             }
    
    private: std_msgs::Float64MultiArray* GetHeightInParametrizeCircle(double h, double k, double l, double r, int numPoints) 
              {
                std_msgs::Float64MultiArray* arrayMsg = new std_msgs::Float64MultiArray();  
                for (int i = 0; i < numPoints; ++i) 
                {
                    double theta = 2.0 * M_PI * i / (double)numPoints;
                    double x = h + r * cos(theta);
                    double y = k + r * sin(theta);
                    double z = GetHeight(x, y);
                    arrayMsg->data.push_back(z);
                }
                // save the center point (the position of the tip of the leg)
                arrayMsg->data.push_back(h);
                arrayMsg->data.push_back(k);
                arrayMsg->data.push_back(l);

                return arrayMsg;
              }

    private: transport::SubscriberPtr subscriber_;
    private: physics::HeightmapShape* hm;
    private: ignition::math::Vector3d size;
    private: ignition::math::Vector2i vc;
    private: ros::NodeHandlePtr rosNode;
    private: ros::Publisher rosPublisher;
    private: ros::Publisher heightmaps_pub_rf;
    private: ros::Publisher heightmaps_pub_rm;
    private: ros::Publisher heightmaps_pub_rr;
    private: ros::Publisher heightmaps_pub_lf;
    private: ros::Publisher heightmaps_pub_lm;
    private: ros::Publisher heightmaps_pub_lr;
    private: ros::Subscriber sub_positions;
    private: std::map<std::string, int> linkPositionsIdxDict;
  };

  GZ_REGISTER_MODEL_PLUGIN(HeightmapsPlugin)
}
