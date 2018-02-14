// ROS
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>

#include <fstream>
#include <sstream>

/**
*
* An example to propogate the flat output dynamics
* from the controls and recover the actual quadrotor
* states
*
* This assumes the OCP has generated a trajectory
* and the states and controls are stored in the files
* provided by the states_file and controls_file params
*
*/
int main(int argc, char**argv) {
  ros::init(argc, argv, "quad_arm_propogation");
  ros::NodeHandle nh;

  std::vector<double> controls;
  std::vector<double> states;
  std::vector<double> times;

  std::string states_file = "./data/states.txt";
  nh.getParam("states_file", states_file);
  std::ifstream statesFile;
  statesFile.open(states_file);
  std::string line;
  while(getline(statesFile, line)) {
    std::stringstream ss(line);
    std::string value;
    getline(ss,value); getline(ss, value);    
    ROS_INFO("Vlue = %s", value[2]);
    times.push_back(std::stod(value));

    while(getline(ss, value)) {
      if(value != "[" && value != "]");
      states.push_back(std::stod(value));
    }
  }
  // ROS_INFO("Size = ", states.size());
  return 0;
}