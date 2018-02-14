// Acado
#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include <acado/function/function.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/user_interaction.hpp>

// ROS
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
/**
* An example of an optimal control problem for
* a quadrotor-arm system without obstacles.
*
* The resulting trajectory is simulated in RViz
*
* The resulting controls and states are stored in 
* files given by states_file and controls_file.
*
*/
USING_NAMESPACE_ACADO

int main(int argc, char** argv) {
  ros::init(argc, argv, "quad_arm_ocp");
  ros::NodeHandle nh;
  ros::Time t1 = ros::Time::now();
  // Time step
  double h; nh.getParam("dt", h);
  /**
  *
  * States and Dynamics
  *
  * State x0,...,x3, y0,...,y3, z0,...,z3,
  * ga0, ga1, q1, q2
  * which is quadrotor flat outputs upto
  * 3rd derivative, yaw, yaw rate and 2 joint
  * angles
  *
  * Control x4, y4, z4, ga2, qd1, qd2
  * which is 4th derivative of flat outputs,
  * derivative of yaw rate and joint velocities
  *
  */
  DifferentialState x0, x1, x2, x3,
                    y0, y1, y2, y3,
                    z0, z1, z2, z3,
                    ga0, ga1,
                    q1, q2;

  Control x4, y4, z4, ga2, qd1, qd2;
  DiscretizedDifferentialEquation  f(h);

  f << next(x0) == x0 + h*x1;
  f << next(y0) == y0 + h*y1;
  f << next(z0) == z0 + h*z1;

  f << next(x1) == x1 + h*x2;
  f << next(y1) == y1 + h*y2;
  f << next(z1) == z1 + h*z2;

  f << next(x2) == x2 + h*x3;
  f << next(y2) == y2 + h*y3;
  f << next(z2) == z2 + h*z3;

  f << next(x3) == x3 + h*x4;
  f << next(y3) == y3 + h*y4;
  f << next(z3) == z3 + h*z4;

  f << next(ga0) == ga0 + h*ga1;
  f << next(ga1) == ga1 + h*ga2;

  f << next(q1) == q1 + h*qd1;
  f << next(q2) == q2 + h*qd2;

  std::cout <<"Added Dynamics" << std::endl;
  /**
  * Cost along trajectory
  *
  * Cost along trajectory is defined on higher 
  * derivatives of flat outputs, yaw and joint
  * velocities, weighted by matrix Q
  */
  std::string qFile;
  nh.getParam("qFile", qFile);
  DMatrix Q(4, 4); Q.read(qFile.c_str());
  DVector offset(4); offset.setAll(0.0);
  Function eta;
  eta << x4 << y4 << z4
      << ga2;
  /**
  * Terminal Cost
  */
  std::string wFile;
  nh.getParam("wFile", wFile);
  DMatrix W(3,3); W.read(wFile.c_str());
  // W.setIdentity();

  // Hardcode goal pose for example
  double gx, gy, gz, l1, l2;
  nh.getParam("goal_x", gx);
  nh.getParam("goal_y", gy);
  nh.getParam("goal_z", gz);
  DVector goal(3);
  goal(0) = gx; goal(1) = gy; goal(2) = gz;

  nh.getParam("link1", l1);
  nh.getParam("link2", l2);
  Function phi;
  phi << x0;
  phi << y0;
  phi << z0;

  /**
  * Set up optimal control problem
  *
  * (Using fixed final time)
  */
  double ts = 0.0;
  double te; nh.getParam("te", te);
  int numSteps = (int) (te/h);
  OCP ocp(ts, te, numSteps);

  ocp.minimizeLSQ(0.01*Q, eta, offset); // Trajectory Cost
  std::cout <<"Added trajectory cost" << std::endl;

  ocp.minimizeLSQEndTerm(W, phi, goal); // Terminal Cost

  ocp.subjectTo(f); // Dynamics

  ocp.subjectTo(0.0 >= q1 >= -1.57); // joint limits
  ocp.subjectTo(0.0 <= q2 <= 1.57);

  ocp.subjectTo(-0.78 <= qd1 <= 0.78); // joint velocity limits
  ocp.subjectTo(-0.78 <= qd2 <= 0.78);

  double start_x, start_y, start_z, start_q1, start_q2,
        ox, oy, oz,
        ox2, oy2, oz2, ora;
  nh.getParam("quad_x0", start_x);
  nh.getParam("quad_y0", start_y);
  nh.getParam("quad_z0", start_z);
  nh.getParam("arm_q1", start_q1);
  nh.getParam("arm_q2", start_q2);
  nh.getParam("obs_x", ox);
  nh.getParam("obs_y", oy);
  nh.getParam("obs_z", oz);
  nh.getParam("obs_x2", ox2);
  nh.getParam("obs_y2", oy2);
  nh.getParam("obs_z2", oz2);
  nh.getParam("obs_r", ora);

  ocp.subjectTo(AT_START, x0 == start_x);
  ocp.subjectTo(AT_START, y0 == start_y);
  ocp.subjectTo(AT_START, z0 == start_z);
  ocp.subjectTo(AT_START, ga0 == 0.0);
  ocp.subjectTo(AT_START, q1 == start_q1);
  ocp.subjectTo(AT_START, q2 == start_q2);

  ocp.subjectTo(AT_START, x1 == 0.0);
  ocp.subjectTo(AT_START, y1 == 0.0);
  ocp.subjectTo(AT_START, z1 == 0.0);
  ocp.subjectTo(AT_START, ga1 == 0.0);

  ocp.subjectTo(AT_END, x1 == 0.0);
  ocp.subjectTo(AT_END, y1 == 0.0);
  ocp.subjectTo(AT_END, z1 == 0.0);
  ocp.subjectTo(AT_END, ga1 == 0.0);
  ocp.subjectTo(AT_END, qd1 == 0.0);
  ocp.subjectTo(AT_END, qd2 == 0.0);

  ocp.subjectTo(((x0-ox)*(x0-ox) + (y0-oy)*(y0-oy) + (z0-oz)*(z0-oz)) >= ora*ora);
  ocp.subjectTo(((x0-ox2)*(x0-ox2) + (y0-oy2)*(y0-oy2) + (z0-oz2)*(z0-oz2)) >= ora*ora);
  std::cout <<"Added ocp" << std::endl;
  /**
  * Set up optimization parameters
  */
  OptimizationAlgorithm algorithm(ocp);

  // algorithm.set( INTEGRATOR_TYPE, INT_DISCRETE);
  // algorithm.set(DISCRETIZATION_TYPE, COLLOCATION);
  algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
  // algorithm.set( LEVENBERG_MARQUARDT, 1e-10 );
  algorithm.set(KKT_TOLERANCE, 5e-2);

  std::cout <<"Added optimization scheme" << std::endl;
  /**
  * Solve
  */
  // GnuplotWindow window;
  // window.addSubplot(x1 ,"Velcoity x");
  // window.addSubplot(q1 ,"joint 1");
  // window.addSubplot(q2 ,"joint 2");
  // window.addSubplot(y1 ,"Velcoity y");
  // window.addSubplot(z1 ,"Velcoity z");
  // algorithm << window;

  algorithm.solve();
  ROS_INFO("OCP took %f seconds to solve", (ros::Time::now()-t1).toSec());

  /**
  * Get final results
  */
  std::string states_file, controls_file;
  nh.getParam("states_file", states_file);
  nh.getParam("controls_file", controls_file);
  algorithm.getDifferentialStates(states_file.c_str());
  algorithm.getControls(controls_file.c_str());

  VariablesGrid states;
  algorithm.getDifferentialStates(states);

  /**
  * Visuaize trajectory in Rviz
  */
  ros::Duration(1.0).sleep();
  tf::TransformBroadcaster br;
  ros::Publisher jointPub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
  ros::Publisher goalPub = nh.advertise<visualization_msgs::Marker>("/goal_marker", 2);
  ros::Publisher obsPub = nh.advertise<visualization_msgs::Marker>("/obs_marker", 2);
  ros::Publisher obsPub2 = nh.advertise<visualization_msgs::Marker>("/obs_marker2", 2);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = goal(0);
  marker.pose.position.y = goal(1);
  marker.pose.position.z = goal(2) - 0.0875;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; 
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  visualization_msgs::Marker marker2;
  marker2.header.frame_id = "world";
  marker2.id = 1;
  marker2.type = visualization_msgs::Marker::SPHERE;
  marker2.action = visualization_msgs::Marker::ADD;
  marker2.pose.position.x = ox;
  marker2.pose.position.y = oy;
  marker2.pose.position.z = oz;
  marker2.pose.orientation.x = 0.0;
  marker2.pose.orientation.y = 0.0;
  marker2.pose.orientation.z = 0.0;
  marker2.pose.orientation.w = 1.0;
  marker2.scale.x = 0.1;
  marker2.scale.y = 0.1;
  marker2.scale.z = 0.1;
  marker2.color.a = 1.0; 
  marker2.color.r = 0.0;
  marker2.color.g = 1.0;
  marker2.color.b = 0.0;

  visualization_msgs::Marker marker3;
  marker3.header.frame_id = "world";
  marker3.id = 1;
  marker3.type = visualization_msgs::Marker::SPHERE;
  marker3.action = visualization_msgs::Marker::ADD;
  marker3.pose.position.x = ox2;
  marker3.pose.position.y = oy2;
  marker3.pose.position.z = oz2;
  marker3.pose.orientation.x = 0.0;
  marker3.pose.orientation.y = 0.0;
  marker3.pose.orientation.z = 0.0;
  marker3.pose.orientation.w = 1.0;
  marker3.scale.x = 0.1;
  marker3.scale.y = 0.1;
  marker3.scale.z = 0.1;
  marker3.color.a = 1.0; 
  marker3.color.r = 0.0;
  marker3.color.g = 1.0;
  marker3.color.b = 0.0;

  for(int tt=0; tt < numSteps+1; tt++) {
    tf::Transform transform = tf::Transform(
     tf::createQuaternionFromRPY(0,0,states(tt, 12)), 
     tf::Vector3(states(tt, 0), states(tt, 4), states(tt, 8)));

    sensor_msgs::JointState joint_state;
    joint_state.name.push_back("airbasetolink1");
    joint_state.name.push_back("link1tolink2");

    joint_state.position.push_back(-states(tt, 14) + 1.57);
    joint_state.position.push_back(-states(tt, 15));
    joint_state.header.stamp = ros::Time::now();

    jointPub.publish(joint_state);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "baselink"));
    marker.header.stamp = ros::Time::now();
    goalPub.publish(marker);
    obsPub.publish(marker2);
    obsPub2.publish(marker3);
    ros::Duration(te/numSteps).sleep();
  }
  return 0;
}
