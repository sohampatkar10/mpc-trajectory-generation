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
#include <nav_msgs/Odometry.h>
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
  DifferentialState x0, y0, z0, 
                    x1, y1, z1,
                    x2, y2, z2,
                    x3, y3, z3,
                    ga0, ga1;

  Control x4, y4, z4, ga2;
  double ts = 0.0;
  double te = 5.0;
  int numSteps = 50; 
  DifferentialEquation  f(te, ts);

  f << dot(x0) == x1;
  f << dot(y0) == y1;
  f << dot(z0) == z1;

  f << dot(x1) == x2;
  f << dot(y1) == y2;
  f << dot(z1) == z2;

  f << dot(x2) == x3;
  f << dot(y2) == y3;
  f << dot(z2) == z3;

  f << dot(x3) == x4;
  f << dot(y3) == y4;
  f << dot(z3) == z4;

  f << dot(ga0) == ga1;
  f << dot(ga1) == ga2;

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
  eta << x4 << y4 << z4 << ga2;
  /**
  * Terminal Cost
  */
  std::string wFile;
  nh.getParam("wFile", wFile);
  DMatrix W(3,3); W.read(wFile.c_str());

  // Hardcode goal pose for example
  double gx, gy, gz;
  nh.getParam("goal_x", gx);
  nh.getParam("goal_y", gy);
  nh.getParam("goal_z", gz);
  DVector goal(3);
  goal(0) = gx; goal(1) = gy; goal(2) = gz;

  Function phi;
  phi << x0;
  phi << y0;
  phi << z0;
  /**
  * Set up optimal control problem
  *
  * (Using fixed final time)
  */
  OCP ocp(ts, te, numSteps);

  ocp.minimizeLSQ(Q, eta, offset); // Trajectory Cost
  std::cout <<"Added trajectory cost" << std::endl;

  // ocp.minimizeLSQEndTerm(W, phi, goal); // Terminal Cost

  ocp.subjectTo(f); // Dynamics

  double start_x, start_y, start_z,
        ox, oy, oz,
        ox2, oy2, oz2, ora;
  nh.getParam("quad_x0", start_x);
  nh.getParam("quad_y0", start_y);
  nh.getParam("quad_z0", start_z);
  nh.getParam("obs_x", ox);
  nh.getParam("obs_y", oy);
  nh.getParam("obs_z", oz);
  nh.getParam("obs_x2", ox2);
  nh.getParam("obs_y2", oy2);
  nh.getParam("obs_z2", oz2);
  nh.getParam("obs_r", ora);

  ROS_INFO("goal %f %f %f", gx, gy, gz);
  ROS_INFO("obs %f %f %f", ox, oy, oz);

  ocp.subjectTo(AT_START, x0 == start_x);
  ocp.subjectTo(AT_START, y0 == start_y);
  ocp.subjectTo(AT_START, z0 == start_z);
  ocp.subjectTo(AT_START, ga0 == 0.0);

  ocp.subjectTo(AT_START, x1 == 0.0);
  ocp.subjectTo(AT_START, y1 == 0.0);
  ocp.subjectTo(AT_START, z1 == 0.0);
  ocp.subjectTo(AT_START, ga1 == 0.0);

  ocp.subjectTo(AT_END, x1 == 0.0);
  ocp.subjectTo(AT_END, y1 == 0.0);
  ocp.subjectTo(AT_END, z1 == 0.0);
  ocp.subjectTo(AT_END, ga1 == 0.0);

  ocp.subjectTo(AT_END, x0 == gx);
  ocp.subjectTo(AT_END, y0 == gy);
  ocp.subjectTo(AT_END, z0 == gz);

  ocp.subjectTo(((x0-ox)*(x0-ox) + (y0-oy)*(y0-oy) + (z0-oz)*(z0-oz)) >= ora*ora);
  ocp.subjectTo(((x0-ox2)*(x0-ox2) + (y0-oy2)*(y0-oy2) + (z0-oz2)*(z0-oz2)) >= ora*ora);
  std::cout <<"Added ocp" << std::endl;
  /**
  * Set up optimization parameters
  */
  OptimizationAlgorithm algorithm(ocp);

  Grid timeGrid(ts, te, numSteps+1);
  VariablesGrid xi(14, timeGrid);
  VariablesGrid ui(4, timeGrid);

  for(int tt=0; tt < numSteps+1; tt++) {
    double k = (double)tt/double(numSteps);
    xi(tt,0) = k*gx + 0.2*sin(k*3.14); xi(tt,1) = k*gy; xi(tt,2) = k*gz;
    xi(tt,3) = gx/te; xi(tt,4) = gy/te; xi(tt,5) = gz/te;
    xi(tt,6) = 0.0; xi(tt,7) = 0.0; xi(tt,8) = 0.0;
    xi(tt,9) = 0.0; xi(tt,10) = 0.0; xi(tt,11) = 0.0;
    xi(tt,12) = k*0.5; xi(tt,13) = 0.5/te;
  }

  algorithm.initializeDifferentialStates(xi);

  algorithm.set( INTEGRATOR_TYPE, INT_RK45);
  algorithm.set(DISCRETIZATION_TYPE, COLLOCATION);
  algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON);
  algorithm.set(KKT_TOLERANCE, 1e-4);

  std::cout <<"Added optimization scheme" << std::endl;
  /**
  * Solve
  */
  // GnuplotWindow window;
  // window.addSubplot(x1 ,"Velocity x");
  // window.addSubplot(y1 ,"Velocity y");
  // window.addSubplot(z1 ,"Velocity z");
  // algorithm << window;
  ros::Time t1 = ros::Time::now();
  // for(int k=0; k < 5; k++)
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

  VariablesGrid states(14, timeGrid);
  algorithm.getDifferentialStates(states);
  ROS_INFO("No of time points = %i", states.getNumPoints());
  /**
  * Visuaize trajectory in Rviz
  */
  ros::Duration(0.5).sleep();
  tf::TransformBroadcaster br;
  ros::Publisher jointPub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
  ros::Publisher goalPub = nh.advertise<visualization_msgs::Marker>("/goal_marker", 2);
  ros::Publisher obsPub = nh.advertise<visualization_msgs::Marker>("/obs_marker", 2);
  ros::Publisher obsPub2 = nh.advertise<visualization_msgs::Marker>("/obs_marker2", 2);
  ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("/odometry",1);

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
     tf::Vector3(states(tt, 0), states(tt, 1), states(tt, 2)));

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "baselink"));
    marker.header.stamp = ros::Time::now();
    goalPub.publish(marker);
    obsPub.publish(marker2);
    obsPub2.publish(marker3);

    nav_msgs::Odometry odommsg;
    odommsg.header.stamp = ros::Time::now();
    odommsg.header.frame_id = "world";
    odommsg.pose.pose.position.x = states(tt,0);
    odommsg.pose.pose.position.y = states(tt,1);
    odommsg.pose.pose.position.z = states(tt,2);

    odommsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(states(tt,12));

    odommsg.twist.twist.linear.x = states(tt,3);
    odommsg.twist.twist.linear.y = states(tt,4);
    odommsg.twist.twist.linear.z = states(tt,5);

    odomPub.publish(odommsg);

    ros::Duration(te/numSteps).sleep();
  }
  return 0;
}
