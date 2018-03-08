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
                    ga0, ga1,
                    q1, q2;

  IntermediateState eex, eey, eez;

  double l1 = 0.175; double l2 = 0.42;

  Control x4, y4, z4, ga2, qd1, qd2;
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

  f << dot(q1) == qd1;
  f << dot(q2) == qd2;

  // eex = x0 + (l1*cos(q1) + l2*cos(q1+q2))*cos(ga0);
  // eey = y0 + (l1*cos(q1) + l2*cos(q1+q2))*sin(ga0);
  // eez = z0 + l1*sin(q1) + l2*sin(q1+q2);

  // f << 0 == T - sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81));
  // f << 0 == r - asin((-x2*sin(ga0) + y2*cos(ga0))/T);
  // f << 0 == p - atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81));
  eex = x0 + (sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) + cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81))))*(l2*sin(q1 + q2) + l1*sin(q1)) + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(ga0)*(l2*cos(q1 + q2) + l1*cos(q1));
  eey = y0 + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0)*(l2*cos(q1 + q2) + l1*cos(q1)) - (cos(ga0)*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)))) - cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0))*(l2*sin(q1 + q2) + l1*sin(q1));
  eez = z0 + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*(l2*sin(q1 + q2) + l1*sin(q1)) - sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*(l2*cos(q1 + q2) + l1*cos(q1));

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
  DMatrix Q(6, 6); Q.setIdentity();
  Q(0,0) = 0.01; Q(1,1) = 0.01; Q(2,2) = 0.01;
  DVector offset(6); offset.setAll(0.0);
  Function eta;
  eta << x4 << y4 << z4 << ga2 << qd1 << qd2;
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

  ocp.subjectTo(AT_START, x0 == start_x);
  ocp.subjectTo(AT_START, y0 == start_y);
  ocp.subjectTo(AT_START, z0 == start_z);
  ocp.subjectTo(AT_START, ga0 == 0.0);
  ocp.subjectTo(AT_START, q1 == -1.56);
  ocp.subjectTo(AT_START, q2 == 0.0);

  ocp.subjectTo(AT_START, x1 == 0.0);
  ocp.subjectTo(AT_START, y1 == 0.0);
  ocp.subjectTo(AT_START, z1 == 0.0);
  ocp.subjectTo(AT_START, ga1 == 0.0);
  ocp.subjectTo(AT_START, qd1 == 0.0);
  ocp.subjectTo(AT_START, qd2 == 0.0);

  ocp.subjectTo(-1.57 <= q1 <= 0.0); // joint limits
  ocp.subjectTo(-1.57 <= q2 <= 1.57); // joint limits
  ocp.subjectTo(-1.57 <= ga0 <= 1.57); // joint limits

  ocp.subjectTo(-0.78 <= qd1 <= 0.78); // joint velocity limits
  ocp.subjectTo(-0.78 <= qd2 <= 0.78);

  ocp.subjectTo(-1.0 <= x1 <= 1.0);
  ocp.subjectTo(-1.0 <= y1 <= 1.0);
  ocp.subjectTo(-1.0 <= z1 <= 1.0);
  ocp.subjectTo(-1.0 <= ga1 <= 1.0);

  ocp.subjectTo(AT_END, x1 == 0.0);
  ocp.subjectTo(AT_END, y1 == 0.0);
  ocp.subjectTo(AT_END, z1 == 0.0);
  ocp.subjectTo(AT_END, x2 == 0.0);
  ocp.subjectTo(AT_END, y2 == 0.0);
  ocp.subjectTo(AT_END, z2 == 0.0);
  ocp.subjectTo(AT_END, x3 == 0.0);
  ocp.subjectTo(AT_END, y3 == 0.0);
  ocp.subjectTo(AT_END, z3 == 0.0);
  ocp.subjectTo(AT_END, ga1 == 0.0);

  ocp.subjectTo(AT_END, (x0 + (l1*cos(q1) + l2*cos(q1+q2))*cos(ga0)) == gx);
  ocp.subjectTo(AT_END, (y0 + (l1*cos(q1) + l2*cos(q1+q2))*sin(ga0)) == gy);
  ocp.subjectTo(AT_END, (z0 + l1*sin(q1) + l2*sin(q1+q2)) == gz);

  ocp.subjectTo(AT_END, (l1*sin(q1) + l2*sin(q1 + q2)) <= -0.05);
  ocp.subjectTo(AT_END, (l1*cos(q1) + l2*cos(q1 + q2)) >= 0.05);

  ocp.subjectTo(x0 <= gx + 0.1); ocp.subjectTo(z0 <= gz + 0.3);
  /**
  * Set up optimization parameters
  */
  OptimizationAlgorithm algorithm(ocp);

  algorithm.set(MAX_NUM_QP_ITERATIONS, 100);
  algorithm.set(INFEASIBLE_QP_HANDLING, IQH_STOP);
  algorithm.set( INTEGRATOR_TYPE, INT_RK45);
  algorithm.set(DISCRETIZATION_TYPE, COLLOCATION);
  algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON);
  algorithm.set(KKT_TOLERANCE, 1e-3);

  Grid timeGrid(ts, te, numSteps+1);
  VariablesGrid xi(19, timeGrid);
  VariablesGrid ui(6, timeGrid);

  algorithm.initializeDifferentialStates(xi);
  algorithm.initializeControls(ui);

  std::cout <<"Added optimization scheme" << std::endl;
  /**
  * Solve
  */
  ros::Time t1 = ros::Time::now();
  algorithm.solve();
  /**
  * Get final results
  */
  std::string states_file, controls_file;
  nh.getParam("states_file", states_file);
  nh.getParam("controls_file", controls_file);
  algorithm.getDifferentialStates(states_file.c_str());
  algorithm.getControls(controls_file.c_str());

  algorithm.getDifferentialStates(xi);
  algorithm.getControls(ui);

  ocp.subjectTo(((x0-ox)*(x0-ox) + (y0-oy)*(y0-oy) + (z0-oz)*(z0-oz)) >= ora*ora);
  ocp.subjectTo(((x0-ox2)*(x0-ox2) + (y0-oy2)*(y0-oy2) + (z0-oz2)*(z0-oz2)) >= ora*ora);

  ocp.subjectTo(((eex-ox)*(eex-ox) + (eey-oy)*(eey-oy) + (eez-oz)*(eez-oz)) >= ora*ora);
  ocp.subjectTo(((eex-ox2)*(eex-ox2) + (eez-oy2)*(eez-oy2) + (eez-oz2)*(eez-oz2)) >= ora*ora);

  algorithm.initializeDifferentialStates(xi);
  algorithm.initializeControls(ui);

  algorithm.solve();
  ROS_INFO("Final OCP took %f seconds to solve", (ros::Time::now()-t1).toSec());

  VariablesGrid states(19, timeGrid);
  algorithm.getDifferentialStates(states);
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

    double x = states(tt,0); double y = states(tt,1); double z = states(tt,2);
    double xdd = states(tt,6); double ydd = states(tt,7); double zdd = states(tt,8);
    double yaw = states(tt,12); double j1 = states(tt,14); double j2 = states(tt,15);
    double ex = x + (l1*cos(j1) + l2*cos(j1+j2))*cos(yaw);
    double ey = y + (l1*cos(j1) + l2*cos(j1+j2))*sin(yaw);
    double ez = z + l1*sin(j1) + l2*sin(j1+j2);
    double T = sqrt(xdd*xdd + ydd*ydd + (zdd+9.81)*(zdd+9.81));
    double r = asin((-xdd*sin(yaw) + ydd*cos(yaw))/T);
    double p = atan((xdd*cos(yaw) + ydd*sin(yaw))/(zdd + 9.81));

    tf::Transform transform = tf::Transform(
     tf::createQuaternionFromRPY(r,p,states(tt, 12)), 
     tf::Vector3(states(tt, 0), states(tt, 1), states(tt, 2)));

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "baselink"));
    marker.header.stamp = ros::Time::now();
    goalPub.publish(marker);
    obsPub.publish(marker2);
    obsPub2.publish(marker3);

    sensor_msgs::JointState joint_state;
    joint_state.name.push_back("airbasetolink1");
    joint_state.name.push_back("link1tolink2");

    joint_state.position.push_back(-states(tt,14) + 1.57);
    joint_state.position.push_back(-states(tt,15));
    joint_state.header.stamp = ros::Time::now();

    jointPub.publish(joint_state);

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
