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
  ros::Time ros_ts = ros::Time::now();
  // Time step
  double h; nh.getParam("dt", h);
  /**
  *
  * States and Dynamics
  *
  * State x,y,z, (Position) 
  *       r,p, ga (Rotation)
  *       vx, vy, vz (Velocity)
  *       wx, wy, wz (Angular Velocity)
  *       xa1, ya1, za1 (Arm link 1 pos)
  *       xa2, ya2, za2 (Arm link 2 pos)
  *
  * Control t1, t2, t3 (body torques)
  *         th (thrust)
  *         qd1, qd2 (joint velocities)
  */
  DifferentialState x, y, z,
                    r, p, ga,
                    vx, vy, vz,
                    wx, wy, wz,
                    xa1, ya1, za1,
                    xa2, ya2, za2;

  Control t1, t2, t3, th, qd1, qd2;
  DiscretizedDifferentialEquation  f(h);

  // Quad Pos
  f << next(x) == x + h*vx;
  f << next(y) == y + h*vy;
  f << next(z) == z + h*vz;

  // Quad Vel
  f << next(vx) == vx + th*(sin(r)*sin(ga) + cos(r)*cos(ga)*sin(p))*h;
  f << next(vy) == vy - th*(cos(ga)*sin(r) - cos(r)*sin(p)*sin(ga))*h;
  f << next(vz) == vz + th*cos(p)*cos(r)*h;

  // Quad Angular vel (Assuming J = I)
  f << next(wx) == wx + t1*h;
  f << next(wy) == wy + t2*h;
  f << next(wz) == wz + t3*h;

  double l1, l2;
  nh.getParam("link1", l1);
  nh.getParam("link2", l2);

  // Arm link 1
  f << next(xa1) == x + xa1*cos(h*qd1) - za1*sin(h*qd1)*cos(ga);
  f << next(ya1) == y + ya1*cos(h*qd1) - za1*sin(h*qd1)*sin(ga);
  f << next(za1) == z + za1*cos(h*qd1) + sqrt(xa1*xa1 + ya1*ya1)*sin(qd1*h);

  // Arm link 2
  f << next(xa2) == x + xa1*cos(h*qd1) - za1*sin(h*qd1)*cos(ga) + 
                    (xa2 - xa1)*cos((qd1 + qd2)*h) - (za2 - za1)*sin((qd1+qd2)*h)*cos(ga);
  f << next(ya2) == y + ya1*cos(h*qd1) - za1*sin(h*qd1)*sin(ga) + 
                    (ya2 - ya1)*cos((qd1 + qd2)*h) - (za2 - za1)*sin((qd1+qd2)*h)*sin(ga);
  f << next(za2) == z + za1*cos(h*qd1) + sqrt(xa1*xa1 + ya1*ya1)*sin(qd1*h) +
                    (za2 - za1)*cos((qd1+qd2)*h) +  sqrt(xa1*xa1 + ya1*ya1)*sin((qd1+qd2)*h);

  std::cout <<"Added Dynamics" << std::endl;
  /**
  * Cost along trajectory
  *
  * Cost along trajectory is defined to minimize 
  * control effort. 
  * Weight matrix Q is read from param qFile
  */
  std::string qFile;
  nh.getParam("qFile", qFile);
  DMatrix Q(6,6); Q.read(qFile.c_str());
  DVector offset(6); offset.setAll(0.0);
  Function eta;
  eta << t1 << t2 << t3 << th << qd1 << qd2;
  /**
  * Terminal Cost
  *
  * Terminal Cost is defined to minimize
  * distance of final end-effector pose
  * from goal pose
  *
  * Weight matrix is read from param wFile
  */
  std::string wFile;
  nh.getParam("wFile", wFile);
  DMatrix W(3,3); W.read(wFile.c_str());

  double gx, gy, gz;
  nh.getParam("goal_x", gx);
  nh.getParam("goal_y", gy);
  nh.getParam("goal_z", gz);
  DVector goal(3);
  goal(0) = gx; goal(1) = gy; goal(2) = gz;

  Function phi;
  phi << xa2;
  phi << ya2;
  phi << za2;
  /**
  * Set up optimal control problem
  *
  * (Using fixed final time)
  */
  double ts = 0.0;
  double te; nh.getParam("te", te);
  int numSteps = (int) (te/h);
  OCP ocp(ts, te, numSteps);

  ocp.minimizeLSQ(Q, eta, offset); // Trajectory Cost
  std::cout <<"Added trajectory cost" << std::endl;

  ocp.minimizeLSQEndTerm(W, phi, goal); // Terminal Cost

  ocp.subjectTo(f); // Dynamics

  ocp.subjectTo(-0.78 <= qd1 <= 0.78); // joint velocity limits
  ocp.subjectTo(-0.78 <= qd2 <= 0.78);

  double start_x, start_y, start_z;
  nh.getParam("quad_x0", start_x);
  nh.getParam("quad_y0", start_y);
  nh.getParam("quad_z0", start_z);

  // Starting position
  ocp.subjectTo(AT_START, x == start_x);
  ocp.subjectTo(AT_START, y == start_y);
  ocp.subjectTo(AT_START, z == start_z);
  ocp.subjectTo(AT_START, xa1 == start_x);
  ocp.subjectTo(AT_START, ya1 == start_y);
  ocp.subjectTo(AT_START, za1 == start_z - l1);
  ocp.subjectTo(AT_START, xa1 == start_x + l2);
  ocp.subjectTo(AT_START, ya1 == start_y);
  ocp.subjectTo(AT_START, za1 == start_z - l1);

  // Velocities and RP zero at start and end
  ocp.subjectTo(AT_START, vx == 0.0);
  ocp.subjectTo(AT_START, vy == 0.0);
  ocp.subjectTo(AT_START, vz == 0.0);
  ocp.subjectTo(AT_START, wx == 0.0);
  ocp.subjectTo(AT_START, wy == 0.0);
  ocp.subjectTo(AT_START, wz == 0.0);
  ocp.subjectTo(AT_START, r == 0.0);
  ocp.subjectTo(AT_START, p == 0.0);
  ocp.subjectTo(AT_START, ga == 0.0);
  ocp.subjectTo(AT_START, qd1 == 0.0);
  ocp.subjectTo(AT_START, qd2 == 0.0);

  ocp.subjectTo(AT_END, vx == 0.0);
  ocp.subjectTo(AT_END, vy == 0.0);
  ocp.subjectTo(AT_END, vz == 0.0);
  ocp.subjectTo(AT_END, wx == 0.0);
  ocp.subjectTo(AT_END, wy == 0.0);
  ocp.subjectTo(AT_END, wz == 0.0);
  ocp.subjectTo(AT_END, qd1 == 0.0);
  ocp.subjectTo(AT_END, qd2 == 0.0);

  std::cout <<"Added ocp" << std::endl;
  /**
  * Set up optimization parameters
  */
  OptimizationAlgorithm algorithm(ocp);

  algorithm.set( INTEGRATOR_TYPE, INT_DISCRETE );
  algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
  algorithm.set( LEVENBERG_MARQUARDT, 1e-10 );
  algorithm.set(KKT_TOLERANCE, 1e-3);

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
  ROS_INFO("OCP took %f seconds to solve", (ros::Time::now()-ros_ts).toSec());

  // /**
  // * Get final results
  // */
  // std::string states_file, controls_file;
  // nh.getParam("states_file", states_file);
  // nh.getParam("controls_file", controls_file);
  // algorithm.getDifferentialStates(states_file.c_str());
  // algorithm.getControls(controls_file.c_str());

  // VariablesGrid states;
  // algorithm.getDifferentialStates(states);

  // /**
  // * Visuaize trajectory in Rviz
  // */
  // ros::Duration(1.0).sleep();
  // tf::TransformBroadcaster br;
  // ros::Publisher jointPub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
  // ros::Publisher markerPub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 2);

  // visualization_msgs::Marker marker;
  // marker.header.frame_id = "world";
  // marker.id = 0;
  // marker.type = visualization_msgs::Marker::CUBE;
  // marker.action = visualization_msgs::Marker::ADD;
  // marker.pose.position.x = goal(0);
  // marker.pose.position.y = goal(1);
  // marker.pose.position.z = goal(2) - 0.0875;
  // marker.pose.orientation.x = 0.0;
  // marker.pose.orientation.y = 0.0;
  // marker.pose.orientation.z = 0.0;
  // marker.pose.orientation.w = 1.0;
  // marker.scale.x = 0.1;
  // marker.scale.y = 0.1;
  // marker.scale.z = 0.1;
  // marker.color.a = 1.0; 
  // marker.color.r = 0.0;
  // marker.color.g = 1.0;
  // marker.color.b = 0.0;

  // for(int tt=0; tt < numSteps+1; tt++) {
  //   tf::Transform transform = tf::Transform(
  //    tf::createQuaternionFromRPY(0,0,states(tt, 12)), 
  //    tf::Vector3(states(tt, 0), states(tt, 4), states(tt, 8)));

  //   sensor_msgs::JointState joint_state;
  //   joint_state.name.push_back("airbasetolink1");
  //   joint_state.name.push_back("link1tolink2");

  //   joint_state.position.push_back(-states(tt, 14) + 1.57);
  //   joint_state.position.push_back(-states(tt, 15));
  //   joint_state.header.stamp = ros::Time::now();

  //   jointPub.publish(joint_state);
  //   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "baselink"));
  //   marker.header.stamp = ros::Time::now();
  //   markerPub.publish(marker);
  //   ros::Duration(te/numSteps).sleep();
  // }
  return 0;
}
