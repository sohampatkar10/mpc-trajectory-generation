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
#include <tf/transform_listener.h>
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

  std::string states_file; nh.getParam("states_file", states_file);
  std::string controls_file; nh.getParam("controls_file", controls_file);

  std::ofstream states_log(states_file);
  std::ofstream controls_log(controls_file);

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

  ros::Time t1 = ros::Time::now();

  DifferentialState x0, y0, z0, 
                    x1, y1, z1,
                    x2, y2, z2,
                    x3, y3, z3,
                    ga0, ga1,
                    q1, q2;

  IntermediateState eex, eey, eez, T, r, p;

  IntermediateState pq1x, pq1y, pq1z;
  IntermediateState pq2x, pq2y, pq2z;
  IntermediateState pq3x, pq3y, pq3z;
  IntermediateState pq4x, pq4y, pq4z;

  IntermediateState p11x, p11y, p11z;
  IntermediateState p12x, p12y, p12z;
  IntermediateState p13x, p13y, p13z;
  IntermediateState p14x, p14y, p14z;

  IntermediateState p21x, p21y, p21z;
  IntermediateState p22x, p22y, p22z;
  IntermediateState p23x, p23y, p23z;
  IntermediateState p24x, p24y, p24z;
  IntermediateState p25x, p25y, p25z;

  double lr; nh.getParam("link_r", lr);

  double l1 = 0.255; double l2 = 0.29 + lr; 
  double qoffx = 0.2; double qoffz = -0.1;

  double l11 = lr; double l12 = 3.0*lr; 
  double l13 = 5.0*lr; double l14 = 7.0*lr;

  double l21 = lr; double l22 = 3.0*lr; 
  double l23 = 5.0*lr; double l24 = 7.0*lr;
  double l25 = 9.0*lr;

  Control x4, y4, z4, ga2;
  Control qd1, qd2;
  // Control qdd1, qdd2;
  // Parameter mq1, mq2, cq1, cq2;
  // IntermediateState qd1, qd2;
  // qd1 = mq1*qdd1 + cq1;
  // qd2 = mq2*qdd2 + cq2;

  double ts = 0.0;
  double te; nh.getParam("te", te);
  int numSteps; nh.getParam("numSteps", numSteps);
  DifferentialEquation  f(ts, te);

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

  T = sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81));
  r = asin((-x2*sin(ga0) + y2*cos(ga0))/T);
  p = atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81));

  double qr;  nh.getParam("quad_r", qr);

  pq1x = x0 + qr*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(ga0) - qr*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) - cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)))));
  pq1y = y0 + qr*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0) + sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0)) + qr*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0);
  pq1z = z0 + qr*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)))) - qr*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)));

  pq2x = x0 + qr*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) - cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))) + qr*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(ga0);
  pq2y = y0 + qr*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0) - qr*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0) + sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0));
  pq2z = z0 - qr*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81))) - qr*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))));

  pq3x = x0 + qr*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) - cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))) - qr*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(ga0);
  pq3y = y0 - qr*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0) + sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0)) - qr*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0);
  pq3z = z0 + qr*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81))) - qr*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))));

  pq4x = x0 - qr*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) - cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))) - qr*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(ga0);
  pq4y = y0 + qr*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0) + sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0)) - qr*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0);
  pq4z = z0 + qr*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81))) + qr*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))));

  eex =  x0 + (sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) + cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81))))*(qoffz + l2*sin(q1 + q2) + l1*sin(q1)) + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(ga0)*(qoffx + l2*cos(q1 + q2) + l1*cos(q1));
  eey =  y0 + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0)*(qoffx + l2*cos(q1 + q2) + l1*cos(q1)) - (cos(ga0)*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)))) - cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0))*(qoffz + l2*sin(q1 + q2) + l1*sin(q1));
  eez =  z0 + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*(qoffz + l2*sin(q1 + q2) + l1*sin(q1)) - sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*(qoffx + l2*cos(q1 + q2) + l1*cos(q1));

  p11x = x0 + (qoffz + l11*sin(q1))*(sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) + cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))) + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(ga0)*(qoffx + l11*cos(q1));
  p11y = y0 + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0)*(qoffx + l11*cos(q1)) - (qoffz + l11*sin(q1))*(cos(ga0)*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)))) - cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0));
  p11z = z0 + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*(qoffz + l11*sin(q1)) - sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*(qoffx + l11*cos(q1));

  p12x = x0 + (qoffz + l12*sin(q1))*(sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) + cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))) + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(ga0)*(qoffx + l12*cos(q1));
  p12y = y0 + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0)*(qoffx + l12*cos(q1)) - (qoffz + l12*sin(q1))*(cos(ga0)*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)))) - cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0));
  p12z = z0 + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*(qoffz + l12*sin(q1)) - sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*(qoffx + l12*cos(q1));

  p13x = x0 + (qoffz + l13*sin(q1))*(sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) + cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))) + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(ga0)*(qoffx + l13*cos(q1));
  p13y = y0 + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0)*(qoffx + l13*cos(q1)) - (qoffz + l13*sin(q1))*(cos(ga0)*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)))) - cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0));
  p13z = z0 + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*(qoffz + l13*sin(q1)) - sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*(qoffx + l13*cos(q1));

  p14x = x0 + (qoffz + l14*sin(q1))*(sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) + cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))) + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(ga0)*(qoffx + l14*cos(q1));
  p14y = y0 + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0)*(qoffx + l14*cos(q1)) - (qoffz + l14*sin(q1))*(cos(ga0)*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)))) - cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0));
  p14z = z0 + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*(qoffz + l14*sin(q1)) - sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*(qoffx + l14*cos(q1));

  p21x =  x0 + (sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) + cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81))))*(qoffz + l21*sin(q1 + q2) + l1*sin(q1)) + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(ga0)*(qoffx + l21*cos(q1 + q2) + l1*cos(q1));
  p21y =  y0 + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0)*(qoffx + l21*cos(q1 + q2) + l1*cos(q1)) - (cos(ga0)*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)))) - cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0))*(qoffz + l21*sin(q1 + q2) + l1*sin(q1));
  p21z =  z0 + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*(qoffz + l21*sin(q1 + q2) + l1*sin(q1)) - sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*(qoffx + l21*cos(q1 + q2) + l1*cos(q1));

  p22x =  x0 + (sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) + cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81))))*(qoffz + l22*sin(q1 + q2) + l1*sin(q1)) + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(ga0)*(qoffx + l22*cos(q1 + q2) + l1*cos(q1));
  p22y =  y0 + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0)*(qoffx + l22*cos(q1 + q2) + l1*cos(q1)) - (cos(ga0)*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)))) - cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0))*(qoffz + l22*sin(q1 + q2) + l1*sin(q1));
  p22z =  z0 + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*(qoffz + l22*sin(q1 + q2) + l1*sin(q1)) - sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*(qoffx + l22*cos(q1 + q2) + l1*cos(q1));

  p23x =  x0 + (sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) + cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81))))*(qoffz + l23*sin(q1 + q2) + l1*sin(q1)) + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(ga0)*(qoffx + l23*cos(q1 + q2) + l1*cos(q1));
  p23y =  y0 + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0)*(qoffx + l23*cos(q1 + q2) + l1*cos(q1)) - (cos(ga0)*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)))) - cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0))*(qoffz + l23*sin(q1 + q2) + l1*sin(q1));
  p23z =  z0 + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*(qoffz + l23*sin(q1 + q2) + l1*sin(q1)) - sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*(qoffx + l23*cos(q1 + q2) + l1*cos(q1));

  p24x =  x0 + (sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) + cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81))))*(qoffz + l24*sin(q1 + q2) + l1*sin(q1)) + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(ga0)*(qoffx + l24*cos(q1 + q2) + l1*cos(q1));
  p24y =  y0 + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0)*(qoffx + l24*cos(q1 + q2) + l1*cos(q1)) - (cos(ga0)*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)))) - cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0))*(qoffz + l24*sin(q1 + q2) + l1*sin(q1));
  p24z =  z0 + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*(qoffz + l24*sin(q1 + q2) + l1*sin(q1)) - sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*(qoffx + l24*cos(q1 + q2) + l1*cos(q1));

  p25x =  x0 + (sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) + cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81))))*(qoffz + l25*sin(q1 + q2) + l1*sin(q1)) + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(ga0)*(qoffx + l25*cos(q1 + q2) + l1*cos(q1));
  p25y =  y0 + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0)*(qoffx + l25*cos(q1 + q2) + l1*cos(q1)) - (cos(ga0)*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)))) - cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0))*(qoffz + l25*sin(q1 + q2) + l1*sin(q1));
  p25z =  z0 + cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*(qoffz + l25*sin(q1 + q2) + l1*sin(q1)) - sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*(qoffx + l25*cos(q1 + q2) + l1*cos(q1));

  IntermediateState tauX, tauY, tauZ;
  double xq = 2*qr; double yq = 2*qr; double zq = 0.2; double m = 1.0/0.15;
  tauX = (ga1*m*(x3*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(ga0) - z3*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81))) + y3*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0))*zq*zq)/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)) - (m*xq*xq*(ga1*(x3*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(ga0) - z3*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81))) + y3*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0)) + (2*(y3*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0) + sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0)) - x3*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) - cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))) + z3*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)))))*(m*x3*(sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) + cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))) - m*y3*(cos(ga0)*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)))) - cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0)) + m*z3*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)) - m*x4*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) - cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))) + m*y4*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0) + sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0)) + m*z4*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)) - (ga1*m*yq*yq*(x3*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(ga0) - z3*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81))) + y3*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0)))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81));
  tauY = (ga1*m*(y3*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0) + sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0)) - x3*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) - cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))) + z3*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)))))*zq*zq)/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)) - (m*yq*yq*((2*(x3*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(ga0) - z3*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81))) + y3*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0))*(m*x3*(sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) + cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))) - m*y3*(cos(ga0)*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)))) - cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0)) + m*z3*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)) - ga1*(y3*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0) + sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0)) - x3*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) - cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))) + z3*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))) - m*x4*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) - cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))) + m*y4*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0) + sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0)) + m*z4*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)) - (ga1*m*xq*xq*(y3*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0) + sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0)) - x3*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) - cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))) + z3*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81));
  tauZ = ga2*m*zq*zq + (m*xq*xq*(x3*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(ga0) - z3*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81))) + y3*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0))*(y3*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0) + sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0)) - x3*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) - cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))) + z3*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))))/(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81)) - (m*yq*yq*(x3*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*cos(ga0) - z3*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81))) + y3*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(ga0))*(y3*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*cos(ga0) + sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0)) - x3*(cos(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))*sin(ga0) - cos(ga0)*sin(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))) + z3*cos(atan((x2*cos(ga0) + y2*sin(ga0))/(z2 + 9.81)))*sin(asin((-x2*sin(ga0) + y2*cos(ga0))/sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81))))))/(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81));

  std::cout <<"Added Dynamics" << std::endl;
  /**
  * Cost along trajectory
  *
  * Cost along trajectory is defined on higher 
  * derivatives of flat outputs, yaw and joint
  * velocities, weighted by matrix Q
  */
  DMatrix Q(6, 6); Q.setIdentity();
  // Q(4,4) = 0.01; Q(5,5) = 0.01;
  Q *= 0.01;
  Q(3,3) = 1.0;
  DVector offset(6); offset.setAll(0.0);
  Function eta;
  eta << x4 << y4 << z4 << ga2 << qd1 << qd2;

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

  ocp.subjectTo(f); // Dynamics

  double start_x, start_y, start_z, start_yaw,
        start_q1, start_q2,
        cx, cy,
        cx2, cy2,
        ox1, oy1, oz1,
        ox2, oy2, oz2,
        ox3, oy3, oz3,
        tr, ora;

  nh.getParam("quad_x0", start_x);
  nh.getParam("quad_y0", start_y);
  nh.getParam("quad_z0", start_z);
  nh.getParam("quad_ga0", start_yaw);

  nh.getParam("arm_q1", start_q1);
  nh.getParam("arm_q2", start_q2);

  nh.getParam("cyl_obs_x", cx); nh.getParam("cyl_obs_y", cy);
  nh.getParam("cyl_obs_x2", cx2); nh.getParam("cyl_obs_y2", cy2);
  nh.getParam("table_obs1_x", ox1); nh.getParam("table_obs1_y", oy1); nh.getParam("table_obs1_z", oz1);
  nh.getParam("table_obs2_x", ox2); nh.getParam("table_obs2_y", oy2); nh.getParam("table_obs2_z", oz2);
  nh.getParam("table_obs3_x", ox3); nh.getParam("table_obs3_y", oy3); nh.getParam("table_obs3_z", oz3);
  nh.getParam("table_r", tr);
  nh.getParam("obs_r", ora);

  ocp.subjectTo(AT_START, x0 == start_x);
  ocp.subjectTo(AT_START, y0 == start_y);
  ocp.subjectTo(AT_START, z0 == start_z);
  ocp.subjectTo(AT_START, ga0 == start_yaw);
  ocp.subjectTo(AT_START, q1 == start_q1);
  ocp.subjectTo(AT_START, q2 == start_q2);

  ocp.subjectTo(AT_START, x1 == 0.0);
  ocp.subjectTo(AT_START, y1 == 0.0);
  ocp.subjectTo(AT_START, z1 == 0.0);
  ocp.subjectTo(AT_START, x2 == 0.0);
  ocp.subjectTo(AT_START, y2 == 0.0);
  ocp.subjectTo(AT_START, z2 == 0.0);
  ocp.subjectTo(AT_START, x3 == 0.0);
  ocp.subjectTo(AT_START, y3 == 0.0);
  ocp.subjectTo(AT_START, z3 == 0.0);
  ocp.subjectTo(AT_START, ga1 == 0.0);
  ocp.subjectTo(AT_START, qd1 == 0.0);
  ocp.subjectTo(AT_START, qd2 == 0.0);

  ocp.subjectTo(-1.57 <= q1 <= 0.0); // joint limits
  ocp.subjectTo(-1.57 <= q2 <= 1.57); // joint limits
  ocp.subjectTo(-1.57 <= ga0 <= 1.57); // joint limits

  ocp.subjectTo(-0.78 <= qd1 <= 0.78); // joint velocity limits
  ocp.subjectTo(-0.78 <= qd2 <= 0.78);

  ocp.subjectTo(-1.0 <= x1 <= 1.0);
  ocp.subjectTo(-2.0 <= y1 <= 1.0);
  ocp.subjectTo(-1.0 <= z1 <= 1.0);
  ocp.subjectTo(-1.0 <= ga1 <= 1.0);

  // ocp.subjectTo(-0.5 <= r <= 0.5);
  // ocp.subjectTo(-0.5 <= p <= 0.5);
  ocp.subjectTo(T <= 12.0);
  // ocp.subjectTo(-10.0 <= tauX <= 10.0);
  // ocp.subjectTo(-10.0 <= tauY <= 10.0);
  // ocp.subjectTo(-10.0 <= tauZ <= 10.0) ;

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
  ocp.subjectTo(AT_END, qd1 == 0.0);
  ocp.subjectTo(AT_END, qd2 == 0.0);

  ocp.subjectTo(AT_END, (x0 + qoffx + (l1*cos(q1) + l2*cos(q1+q2))*cos(ga0)) == gx);
  ocp.subjectTo(AT_END, (y0 + (l1*cos(q1) + l2*cos(q1+q2))*sin(ga0)) == gy);
  ocp.subjectTo(AT_END, (z0 + qoffz + l1*sin(q1) + l2*sin(q1+q2)) == gz);

  ocp.subjectTo(AT_END, (q1+q2) == 0);

  ocp.subjectTo(x0 <= gx + 0.1);
  ocp.subjectTo(-1.5 <= y0 <= 1.5);
  ocp.subjectTo(0.001 <= z0 <= 5.0);

  /**
  * Set up optimization parameters
  */
  std::unique_ptr<OptimizationAlgorithm> algorithm;
  algorithm.reset(new OptimizationAlgorithm(ocp));
  algorithm->set(MAX_NUM_QP_ITERATIONS, 20);
  algorithm->set(INFEASIBLE_QP_HANDLING, IQH_STOP);
  algorithm->set( INTEGRATOR_TYPE, INT_RK45);
  algorithm->set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
  algorithm->set( HESSIAN_APPROXIMATION, GAUSS_NEWTON);
  algorithm->set(KKT_TOLERANCE, 1e-5);

  Grid timeGrid(ts, te, numSteps+1);
  VariablesGrid xi(16, timeGrid);
  VariablesGrid ui(6, timeGrid);

  algorithm->initializeDifferentialStates(xi);
  algorithm->initializeControls(ui);

  ROS_INFO("Setup took %f seconds", (ros::Time::now()-t1).toSec());

  algorithm->solve();

  algorithm->getDifferentialStates(xi);
  algorithm->getControls(ui);

  ocp.subjectTo(((pq1x-cx)*(pq1x-cx) + (pq1y-cy)*(pq1y-cy)) >= (ora+qr)*(ora+qr));
  ocp.subjectTo(((pq1x-cx2)*(pq1x-cx2) + (pq1y-cy2)*(pq1y-cy2)) >= (ora+qr)*(ora+qr));
  ocp.subjectTo(((pq1x-ox1)*(pq1x-ox1) + (pq1y-oy1)*(pq1y-oy1) + (pq1z-oz1)*(pq1z-oz1)) >= (qr+tr)*(qr+tr));
  ocp.subjectTo(((pq1x-ox2)*(pq1x-ox2) + (pq1y-oy2)*(pq1y-oy2) + (pq1z-oz2)*(pq1z-oz2)) >= (qr+tr)*(qr+tr));
  ocp.subjectTo(((pq1x-ox3)*(pq1x-ox3) + (pq1y-oy3)*(pq1y-oy3) + (pq1z-oz3)*(pq1z-oz3)) >= (qr+tr)*(qr+tr));

  ocp.subjectTo(((pq2x-cx)*(pq2x-cx) + (pq2y-cy)*(pq2y-cy)) >= (ora+qr)*(ora+qr));
  ocp.subjectTo(((pq2x-cx2)*(pq2x-cx2) + (pq2y-cy2)*(pq2y-cy2)) >= (ora+qr)*(ora+qr));
  ocp.subjectTo(((pq2x-ox1)*(pq2x-ox1) + (pq2y-oy1)*(pq2y-oy1) + (pq2z-oz1)*(pq2z-oz1)) >= (qr+tr)*(qr+tr));
  ocp.subjectTo(((pq2x-ox2)*(pq2x-ox2) + (pq2y-oy2)*(pq2y-oy2) + (pq2z-oz2)*(pq2z-oz2)) >= (qr+tr)*(qr+tr));
  ocp.subjectTo(((pq2x-ox3)*(pq2x-ox3) + (pq2y-oy3)*(pq2y-oy3) + (pq2z-oz3)*(pq2z-oz3)) >= (qr+tr)*(qr+tr));

  ocp.subjectTo(((pq3x-cx)*(pq3x-cx) + (pq3y-cy)*(pq3y-cy)) >= (ora+qr)*(ora+qr));
  ocp.subjectTo(((pq3x-cx2)*(pq3x-cx2) + (pq3y-cy2)*(pq3y-cy2)) >= (ora+qr)*(ora+qr));
  ocp.subjectTo(((pq3x-ox1)*(pq3x-ox1) + (pq3y-oy1)*(pq3y-oy1) + (pq3z-oz1)*(pq3z-oz1)) >= (qr+tr)*(qr+tr));
  ocp.subjectTo(((pq3x-ox2)*(pq3x-ox2) + (pq3y-oy2)*(pq3y-oy2) + (pq3z-oz2)*(pq3z-oz2)) >= (qr+tr)*(qr+tr));
  ocp.subjectTo(((pq3x-ox3)*(pq3x-ox3) + (pq3y-oy3)*(pq3y-oy3) + (pq3z-oz3)*(pq3z-oz3)) >= (qr+tr)*(qr+tr));

  ocp.subjectTo(((pq4x-cx)*(pq4x-cx) + (pq4y-cy)*(pq4y-cy)) >= (ora+qr)*(ora+qr));
  ocp.subjectTo(((pq4x-cx2)*(pq4x-cx2) + (pq4y-cy2)*(pq4y-cy2)) >= (ora+qr)*(ora+qr));
  ocp.subjectTo(((pq4x-ox1)*(pq4x-ox1) + (pq4y-oy1)*(pq4y-oy1) + (pq4z-oz1)*(pq4z-oz1)) >= (qr+tr)*(qr+tr));
  ocp.subjectTo(((pq4x-ox2)*(pq4x-ox2) + (pq4y-oy2)*(pq4y-oy2) + (pq4z-oz2)*(pq4z-oz2)) >= (qr+tr)*(qr+tr));
  ocp.subjectTo(((pq4x-ox3)*(pq4x-ox3) + (pq4y-oy3)*(pq4y-oy3) + (pq4z-oz3)*(pq4z-oz3)) >= (qr+tr)*(qr+tr));

  ocp.subjectTo(((eex-cx)*(eex-cx) + (eey-cy)*(eey-cy)) >= (2*lr+ora)*(2*lr+ora));
  ocp.subjectTo(((eex-cx2)*(eex-cx2) + (eey-cy2)*(eey-cy2)) >= (2*lr+ora)*(2*lr+ora));
  ocp.subjectTo(((eex-ox1)*(eex-ox1) + (eey-oy1)*(eey-oy1) + (eez-oz1)*(eez-oz1)) >= (2*lr+tr)*(2*lr+tr));
  ocp.subjectTo(((eex-ox2)*(eex-ox2) + (eey-oy2)*(eey-oy2) + (eez-oz2)*(eez-oz2)) >= (2*lr+tr)*(2*lr+tr));
  ocp.subjectTo(((eex-ox3)*(eex-ox3) + (eey-oy3)*(eey-oy3) + (eez-oz3)*(eez-oz3)) >= (2*lr+tr)*(2*lr+tr));

  ocp.subjectTo(((p11x-cx)*(p11x-cx) + (p11y-cy)*(p11y-cy)) >= (lr+ora)*(lr+ora));
  ocp.subjectTo(((p11x-cx2)*(p11x-cx2) + (p11y-cy2)*(p11y-cy2)) >= (lr+ora)*(lr+ora));
  ocp.subjectTo(((p11x-ox1)*(p11x-ox1) + (p11y-oy1)*(p11y-oy1) + (p11z-oz1)*(p11z-oz1)) >= (lr+tr)*(lr+tr));
  ocp.subjectTo(((p11x-ox2)*(p11x-ox2) + (p11y-oy2)*(p11y-oy2) + (p11z-oz2)*(p11z-oz2)) >= (lr+tr)*(lr+tr));
  ocp.subjectTo(((p11x-ox3)*(p11x-ox3) + (p11y-oy3)*(p11y-oy3) + (p11z-oz3)*(p11z-oz3)) >= (lr+tr)*(lr+tr));

  ocp.subjectTo(((p12x-cx)*(p12x-cx) + (p12y-cy)*(p12y-cy)) >= (lr+ora)*(lr+ora));
  ocp.subjectTo(((p12x-cx2)*(p12x-cx2) + (p12y-cy2)*(p12y-cy2)) >= (lr+ora)*(lr+ora));
  ocp.subjectTo(((p12x-ox1)*(p12x-ox1) + (p12y-oy1)*(p12y-oy1) + (p12z-oz1)*(p12z-oz1)) >= (lr+tr)*(lr+tr));
  ocp.subjectTo(((p12x-ox2)*(p12x-ox2) + (p12y-oy2)*(p12y-oy2) + (p12z-oz2)*(p12z-oz2)) >= (lr+tr)*(lr+tr));
  ocp.subjectTo(((p12x-ox3)*(p12x-ox3) + (p12y-oy3)*(p12y-oy3) + (p12z-oz3)*(p12z-oz3)) >= (lr+tr)*(lr+tr));

  ocp.subjectTo(((p13x-cx)*(p13x-cx) + (p13y-cy)*(p13y-cy)) >= (lr+ora)*(lr+ora));
  ocp.subjectTo(((p13x-cx2)*(p13x-cx2) + (p13y-cy2)*(p13y-cy2)) >= (lr+ora)*(lr+ora));
  ocp.subjectTo(((p13x-ox1)*(p13x-ox1) + (p13y-oy1)*(p13y-oy1) + (p13z-oz1)*(p13z-oz1)) >= (lr+tr)*(lr+tr));
  ocp.subjectTo(((p13x-ox2)*(p13x-ox2) + (p13y-oy2)*(p13y-oy2) + (p13z-oz2)*(p13z-oz2)) >= (lr+tr)*(lr+tr));
  ocp.subjectTo(((p13x-ox3)*(p13x-ox3) + (p13y-oy3)*(p13y-oy3) + (p13z-oz3)*(p13z-oz3)) >= (lr+tr)*(lr+tr));

  ocp.subjectTo(((p14x-cx2)*(p14x-cx2) + (p14y-cy2)*(p14y-cy2)) >= (lr+ora)*(lr+ora));
  ocp.subjectTo(((p14x-cx)*(p14x-cx) + (p14y-cy)*(p14y-cy)) >= (lr+ora)*(lr+ora));
  ocp.subjectTo(((p14x-ox1)*(p14x-ox1) + (p14y-oy1)*(p14y-oy1) + (p14z-oz1)*(p14z-oz1)) >= (lr+tr)*(lr+tr));
  ocp.subjectTo(((p14x-ox2)*(p14x-ox2) + (p14y-oy2)*(p14y-oy2) + (p14z-oz2)*(p14z-oz2)) >= (lr+tr)*(lr+tr));
  ocp.subjectTo(((p14x-ox3)*(p14x-ox3) + (p14y-oy3)*(p14y-oy3) + (p14z-oz3)*(p14z-oz3)) >= (lr+tr)*(lr+tr));

  ocp.subjectTo(((p21x-cx)*(p21x-cx) + (p21y-cy)*(p21y-cy)) >= (lr+ora)*(lr+ora));
  ocp.subjectTo(((p21x-cx2)*(p21x-cx2) + (p21y-cy2)*(p21y-cy2)) >= (lr+ora)*(lr+ora));
  ocp.subjectTo(((p21x-ox1)*(p21x-ox1) + (p21y-oy1)*(p21y-oy1) + (p21z-oz1)*(p21z-oz1)) >= (lr+tr)*(lr+tr));
  ocp.subjectTo(((p21x-ox2)*(p21x-ox2) + (p21y-oy2)*(p21y-oy2) + (p21z-oz2)*(p21z-oz2)) >= (lr+tr)*(lr+tr));
  ocp.subjectTo(((p21x-ox3)*(p21x-ox3) + (p21y-oy3)*(p21y-oy3) + (p21z-oz3)*(p21z-oz3)) >= (lr+tr)*(lr+tr));

  ocp.subjectTo(((p22x-cx2)*(p22x-cx2) + (p22y-cy2)*(p22y-cy2)) >= (lr+ora)*(lr+ora));
  ocp.subjectTo(((p22x-cx)*(p22x-cx) + (p22y-cy)*(p22y-cy)) >= (lr+ora)*(lr+ora));
  ocp.subjectTo(((p22x-ox1)*(p22x-ox1) + (p22y-oy1)*(p22y-oy1) + (p22z-oz1)*(p22z-oz1)) >= (lr+tr)*(lr+tr));
  ocp.subjectTo(((p22x-ox2)*(p22x-ox2) + (p22y-oy2)*(p22y-oy2) + (p22z-oz2)*(p22z-oz2)) >= (lr+tr)*(lr+tr));
  ocp.subjectTo(((p22x-ox3)*(p22x-ox3) + (p22y-oy3)*(p22y-oy3) + (p22z-oz3)*(p22z-oz3)) >= (lr+tr)*(lr+tr));

  ocp.subjectTo(((p23x-cx)*(p23x-cx) + (p23y-cy)*(p23y-cy)) >= (lr+ora)*(lr+ora));
  ocp.subjectTo(((p23x-cx2)*(p23x-cx2) + (p23y-cy2)*(p23y-cy2)) >= (lr+ora)*(lr+ora));
  ocp.subjectTo(((p23x-ox1)*(p23x-ox1) + (p23y-oy1)*(p23y-oy1) + (p23z-oz1)*(p23z-oz1)) >= (lr+tr)*(lr+tr));
  ocp.subjectTo(((p23x-ox2)*(p23x-ox2) + (p23y-oy2)*(p23y-oy2) + (p23z-oz2)*(p23z-oz2)) >= (lr+tr)*(lr+tr));
  ocp.subjectTo(((p23x-ox3)*(p23x-ox3) + (p23y-oy3)*(p23y-oy3) + (p23z-oz3)*(p23z-oz3)) >= (lr+tr)*(lr+tr));

  ocp.subjectTo(((p24x-cx)*(p24x-cx) + (p24y-cy)*(p24y-cy)) >= (lr+ora)*(lr+ora));
  ocp.subjectTo(((p24x-cx2)*(p24x-cx2) + (p24y-cy2)*(p24y-cy2)) >= (lr+ora)*(lr+ora));
  ocp.subjectTo(((p24x-ox1)*(p24x-ox1) + (p24y-oy1)*(p24y-oy1) + (p24z-oz1)*(p24z-oz1)) >= (lr+tr)*(lr+tr));
  ocp.subjectTo(((p24x-ox2)*(p24x-ox2) + (p24y-oy2)*(p24y-oy2) + (p24z-oz2)*(p24z-oz2)) >= (lr+tr)*(lr+tr));
  ocp.subjectTo(((p24x-ox3)*(p24x-ox3) + (p24y-oy3)*(p24y-oy3) + (p24z-oz3)*(p24z-oz3)) >= (lr+tr)*(lr+tr));

  ocp.subjectTo(((p25x-cx)*(p25x-cx) + (p25y-cy)*(p25y-cy)) >= (lr+ora)*(lr+ora));
  ocp.subjectTo(((p25x-cx2)*(p25x-cx2) + (p25y-cy2)*(p25y-cy2)) >= (lr+ora)*(lr+ora));
  ocp.subjectTo(((p25x-ox1)*(p25x-ox1) + (p25y-oy1)*(p25y-oy1) + (p25z-oz1)*(p25z-oz1)) >= (lr+tr)*(lr+tr));
  ocp.subjectTo(((p25x-ox2)*(p25x-ox2) + (p25y-oy2)*(p25y-oy2) + (p25z-oz2)*(p25z-oz2)) >= (lr+tr)*(lr+tr));
  ocp.subjectTo(((p25x-ox3)*(p25x-ox3) + (p25y-oy3)*(p25y-oy3) + (p25z-oz3)*(p25z-oz3)) >= (lr+tr)*(lr+tr));

  algorithm.reset(new OptimizationAlgorithm(ocp));
  algorithm->set(MAX_NUM_QP_ITERATIONS, 20);
  algorithm->set(INFEASIBLE_QP_HANDLING, IQH_STOP);
  algorithm->set(INTEGRATOR_TYPE, INT_RK45);
  algorithm->set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
  algorithm->set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
  algorithm->set(KKT_TOLERANCE, 1e-5);

  ROS_INFO("Number of parameters = %i", algorithm->getNP());

  algorithm->initializeDifferentialStates(xi);
  algorithm->initializeControls(ui);

  if(algorithm->solve() != SUCCESSFUL_RETURN) {
    ROS_WARN("Failed to solve. Exiting...");
    return 0;
  }

  ROS_INFO("Final OCP took %f seconds to solve", (ros::Time::now()-t1).toSec());

  VariablesGrid states(16, timeGrid);
  VariablesGrid controls(6, timeGrid);
  DVector params(4); 
  algorithm->getDifferentialStates(states);
  algorithm->getControls(controls);
  algorithm->getParameters(params);
  double m1 = params(0);
  double m2 = params(1);
  double c1 = params(2);
  double c2 = params(3);
  ROS_INFO("Parameters : %f %f %f %f", m1, m2, c1, c2);
  /**
  * Visuaize trajectory in Rviz
  */
  ros::Duration(2.0).sleep();
  tf::TransformBroadcaster br;
  ros::Publisher jointPub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
  ros::Publisher goalPub = nh.advertise<visualization_msgs::Marker>("/goal_marker", 2);
  ros::Publisher quadPub = nh.advertise<visualization_msgs::Marker>("/quad_marker", 5);
  ros::Publisher obsPub = nh.advertise<visualization_msgs::Marker>("/obs_marker", 20);
  ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("/odometry",1);

  visualization_msgs::Marker cage;
  cage.header.frame_id = "world";
  cage.id = 0;
  cage.type = visualization_msgs::Marker::CUBE;
  cage.action = visualization_msgs::Marker::ADD;
  cage.pose.position.x = 1.75;
  cage.pose.position.y = 0.0;
  cage.pose.position.z = 1.5;
  cage.pose.orientation.x = 0.0;
  cage.pose.orientation.y = 0.0;
  cage.pose.orientation.z = 0.0;
  cage.pose.orientation.w = 1.0;
  cage.scale.x = 7.5;
  cage.scale.y = 4.2;
  cage.scale.z = 3.0;
  cage.color.a = 0.2;
  cage.color.r = 1.0;
  cage.color.g = 0.0;
  cage.color.b = 0.0;

  visualization_msgs::Marker quad_body_marker1;
  quad_body_marker1.header.frame_id = "baselink";
  quad_body_marker1.id = 18;
  quad_body_marker1.type = visualization_msgs::Marker::SPHERE;
  quad_body_marker1.action = visualization_msgs::Marker::ADD;
  quad_body_marker1.pose.position.x = qr;
  quad_body_marker1.pose.position.y = qr;
  quad_body_marker1.pose.position.z = 0.0;
  quad_body_marker1.pose.orientation.x = 0.0;
  quad_body_marker1.pose.orientation.y = 0.0;
  quad_body_marker1.pose.orientation.z = 0.0;
  quad_body_marker1.pose.orientation.w = 1.0;
  quad_body_marker1.scale.x = 2*qr;
  quad_body_marker1.scale.y = 2*qr;
  quad_body_marker1.scale.z = 2*qr;
  quad_body_marker1.color.a = 0.1;
  quad_body_marker1.color.r = 0.0;
  quad_body_marker1.color.g = 0.0;
  quad_body_marker1.color.b = 1.0;

  visualization_msgs::Marker quad_body_marker2;
  quad_body_marker2.header.frame_id = "baselink";
  quad_body_marker2.id = 19;
  quad_body_marker2.type = visualization_msgs::Marker::SPHERE;
  quad_body_marker2.action = visualization_msgs::Marker::ADD;
  quad_body_marker2.pose.position.x = -qr;
  quad_body_marker2.pose.position.y = qr;
  quad_body_marker2.pose.position.z = 0.0;
  quad_body_marker2.pose.orientation.x = 0.0;
  quad_body_marker2.pose.orientation.y = 0.0;
  quad_body_marker2.pose.orientation.z = 0.0;
  quad_body_marker2.pose.orientation.w = 1.0;
  quad_body_marker2.scale.x = 2*qr;
  quad_body_marker2.scale.y = 2*qr;
  quad_body_marker2.scale.z = 2*qr;
  quad_body_marker2.color.a = 0.1;
  quad_body_marker2.color.r = 0.0;
  quad_body_marker2.color.g = 0.0;
  quad_body_marker2.color.b = 1.0;

  visualization_msgs::Marker quad_body_marker3;
  quad_body_marker3.header.frame_id = "baselink";
  quad_body_marker3.id = 20;
  quad_body_marker3.type = visualization_msgs::Marker::SPHERE;
  quad_body_marker3.action = visualization_msgs::Marker::ADD;
  quad_body_marker3.pose.position.x = -qr;
  quad_body_marker3.pose.position.y = -qr;
  quad_body_marker3.pose.position.z = 0.0;
  quad_body_marker3.pose.orientation.x = 0.0;
  quad_body_marker3.pose.orientation.y = 0.0;
  quad_body_marker3.pose.orientation.z = 0.0;
  quad_body_marker3.pose.orientation.w = 1.0;
  quad_body_marker3.scale.x = 2*qr;
  quad_body_marker3.scale.y = 2*qr;
  quad_body_marker3.scale.z = 2*qr;
  quad_body_marker3.color.a = 0.1;
  quad_body_marker3.color.r = 0.0;
  quad_body_marker3.color.g = 0.0;
  quad_body_marker3.color.b = 1.0;

  visualization_msgs::Marker quad_body_marker4;
  quad_body_marker4.header.frame_id = "baselink";
  quad_body_marker4.id = 21;
  quad_body_marker4.type = visualization_msgs::Marker::SPHERE;
  quad_body_marker4.action = visualization_msgs::Marker::ADD;
  quad_body_marker4.pose.position.x = qr;
  quad_body_marker4.pose.position.y = -qr;
  quad_body_marker4.pose.position.z = 0.0;
  quad_body_marker4.pose.orientation.x = 0.0;
  quad_body_marker4.pose.orientation.y = 0.0;
  quad_body_marker4.pose.orientation.z = 0.0;
  quad_body_marker4.pose.orientation.w = 1.0;
  quad_body_marker4.scale.x = 2*qr;
  quad_body_marker4.scale.y = 2*qr;
  quad_body_marker4.scale.z = 2*qr;
  quad_body_marker4.color.a = 0.1;
  quad_body_marker4.color.r = 0.0;
  quad_body_marker4.color.g = 0.0;
  quad_body_marker4.color.b = 1.0;

  visualization_msgs::Marker goal_marker;
  goal_marker.header.frame_id = "world";
  goal_marker.id = 2;
  goal_marker.type = visualization_msgs::Marker::CUBE;
  goal_marker.action = visualization_msgs::Marker::ADD;
  goal_marker.pose.position.x = goal(0);
  goal_marker.pose.position.y = goal(1);
  goal_marker.pose.position.z = goal(2);
  goal_marker.pose.orientation.x = 0.0;
  goal_marker.pose.orientation.y = 0.0;
  goal_marker.pose.orientation.z = 0.0;
  goal_marker.pose.orientation.w = 1.0;
  goal_marker.scale.x = 0.1;
  goal_marker.scale.y = 0.1;
  goal_marker.scale.z = 0.1;
  goal_marker.color.a = 0.7; 
  goal_marker.color.r = 0.0;
  goal_marker.color.g = 1.0;
  goal_marker.color.b = 0.0;

  visualization_msgs::Marker obs_marker;
  obs_marker.header.frame_id = "world";
  obs_marker.id = 3;
  obs_marker.type = visualization_msgs::Marker::CYLINDER;
  obs_marker.action = visualization_msgs::Marker::ADD;
  obs_marker.pose.position.x = cx;
  obs_marker.pose.position.y = cy;
  obs_marker.pose.position.z = 1.5;
  obs_marker.pose.orientation.x = 0.0;
  obs_marker.pose.orientation.y = 0.0;
  obs_marker.pose.orientation.z = 0.0;
  obs_marker.pose.orientation.w = 1.0;
  obs_marker.scale.x = ora;
  obs_marker.scale.y = ora;
  obs_marker.scale.z = 3.0;
  obs_marker.color.a = 1.0;
  obs_marker.color.r = 1.0;
  obs_marker.color.g = 0.0;
  obs_marker.color.b = 0.0;


  visualization_msgs::Marker obs_marker2;
  obs_marker2.header.frame_id = "world";
  obs_marker2.id = 17;
  obs_marker2.type = visualization_msgs::Marker::CYLINDER;
  obs_marker2.action = visualization_msgs::Marker::ADD;
  obs_marker2.pose.position.x = cx2;
  obs_marker2.pose.position.y = cy2;
  obs_marker2.pose.position.z = 1.5;
  obs_marker2.pose.orientation.x = 0.0;
  obs_marker2.pose.orientation.y = 0.0;
  obs_marker2.pose.orientation.z = 0.0;
  obs_marker2.pose.orientation.w = 1.0;
  obs_marker2.scale.x = ora;
  obs_marker2.scale.y = ora;
  obs_marker2.scale.z = 3.0;
  obs_marker2.color.a = 1.0;
  obs_marker2.color.r = 1.0;
  obs_marker2.color.g = 0.0;
  obs_marker2.color.b = 0.0;

  visualization_msgs::Marker marker1;
  marker1.header.frame_id = "world";
  marker1.id = 4;
  marker1.type = visualization_msgs::Marker::SPHERE;
  marker1.action = visualization_msgs::Marker::ADD;
  marker1.pose.position.x = ox1;
  marker1.pose.position.y = oy1;
  marker1.pose.position.z = oz1;
  marker1.pose.orientation.x = 0.0;
  marker1.pose.orientation.y = 0.0;
  marker1.pose.orientation.z = 0.0;
  marker1.pose.orientation.w = 1.0;
  marker1.scale.x = 2*tr;
  marker1.scale.y = 2*tr;
  marker1.scale.z = 2*tr;
  marker1.color.a = 1.0;
  marker1.color.r = 1.0;
  marker1.color.g = 0.0;
  marker1.color.b = 0.0;

  visualization_msgs::Marker marker2;
  marker2.header.frame_id = "world";
  marker2.id = 5;
  marker2.type = visualization_msgs::Marker::SPHERE;
  marker2.action = visualization_msgs::Marker::ADD;
  marker2.pose.position.x = ox2;
  marker2.pose.position.y = oy2;
  marker2.pose.position.z = oz2;
  marker2.pose.orientation.x = 0.0;
  marker2.pose.orientation.y = 0.0;
  marker2.pose.orientation.z = 0.0;
  marker2.pose.orientation.w = 1.0;
  marker2.scale.x = 2*tr;
  marker2.scale.y = 2*tr;
  marker2.scale.z = 2*tr;
  marker2.color.a = 1.0;
  marker2.color.r = 1.0;
  marker2.color.g = 0.0;
  marker2.color.b = 0.0;

  visualization_msgs::Marker marker3;
  marker3.header.frame_id = "world";
  marker3.id = 6;
  marker3.type = visualization_msgs::Marker::SPHERE;
  marker3.action = visualization_msgs::Marker::ADD;
  marker3.pose.position.x = ox3;
  marker3.pose.position.y = oy3;
  marker3.pose.position.z = oz3;
  marker3.pose.orientation.x = 0.0;
  marker3.pose.orientation.y = 0.0;
  marker3.pose.orientation.z = 0.0;
  marker3.pose.orientation.w = 1.0;
  marker3.scale.x = 2*tr;
  marker3.scale.y = 2*tr;
  marker3.scale.z = 2*tr;
  marker3.color.a = 1.0;
  marker3.color.r = 1.0;
  marker3.color.g = 0.0;
  marker3.color.b = 0.0;

  visualization_msgs::Marker link1marker1;
  link1marker1.header.frame_id = "link1";
  link1marker1.id = 7;
  link1marker1.type = visualization_msgs::Marker::SPHERE;
  link1marker1.action = visualization_msgs::Marker::ADD;
  link1marker1.pose.position.x = 0.0;
  link1marker1.pose.position.y = 0.0;
  link1marker1.pose.position.z = l11;
  link1marker1.pose.orientation.x = 0.0;
  link1marker1.pose.orientation.y = 0.0;
  link1marker1.pose.orientation.z = 0.0;
  link1marker1.pose.orientation.w = 1.0;
  link1marker1.scale.x = 2*lr;
  link1marker1.scale.y = 2*lr;
  link1marker1.scale.z = 2*lr;
  link1marker1.color.a = 0.5;
  link1marker1.color.r = 1.0;
  link1marker1.color.g = 0.0;
  link1marker1.color.b = 0.0;

  visualization_msgs::Marker link1marker2;
  link1marker2.header.frame_id = "link1";
  link1marker2.id = 8;
  link1marker2.type = visualization_msgs::Marker::SPHERE;
  link1marker2.action = visualization_msgs::Marker::ADD;
  link1marker2.pose.position.x = 0.0;
  link1marker2.pose.position.y = 0.0;
  link1marker2.pose.position.z = l12;
  link1marker2.pose.orientation.x = 0.0;
  link1marker2.pose.orientation.y = 0.0;
  link1marker2.pose.orientation.z = 0.0;
  link1marker2.pose.orientation.w = 1.0;
  link1marker2.scale.x = 2*lr;
  link1marker2.scale.y = 2*lr;
  link1marker2.scale.z = 2*lr;
  link1marker2.color.a = 0.5;
  link1marker2.color.r = 1.0;
  link1marker2.color.g = 0.0;
  link1marker2.color.b = 0.0;

  visualization_msgs::Marker link1marker3;
  link1marker3.header.frame_id = "link1";
  link1marker3.id = 9;
  link1marker3.type = visualization_msgs::Marker::SPHERE;
  link1marker3.action = visualization_msgs::Marker::ADD;
  link1marker3.pose.position.x = 0.0;
  link1marker3.pose.position.y = 0.0;
  link1marker3.pose.position.z = l13;
  link1marker3.pose.orientation.x = 0.0;
  link1marker3.pose.orientation.y = 0.0;
  link1marker3.pose.orientation.z = 0.0;
  link1marker3.pose.orientation.w = 1.0;
  link1marker3.scale.x = 2*lr;
  link1marker3.scale.y = 2*lr;
  link1marker3.scale.z = 2*lr;
  link1marker3.color.a = 0.5;
  link1marker3.color.r = 1.0;
  link1marker3.color.g = 0.0;
  link1marker3.color.b = 0.0;

  visualization_msgs::Marker link1marker4;
  link1marker4.header.frame_id = "link1";
  link1marker4.id = 10;
  link1marker4.type = visualization_msgs::Marker::SPHERE;
  link1marker4.action = visualization_msgs::Marker::ADD;
  link1marker4.pose.position.x = 0.0;
  link1marker4.pose.position.y = 0.0;
  link1marker4.pose.position.z = l14;
  link1marker4.pose.orientation.x = 0.0;
  link1marker4.pose.orientation.y = 0.0;
  link1marker4.pose.orientation.z = 0.0;
  link1marker4.pose.orientation.w = 1.0;
  link1marker4.scale.x = 2*lr;
  link1marker4.scale.y = 2*lr;
  link1marker4.scale.z = 2*lr;
  link1marker4.color.a = 0.5;
  link1marker4.color.r = 1.0;
  link1marker4.color.g = 0.0;
  link1marker4.color.b = 0.0;

  visualization_msgs::Marker link2marker1;
  link2marker1.header.frame_id = "link2";
  link2marker1.id = 11;
  link2marker1.type = visualization_msgs::Marker::SPHERE;
  link2marker1.action = visualization_msgs::Marker::ADD;
  link2marker1.pose.position.x = 0.0;
  link2marker1.pose.position.y = 0.0;
  link2marker1.pose.position.z = l21;
  link2marker1.pose.orientation.x = 0.0;
  link2marker1.pose.orientation.y = 0.0;
  link2marker1.pose.orientation.z = 0.0;
  link2marker1.pose.orientation.w = 1.0;
  link2marker1.scale.x = 2*lr;
  link2marker1.scale.y = 2*lr;
  link2marker1.scale.z = 2*lr;
  link2marker1.color.a = 0.5;
  link2marker1.color.r = 1.0;
  link2marker1.color.g = 0.0;
  link2marker1.color.b = 0.0;

  visualization_msgs::Marker link2marker2;
  link2marker2.header.frame_id = "link2";
  link2marker2.id = 12;
  link2marker2.type = visualization_msgs::Marker::SPHERE;
  link2marker2.action = visualization_msgs::Marker::ADD;
  link2marker2.pose.position.x = 0.0;
  link2marker2.pose.position.y = 0.0;
  link2marker2.pose.position.z = l22;
  link2marker2.pose.orientation.x = 0.0;
  link2marker2.pose.orientation.y = 0.0;
  link2marker2.pose.orientation.z = 0.0;
  link2marker2.pose.orientation.w = 1.0;
  link2marker2.scale.x = 2*lr;
  link2marker2.scale.y = 2*lr;
  link2marker2.scale.z = 2*lr;
  link2marker2.color.a = 0.5;
  link2marker2.color.r = 1.0;
  link2marker2.color.g = 0.0;
  link2marker2.color.b = 0.0;

  visualization_msgs::Marker link2marker3;
  link2marker3.header.frame_id = "link2";
  link2marker3.id = 13;
  link2marker3.type = visualization_msgs::Marker::SPHERE;
  link2marker3.action = visualization_msgs::Marker::ADD;
  link2marker3.pose.position.x = 0.0;
  link2marker3.pose.position.y = 0.0;
  link2marker3.pose.position.z = l23;
  link2marker3.pose.orientation.x = 0.0;
  link2marker3.pose.orientation.y = 0.0;
  link2marker3.pose.orientation.z = 0.0;
  link2marker3.pose.orientation.w = 1.0;
  link2marker3.scale.x = 2*lr;
  link2marker3.scale.y = 2*lr;
  link2marker3.scale.z = 2*lr;
  link2marker3.color.a = 0.5;
  link2marker3.color.r = 1.0;
  link2marker3.color.g = 0.0;
  link2marker3.color.b = 0.0;

  visualization_msgs::Marker link2marker4;
  link2marker4.header.frame_id = "link2";
  link2marker4.id = 14;
  link2marker4.type = visualization_msgs::Marker::SPHERE;
  link2marker4.action = visualization_msgs::Marker::ADD;
  link2marker4.pose.position.x = 0.0;
  link2marker4.pose.position.y = 0.0;
  link2marker4.pose.position.z = l24;
  link2marker4.pose.orientation.x = 0.0;
  link2marker4.pose.orientation.y = 0.0;
  link2marker4.pose.orientation.z = 0.0;
  link2marker4.pose.orientation.w = 1.0;
  link2marker4.scale.x = 2*lr;
  link2marker4.scale.y = 2*lr;
  link2marker4.scale.z = 2*lr;
  link2marker4.color.a = 0.5;
  link2marker4.color.r = 1.0;
  link2marker4.color.g = 0.0;
  link2marker4.color.b = 0.0;

  visualization_msgs::Marker link2marker5;
  link2marker5.header.frame_id = "link2";
  link2marker5.id = 15;
  link2marker5.type = visualization_msgs::Marker::SPHERE;
  link2marker5.action = visualization_msgs::Marker::ADD;
  link2marker5.pose.position.x = 0.0;
  link2marker5.pose.position.y = 0.0;
  link2marker5.pose.position.z = l25;
  link2marker5.pose.orientation.x = 0.0;
  link2marker5.pose.orientation.y = 0.0;
  link2marker5.pose.orientation.z = 0.0;
  link2marker5.pose.orientation.w = 1.0;
  link2marker5.scale.x = 2*lr;
  link2marker5.scale.y = 2*lr;
  link2marker5.scale.z = 2*lr;
  link2marker5.color.a = 0.5;
  link2marker5.color.r = 1.0;
  link2marker5.color.g = 0.0;
  link2marker5.color.b = 0.0;


  visualization_msgs::Marker eemarker;
  eemarker.header.frame_id = "link2";
  eemarker.id = 16;
  eemarker.type = visualization_msgs::Marker::SPHERE;
  eemarker.action = visualization_msgs::Marker::ADD;
  eemarker.pose.position.x = 0.0;
  eemarker.pose.position.y = 0.0;
  eemarker.pose.position.z = l2;
  eemarker.pose.orientation.x = 0.0;
  eemarker.pose.orientation.y = 0.0;
  eemarker.pose.orientation.z = 0.0;
  eemarker.pose.orientation.w = 1.0;
  eemarker.scale.x = 4*lr;
  eemarker.scale.y = 4*lr;
  eemarker.scale.z = 4*lr;
  eemarker.color.a = 0.5;
  eemarker.color.r = 1.0;
  eemarker.color.g = 0.0;
  eemarker.color.b = 0.0;

  tf::TransformListener listener;
  ros::Duration(0.5).sleep();
  ros::Time start_time = ros::Time::now();
  for(int tt=0; tt < numSteps+1; tt++) {

    double x = states(tt,0); double y = states(tt,1); double z = states(tt,2);
    double xdd = states(tt,6); double ydd = states(tt,7); double zdd = states(tt,8);
    double vx = states(tt,3); double vy = states(tt,4); double vz = states(tt,5);
    double yaw = states(tt,12); double yaw_rate = states(tt,13);
    double j1 = m1*states(tt,14) + c1; 
    double j2 = m1*states(tt,15) + c2;

    double T = sqrt(xdd*xdd + ydd*ydd + (zdd+9.81)*(zdd+9.81));
    double r = asin((-xdd*sin(yaw) + ydd*cos(yaw))/T);
    double p = atan((xdd*cos(yaw) + ydd*sin(yaw))/(zdd + 9.81));

    tf::Transform transform = tf::Transform(
     tf::createQuaternionFromRPY(0,0,states(tt, 12)), 
     tf::Vector3(states(tt, 0), states(tt, 1), states(tt, 2)));

    // states_log << (double)tt/10.0 <<" ";
    for(int xx=0; xx < 16; xx++) states_log << states(tt, xx) <<" ";
    states_log << "\n";

    for(int uu=0; uu < 6; uu++) controls_log << controls(tt, uu) <<" ";
    controls_log << "\n";

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "baselink"));

    // ROS_INFO("Thrust = %f", T);

    // double exp_x = x + (sin(asin((-xdd*sin(yaw) + ydd*cos(yaw))/sqrt(xdd*xdd + ydd*ydd + (zdd+9.81)*(zdd+9.81))))*sin(yaw) + cos(asin((-xdd*sin(yaw) + ydd*cos(yaw))/sqrt(xdd*xdd + ydd*ydd + (zdd+9.81)*(zdd+9.81))))*cos(yaw)*sin(atan((xdd*cos(yaw) + ydd*sin(yaw))/(zdd + 9.81))))*(l2*sin(j1 + j2) + l1*sin(j1)) + cos(atan((xdd*cos(yaw) + ydd*sin(yaw))/(zdd + 9.81)))*cos(yaw)*(l2*cos(j1 + j2) + l1*cos(j1));
    // double exp_y = y + cos(atan((xdd*cos(yaw) + ydd*sin(yaw))/(zdd + 9.81)))*sin(yaw)*(l2*cos(j1 + j2) + l1*cos(j1)) - (cos(yaw)*sin(asin((-xdd*sin(yaw) + ydd*cos(yaw))/sqrt(xdd*xdd + ydd*ydd + (zdd+9.81)*(zdd+9.81)))) - cos(asin((-xdd*sin(yaw) + ydd*cos(yaw))/sqrt(xdd*xdd + ydd*ydd + (zdd+9.81)*(zdd+9.81))))*sin(atan((xdd*cos(yaw) + ydd*sin(yaw))/(zdd + 9.81)))*sin(yaw))*(l2*sin(j1 + j2) + l1*sin(j1));
    // double exp_z = z + cos(atan((xdd*cos(yaw) + ydd*sin(yaw))/(zdd + 9.81)))*cos(asin((-xdd*sin(yaw) + ydd*cos(yaw))/sqrt(xdd*xdd + ydd*ydd + (zdd+9.81)*(zdd+9.81))))*(l2*sin(j1 + j2) + l1*sin(j1)) - sin(atan((xdd*cos(yaw) + ydd*sin(yaw))/(zdd + 9.81)))*(l2*cos(j1 + j2) + l1*cos(j1));

    goalPub.publish(goal_marker);
    goalPub.publish(cage);
    quadPub.publish(quad_body_marker1);
    quadPub.publish(quad_body_marker2);
    quadPub.publish(quad_body_marker3);
    quadPub.publish(quad_body_marker4);
    obsPub.publish(link2marker1);
    obsPub.publish(link2marker2);
    obsPub.publish(link2marker3);
    obsPub.publish(link2marker4);
    obsPub.publish(obs_marker);
    obsPub.publish(obs_marker2);
    obsPub.publish(marker1);
    obsPub.publish(marker2);
    obsPub.publish(marker3);
    obsPub.publish(link1marker1);
    obsPub.publish(link1marker2);
    obsPub.publish(link1marker3);
    obsPub.publish(link1marker4);
    obsPub.publish(link2marker5);
    obsPub.publish(eemarker);

    sensor_msgs::JointState joint_state;
    joint_state.name.push_back("airbasetolink1");
    joint_state.name.push_back("link1tolink2");

    // joint_state.position.push_back(-j1 + 1.57);
    // joint_state.position.push_back(-j2);
    joint_state.position.push_back(-states(tt, 14) + 1.57);
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
