#include <ros/ros.h>
#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include <time.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "ocp_example");
  ros::NodeHandle nh;
  USING_NAMESPACE_ACADO
  const double h = 0.05;

  DifferentialState s,v;
  Control a;
  DiscretizedDifferentialEquation  f(h);

  Function eta;
  eta << a;

  OCP ocp(0.0, 2.0, 20);
  ocp.minimizeLSQ(eta);

  f << next(v) == v + h*a;
  f << next(s) == s + h*v;

  ocp.subjectTo(f);
  ocp.subjectTo(AT_START, s == 0.0);
  ocp.subjectTo(AT_START, v == 0.0);

  ocp.subjectTo(AT_END, s == 10.0);
  ocp.subjectTo(AT_END, v == 0.0);

  // ocp.subjectTo(-0.1 <= v <= 1.3);

  // GnuplotWindow window;
  // window.addSubplot( s, "THE DISTANCE s"      );
  // window.addSubplot( v, "THE VELOCITY v"      );
  // window.addSubplot( m, "THE MASS m"          );
  // window.addSubplot( u, "THE CONTROL INPUT u" );

  OptimizationAlgorithm algorithm(ocp);     // the optimization algorithm

  algorithm.set( INTEGRATOR_TYPE, INT_DISCRETE );
  // algorithm.set(DISCRETIZATION_TYPE, COLLOCATION);
  // algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
  algorithm.set( KKT_TOLERANCE, 1e-2);

  // algorithm << window;
  ros::Time ts = ros::Time::now();
  algorithm.solve();                        // solves the problem.
  ROS_INFO("Time taken: %f seconds", (ros::Time::now()-ts).toSec());
  ts = ros::Time::now();
  algorithm.solve();                        // solves the problem.
  ROS_INFO("Time taken: %f seconds", (ros::Time::now()-ts).toSec());

  return 0;
}
