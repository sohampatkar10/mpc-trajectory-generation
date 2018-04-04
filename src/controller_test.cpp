#include "ros/ros.h"
#include "mpc_controller.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "controller_test");
  ros::NodeHandle nh;
  double te; nh.getParam("te1", te);
  double num_steps; nh.getParam("numSteps", num_steps);
  std::unique_ptr<MPController> controller;
  controller.reset(new MPController(te, num_steps, nh));

  double ix, iy, iz;
  nh.getParam("quad_x0", ix);
  nh.getParam("quad_y0", iy);
  nh.getParam("quad_z0", iz);

  double gx, gy, gz;
  nh.getParam("goal_x", gx);
  nh.getParam("goal_y", gy);
  nh.getParam("goal_z", gz);

  ACADO::VariablesGrid states(16, 0.0, te, num_steps);
  ACADO::VariablesGrid controls(6, 0.0, te, num_steps);

  // controller->setInitialState(ix,iy,iz,0.0,1.57,-1.57);
  // controller->setGoal(gx, gy, gz);
  controller->runController(states, controls);

  controller.reset();

  // controller.reset(new MPController(te, num_steps, nh));
  // controller->setInitialState(ix,iy,iz,0.0);
  // controller->setGoal(gx, gy, gz);

  // double ox,oy,oz;
  // double l,w,h;
  // nh.getParam("table_obs1_x",ox);
  // nh.getParam("table_obs1_y",oy);
  // nh.getParam("table_obs1_z",oz);
  // nh.getParam("table_l",l);
  // nh.getParam("table_w",w);
  // nh.getParam("table_h",h);
  // controller->addSphericalObstacle(ox, oy, oz, 0.05);
  // controller->runController(states, controls);
  return 0;
}