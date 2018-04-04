#include "ros/ros.h"

#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include <acado/function/function.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/user_interaction.hpp>

class MPController {
public:
  MPController(double te, double numsteps, ros::NodeHandle& nh) :
      nh_(nh),
      ocp_(ACADO::OCP(0.0, te, numsteps)) {
    nh_.getParam("link_r", lr);
    nh_.getParam("quad_r", qr);

    ACADO::DifferentialEquation f(0.0, te);
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

    ACADO::DMatrix Q(6,6); Q.setIdentity();
    Q *= 0.01; Q(3,3) = 1.0;
    ACADO::DVector offset(6); offset.setAll(0.0);
    ACADO::Function eta;
    eta << x4 << y4 << z4 << ga2 << qd1 << qd2;

    ocp_.minimizeLSQ(Q, eta, offset);
    ocp_.subjectTo(f);

    ocp_.subjectTo(ACADO::AT_START, x1 == 0.0);
    ocp_.subjectTo(ACADO::AT_START, y1 == 0.0);
    ocp_.subjectTo(ACADO::AT_START, z1 == 0.0);
    ocp_.subjectTo(ACADO::AT_START, x2 == 0.0);
    ocp_.subjectTo(ACADO::AT_START, y2 == 0.0);
    ocp_.subjectTo(ACADO::AT_START, z2 == 0.0);
    ocp_.subjectTo(ACADO::AT_START, x3 == 0.0);
    ocp_.subjectTo(ACADO::AT_START, y3 == 0.0);
    ocp_.subjectTo(ACADO::AT_START, z3 == 0.0);
    ocp_.subjectTo(ACADO::AT_START, ga1 == 0.0);
    ocp_.subjectTo(ACADO::AT_START, qd1 == 0.0);
    ocp_.subjectTo(ACADO::AT_START, qd2 == 0.0);

    ocp_.subjectTo(ACADO::AT_END, x1 == 0.0);
    ocp_.subjectTo(ACADO::AT_END, y1 == 0.0);
    ocp_.subjectTo(ACADO::AT_END, z1 == 0.0);
    ocp_.subjectTo(ACADO::AT_END, x2 == 0.0);
    ocp_.subjectTo(ACADO::AT_END, y2 == 0.0);
    ocp_.subjectTo(ACADO::AT_END, z2 == 0.0);
    ocp_.subjectTo(ACADO::AT_END, x3 == 0.0);
    ocp_.subjectTo(ACADO::AT_END, y3 == 0.0);
    ocp_.subjectTo(ACADO::AT_END, z3 == 0.0);
    ocp_.subjectTo(ACADO::AT_END, ga1 == 0.0);
    ocp_.subjectTo(ACADO::AT_END, qd1 == 0.0);
    ocp_.subjectTo(ACADO::AT_END, qd2 == 0.0);

    ocp_.subjectTo(ACADO::AT_START, x0==0.0);
    ocp_.subjectTo(ACADO::AT_START, y0==0.0);
    ocp_.subjectTo(ACADO::AT_START, z0==1.5);
    ocp_.subjectTo(ACADO::AT_START, ga0==0.0);
    ocp_.subjectTo(ACADO::AT_START, q1 == 1.57);
    ocp_.subjectTo(ACADO::AT_START, q2 == -1.57); 

    ocp_.subjectTo(ACADO::AT_END, (x0 + qoffx + (l1*cos(q1) + l2*cos(q1+q2))*cos(ga0)) == 3.0);
    ocp_.subjectTo(ACADO::AT_END, (y0 + (l1*cos(q1) + l2*cos(q1+q2))*sin(ga0)) == 0.0);
    ocp_.subjectTo(ACADO::AT_END, (z0 + qoffz + l1*sin(q1) + l2*sin(q1+q2)) == 0.7);

  }

  void setGoal(double gx, double gy, double gz) {
    ocp_.subjectTo(ACADO::AT_END, (x0 + qoffx + (l1*cos(q1) + l2*cos(q1+q2))*cos(ga0)) == gx);
    ocp_.subjectTo(ACADO::AT_END, (y0 + (l1*cos(q1) + l2*cos(q1+q2))*sin(ga0)) == gy);
    ocp_.subjectTo(ACADO::AT_END, (z0 + qoffz + l1*sin(q1) + l2*sin(q1+q2)) == gz);
    // ocp_.subjectTo(x0 <= gx + 0.1);
  }

  void setInitialState(double xi, double yi, double zi, double yawi, double q1i, double q2i) {
    ocp_.subjectTo(ACADO::AT_START, x0==xi);
    ocp_.subjectTo(ACADO::AT_START, y0==yi);
    ocp_.subjectTo(ACADO::AT_START, z0==zi);
    ocp_.subjectTo(ACADO::AT_START, ga0==yawi);
    ocp_.subjectTo(ACADO::AT_START, q1 == q1i);
    ocp_.subjectTo(ACADO::AT_START, q2 == q2i); 
  }

  void addSphericalObstacle(double ox, double oy, double oz, double ora) {
    ocp_.subjectTo(((x0-ox)*(x0-ox) + (y0-oy)*(y0-oy) + (z0-oz)*(z0-oz)) >= (ora+qr)*(ora+qr));
    ocp_.subjectTo(((eex-ox)*(eex-ox) + (eey-oy)*(eey-oy)) >= (2*lr+ora)*(2*lr+ora));
    ocp_.subjectTo(((p11x-ox)*(p11x-ox) + (p11y-oy)*(p11y-oy)) >= (lr+ora)*(lr+ora));
    ocp_.subjectTo(((p12x-ox)*(p12x-ox) + (p12y-oy)*(p12y-oy)) >= (lr+ora)*(lr+ora));
    ocp_.subjectTo(((p13x-ox)*(p13x-ox) + (p13y-oy)*(p13y-oy)) >= (lr+ora)*(lr+ora));
    ocp_.subjectTo(((p14x-ox)*(p14x-ox) + (p14y-oy)*(p14y-oy)) >= (lr+ora)*(lr+ora));
    ocp_.subjectTo(((p21x-ox)*(p21x-ox) + (p21y-oy)*(p21y-oy)) >= (lr+ora)*(lr+ora));
    ocp_.subjectTo(((p22x-ox)*(p22x-ox) + (p22y-oy)*(p22y-oy)) >= (lr+ora)*(lr+ora));
    ocp_.subjectTo(((p23x-ox)*(p23x-ox) + (p23y-oy)*(p23y-oy)) >= (lr+ora)*(lr+ora));
    ocp_.subjectTo(((p24x-ox)*(p24x-ox) + (p24y-oy)*(p24y-oy)) >= (lr+ora)*(lr+ora));
    ocp_.subjectTo(((p25x-ox)*(p25x-ox) + (p25y-oy)*(p25y-oy)) >= (lr+ora)*(lr+ora));
  }

  void addBoxObstacle(double ox, double oy, double oz, double l, double w, double h) {
    ocp_.subjectTo(((x0-ox)*(x0-ox)-(l+qr)*(l+qr))*((y0-oy)*(y0-oy)-(w+qr)*(w+qr))*
                    ((z0-oz)*(z0-oz)-(h+qr)*(h+qr)) >= 0.00001); // > 0
    ocp_.subjectTo(((eex-ox)*(eex-ox)-(l+qr)*(l+qr))*((eey-oy)*(eey-oy)-(w+qr)*(w+qr))*
                    ((eez-oz)*(eez-oz)-(h+qr)*(h+qr)) >= 0.00001); // > 0
    ocp_.subjectTo(((p11x-ox)*(p11x-ox)-(l+qr)*(l+qr))*((p11y-oy)*(p11y-oy)-(w+qr)*(w+qr))*
                    ((p11z-oz)*(p11z-oz)-(h+qr)*(h+qr)) >= 0.00001); // > 0
    ocp_.subjectTo(((p12x-ox)*(p12x-ox)-(l+qr)*(l+qr))*((p12y-oy)*(p12y-oy)-(w+qr)*(w+qr))*
                    ((p12z-oz)*(p12z-oz)-(h+qr)*(h+qr)) >= 0.00001); // > 0
    ocp_.subjectTo(((p13x-ox)*(p13x-ox)-(l+qr)*(l+qr))*((p13y-oy)*(p13y-oy)-(w+qr)*(w+qr))*
                    ((p13z-oz)*(p13z-oz)-(h+qr)*(h+qr)) >= 0.00001); // > 0
    ocp_.subjectTo(((p14x-ox)*(p14x-ox)-(l+qr)*(l+qr))*((p14y-oy)*(p14y-oy)-(w+qr)*(w+qr))*
                    ((p14z-oz)*(p14z-oz)-(h+qr)*(h+qr)) >= 0.00001); // > 0
    ocp_.subjectTo(((p21x-ox)*(p21x-ox)-(l+qr)*(l+qr))*((p21y-oy)*(p21y-oy)-(w+qr)*(w+qr))*
                    ((p21z-oz)*(p21z-oz)-(h+qr)*(h+qr)) >= 0.00001); // > 0
    ocp_.subjectTo(((p22x-ox)*(p22x-ox)-(l+qr)*(l+qr))*((p22y-oy)*(p22y-oy)-(w+qr)*(w+qr))*
                    ((p22z-oz)*(p22z-oz)-(h+qr)*(h+qr)) >= 0.00001); // > 0
    ocp_.subjectTo(((p23x-ox)*(p23x-ox)-(l+qr)*(l+qr))*((p23y-oy)*(p23y-oy)-(w+qr)*(w+qr))*
                    ((p23z-oz)*(p23z-oz)-(h+qr)*(h+qr)) >= 0.00001); // > 0
    ocp_.subjectTo(((p24x-ox)*(p24x-ox)-(l+qr)*(l+qr))*((p24y-oy)*(p24y-oy)-(w+qr)*(w+qr))*
                    ((p24z-oz)*(p24z-oz)-(h+qr)*(h+qr)) >= 0.00001); // > 0
    ocp_.subjectTo(((p25x-ox)*(p25x-ox)-(l+qr)*(l+qr))*((p25y-oy)*(p25y-oy)-(w+qr)*(w+qr))*
                    ((p25z-oz)*(p25z-oz)-(h+qr)*(h+qr)) >= 0.00001); // > 0
  }

  bool runController(ACADO::VariablesGrid& states, ACADO::VariablesGrid& controls) {
    std::unique_ptr<ACADO::OptimizationAlgorithm> algorithm_; // Optimzation algorithm
    /**
    * Initialize Optimization
    */
    algorithm_.reset(new ACADO::OptimizationAlgorithm(ocp_));
    algorithm_->set(ACADO::MAX_NUM_QP_ITERATIONS, 50);
    algorithm_->set(ACADO::INFEASIBLE_QP_HANDLING, ACADO::IQH_STOP);
    algorithm_->set(ACADO::INTEGRATOR_TYPE, ACADO::INT_RK45);
    algorithm_->set(ACADO::DISCRETIZATION_TYPE, ACADO::MULTIPLE_SHOOTING);
    algorithm_->set(ACADO::HESSIAN_APPROXIMATION, ACADO::GAUSS_NEWTON);
    algorithm_->set(ACADO::KKT_TOLERANCE, 1e-6);

    algorithm_->initializeDifferentialStates(states);
    algorithm_->initializeControls(controls);

    if(!algorithm_->solve()) return false;
    algorithm_->getDifferentialStates(states);
    algorithm_->getControls(controls);
    return true;
  }

  ~MPController() {clearAllStaticCounters();}
private:
  ros::NodeHandle nh_;
  /**
  * States are quad position and upto 3rd derivative,
  * yaw and yaw rate
  */
  ACADO::DifferentialState x0, y0, z0, 
                           x1, y1, z1,
                           x2, y2, z2,
                           x3, y3, z3,
                           ga0, ga1,
                           q1, q2;
  /**
  * Controls are snap and angular acceleration around z
  */
  ACADO::Control x4, y4, z4, ga2, qd1, qd2;
  /**
  * Dynamics
  */
  ACADO::IntermediateState eex, eey, eez, T, r, p;

  ACADO::IntermediateState p11x, p11y, p11z;
  ACADO::IntermediateState p12x, p12y, p12z;
  ACADO::IntermediateState p13x, p13y, p13z;
  ACADO::IntermediateState p14x, p14y, p14z;

  ACADO::IntermediateState p21x, p21y, p21z;
  ACADO::IntermediateState p22x, p22y, p22z;
  ACADO::IntermediateState p23x, p23y, p23z;
  ACADO::IntermediateState p24x, p24y, p24z;
  ACADO::IntermediateState p25x, p25y, p25z;

  ACADO::OCP ocp_; // Optimal Control Problem
  double qr; 
  const double qoffx = 0.2;
  const double qoffz = -0.1;
  const double l1 = 0.255;
  const double l2 = 0.29;
  double lr; // link radius 
};