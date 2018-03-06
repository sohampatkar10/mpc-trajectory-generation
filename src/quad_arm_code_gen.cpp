// Acado
#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include <acado/function/function.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/user_interaction.hpp>
#include <acado_code_generation.hpp>

USING_NAMESPACE_ACADO

int main(int argc, char** argv) {

  DifferentialState x0, y0, z0, 
                    x1, y1, z1,
                    x2, y2, z2,
                    x3, y3, z3,
                    ga0, ga1,
                    q1, q2;

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

  DMatrix Q(6, 6);
  Q(0,0) = 0.01; Q(1,1) = 0.01; Q(2,2) = 0.01;
  Q(3,3) = 0.01; Q(4,4) = 0.01; Q(5,5) = 0.01;

  Function eta;
  eta << x4 << y4 << z4 << ga2 << qd1 << qd2;

  double l1 = 0.175;
  double l2 = 0.42;

  OCP ocp(ts, te, numSteps);

  ocp.minimizeLSQ(Q, eta); // Trajectory Cost

  Function phi;
  phi << 0.0;
  DMatrix W(1,1); W.setIdentity();
  ocp.minimizeLSQEndTerm(W, phi); // Terminal Cost

  ocp.subjectTo(f); // Dynamics

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

  double gx = 2.0; double gy = 0.0; double gz = 2.0;

  ocp.subjectTo(AT_END, (x0 + (l1*cos(q1) + l2*cos(q1+q2))*cos(ga0)) == gx);
  ocp.subjectTo(AT_END, (y0 + (l1*cos(q1) + l2*cos(q1+q2))*sin(ga0)) == gy);
  ocp.subjectTo(AT_END, (z0 + l1*sin(q1) + l2*sin(q1+q2)) == gz);

  ocp.subjectTo(AT_END, (l1*sin(q1) + l2*sin(q1 + q2)) <= -0.05);
  ocp.subjectTo(AT_END, (l1*cos(q1) + l2*cos(q1 + q2)) >= 0.05);

  double obs[] = {1.4};
  double ora = 0.5;
  for(int ii = 0; ii < 1; ii++) {
    ocp.subjectTo((((x0 + (l1*cos(q1) + l2*cos(q1 + q2))*cos(ga0))-obs[ii])*((x0 + (l1*cos(q1) + l2*cos(q1 + q2))*cos(ga0))-obs[ii])
                   +((y0 + (l1*cos(q1) + l2*cos(q1 + q2))*sin(ga0)))*((y0 + (l1*cos(q1) + l2*cos(q1 + q2))*sin(ga0)))
                   +((z0 + l1*sin(q1) + l2*sin(q1 + q2))-obs[ii]-0.2)*((z0 + l1*sin(q1) + l2*sin(q1 + q2))-obs[ii]-0.2)) >= ora*ora);

    ocp.subjectTo(((x0-obs[ii])*(x0-obs[ii])+(y0)*(y0)+(z0-obs[ii]-0.2)*(z0-obs[ii]-0.2)) >= ora*ora);
  }

  OCPexport mpc(ocp);

  ocp.subjectTo(x0 <= gx + 0.1); ocp.subjectTo(z0 <= gz + 0.5);

  mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
  mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
  mpc.set(INTEGRATOR_TYPE, INT_RK4);
  mpc.set(NUM_INTEGRATOR_STEPS, 200);

  mpc.set( QP_SOLVER, QP_QPOASES);
  mpc.set( GENERATE_TEST_FILE, YES);
  mpc.set( GENERATE_MAKE_FILE, YES);
  mpc.set( USE_SINGLE_PRECISION, YES);

  if (mpc.exportCode( "quad_arm_mpc_export" ) != SUCCESSFUL_RETURN)
    exit( EXIT_FAILURE );

  mpc.printDimensionsQP();

  return EXIT_SUCCESS;
}
