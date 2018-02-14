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
                    ga0, ga1;

  Control x4, y4, z4, ga2;
  double ts = 0.0;
  double te = 5.0;
  int numSteps = 10;
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

  DMatrix Q(4, 4); Q.setIdentity();
  Q(0,0) = 0.01; Q(1,1) = 0.01; Q(2,2) = 0.01;
  Function eta;
  eta << x4 << y4 << z4 << ga2;

  DMatrix W(3,3); W.setIdentity();
  W(0,0) = 10.0; W(1,1) = 10.0; W(2,2) = 10.0;

  Function phi;
  phi << x0;
  phi << y0;
  phi << z0;

  OCP ocp(ts, te, numSteps);

  ocp.minimizeLSQ(Q, eta); // Trajectory Cost
  ocp.minimizeLSQEndTerm(W, phi); // Terminal Cost

  ocp.subjectTo(f); // Dynamics

  ocp.subjectTo(AT_START, x0 == 0.0);
  ocp.subjectTo(AT_START, y0 == 0.0);
  ocp.subjectTo(AT_START, z0 == 0.0);
  ocp.subjectTo(AT_START, ga0 == 0.0);

  ocp.subjectTo(AT_START, x1 == 0.0);
  ocp.subjectTo(AT_START, y1 == 0.0);
  ocp.subjectTo(AT_START, z1 == 0.0);
  ocp.subjectTo(AT_START, ga1 == 0.0);

  ocp.subjectTo(AT_END, x1 == 0.0);
  ocp.subjectTo(AT_END, y1 == 0.0);
  ocp.subjectTo(AT_END, z1 == 0.0);
  ocp.subjectTo(AT_END, ga1 == 0.0);

  double obs[5] = {1.5, 2.0, 2.5, 3.0};
  double ora = 0.3;
  for(int ii = 0; ii < 4; ii++)
    ocp.subjectTo(((x0-obs[ii])*(x0-obs[ii])+(y0-obs[ii]-0.2)*(y0-obs[ii]-0.2)+(z0-obs[ii]-0.2)*(z0-obs[ii]-0.2)) >= ora*ora);

  /**
  * Code Generation
  */
  OCPexport mpc(ocp);

  mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
  mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
  mpc.set(INTEGRATOR_TYPE, INT_RK4);
  mpc.set(NUM_INTEGRATOR_STEPS, 100);

  mpc.set( QP_SOLVER, QP_QPOASES);
  mpc.set( GENERATE_TEST_FILE, YES);
  mpc.set( GENERATE_MAKE_FILE, YES);
  mpc.set( USE_SINGLE_PRECISION, YES);

  if (mpc.exportCode( "quad_mpc_export" ) != SUCCESSFUL_RETURN)
    exit( EXIT_FAILURE );

  mpc.printDimensionsQP();

  return EXIT_SUCCESS;
}
