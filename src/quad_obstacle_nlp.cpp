#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>

USING_NAMESPACE_ACADO

int main() {
  int N = 50;
  double te = 5.0;
  double h = te/N;

  Parameter x0[N], y0[N], z0[N],
            x1[N], y1[N], z1[N],
            x2[N], y2[N], z2[N],
            x3[N], y3[N], z3[N],
            ga0[N], ga1[N];

  Parameter x4[N], y4[N], z4[N], ga2[N];

  double gx = 3.0; double gy = 3.0; double gz = 3.0;

  Function cost;
  cost << (x0[N-1] - gx) << (y0[N-1]-gy) << (z0[N-1]-gz);
  for(int i=0; i < N; i++)
    cost << x4[i] << y4[i] << z4[i] << ga2[i];

  NLP nlp;
  nlp.minimizeLSQ(cost);

  nlp.subjectTo(x0[0] == 0.0);
  nlp.subjectTo(y0[0] == 0.0);
  nlp.subjectTo(z0[0] == 0.0);
  nlp.subjectTo(ga0[0] == 0.0);

  nlp.subjectTo(x1[0] == 0.0);
  nlp.subjectTo(y1[0] == 0.0);
  nlp.subjectTo(z1[0] == 0.0);
  nlp.subjectTo(ga1[0] == 0.0);

  nlp.subjectTo(x4[0] == 0.0); 
  nlp.subjectTo(y4[0] == 0.0);
  nlp.subjectTo(z4[0] == 0.0);

  for(int i=1; i < N; i++) {
    nlp.subjectTo((x0[i] - x0[i-1] - 0.5*h*(x1[i] + x1[i-1])) == 0);
    nlp.subjectTo((x1[i] - x1[i-1] - 0.5*h*(x2[i] + x2[i-1])) == 0);
    nlp.subjectTo((x2[i] - x2[i-1] - 0.5*h*(x3[i] + x3[i-1])) == 0);
    nlp.subjectTo((x3[i] - x3[i-1] - 0.5*h*(x4[i] + x4[i-1])) == 0);

    nlp.subjectTo((y0[i] - y0[i-1] - 0.5*h*(y1[i] + y1[i-1])) == 0);
    nlp.subjectTo((y1[i] - y1[i-1] - 0.5*h*(y2[i] + y2[i-1])) == 0);
    nlp.subjectTo((y2[i] - y2[i-1] - 0.5*h*(y3[i] + y3[i-1])) == 0);
    nlp.subjectTo((y3[i] - y3[i-1] - 0.5*h*(y4[i] + y4[i-1])) == 0);

    nlp.subjectTo((z0[i] - z0[i-1] - 0.5*h*(y1[i] + z1[i-1])) == 0);
    nlp.subjectTo((z1[i] - z1[i-1] - 0.5*h*(y2[i] + z2[i-1])) == 0);
    nlp.subjectTo((z2[i] - z2[i-1] - 0.5*h*(y3[i] + z3[i-1])) == 0);
    nlp.subjectTo((z3[i] - z3[i-1] - 0.5*h*(y4[i] + z4[i-1])) == 0);

    nlp.subjectTo((ga0[i] - ga0[i-1] - 0.5*h*(ga1[i] + ga1[i-1])) == 0);
    nlp.subjectTo((ga1[i] - ga1[i-1] - 0.5*h*(ga2[i] + ga2[i-1])) == 0);
  }

  nlp.subjectTo(x1[N-1] == 0); nlp.subjectTo(y1[N-1] == 0); nlp.subjectTo(z1[N-1] == 0);
  nlp.subjectTo(ga1[N-1] == 0);

  OptimizationAlgorithm algorithm(nlp);
  algorithm.solve();

  algorithm.set(KKT_TOLERANCE, 1e-1);
  return 0;
}