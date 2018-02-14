#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/tf_eigen.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>

#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#include <Eigen/Dense>

using namespace Eigen;

#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */
#define NUM_OBS   3
#define NUM_STEPS 50

void flat2input(Vector3d y2, double yaw, tf::Matrix3x3 &quadTf) {
  Vector3d g(0.0, 0.0, -9,81);
  Vector3d acc = (y2 - g);

  double T = norm(acc);

  Vector3d Rz = acc/T;
  Vector3d rot(cos(yaw), sin(yaw), 0.0);

  Vector3d Ry = Rz.cross(rot)/(norm(Rz.cross(rot)));

  Vector3d Rx = Ry.cross(Rz);

  Matrix3d R;
  R << Rx(0), Ry(0), Rz(0),
       Rx(1), Ry(1), Rz(1),
       Rx(2), Ry(2), Rz(2);

  tf::matrixEigenToTF(R, quadTf);
}

void 

