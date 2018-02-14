#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>

#include "acado_common.h"
#include "acado_auxiliary_functions.h"

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */
#define NUM_OBS     3
#define NUM_STEPS     50

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

/* A template for testing of the solver. */
int main(int argc, char** argv) {
  ros::init(argc, argv, "quad_sim");
  ros::NodeHandle nh;
  int    i, iter;
  acado_timer t;

  /* Initialize the solver. */
  acado_initializeSolver();

  /* Initialize the states and controls. */
  for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
  for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;

  /* Initialize the measurements/reference. */
  for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
  for (i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = 3.5;

  /* MPC: initialize the current state feedback. */
#if ACADO_INITIAL_STATE_FIXED
  for (i = 0; i < NX; ++i) acadoVariables.x0[ i ] = 0.01;
#endif

  /* Prepare first step */
  acado_preparationStep();

  /* Get the time before start of the loop. */
  acado_tic( &t );

  /* The "real-time iterations" loop. */
  for(iter = 0; iter < NUM_STEPS; ++iter) {
        /* Perform the feedback step. */
    acado_feedbackStep( );

    /* Prepare for the next step. */
    acado_preparationStep();
  }
  /* Read the elapsed time. */
  real_t te = acado_toc( &t );
  printf("\n\n time:   %.3g seconds\n\n", te);

  ros::Duration(1.0).sleep();
  tf::TransformBroadcaster br;
  ros::Publisher jointPub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
  ros::Publisher goalPub = nh.advertise<visualization_msgs::Marker>("/goal_marker", 2);
  ros::Publisher obsPub = nh.advertise<visualization_msgs::Marker>("/obs_marker", 2);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 3.5;
  marker.pose.position.y = 3.5;
  marker.pose.position.z = 3.5 - 0.0875;
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

  double obs[] = {2.0, 2.5, 3.0};
  visualization_msgs::Marker obs_marker[NUM_OBS];

  for(int o = 0; o < NUM_OBS; o++) {
    obs_marker[o].header.frame_id = "world";
    obs_marker[o].id = o;
    obs_marker[o].type = visualization_msgs::Marker::SPHERE;
    obs_marker[o].action = visualization_msgs::Marker::ADD;
    obs_marker[o].pose.position.x = obs[o];
    obs_marker[o].pose.position.y = obs[o]+0.2;
    obs_marker[o].pose.position.z = obs[o]+0.2;
    obs_marker[o].pose.orientation.x = 0.0;
    obs_marker[o].pose.orientation.y = 0.0;
    obs_marker[o].pose.orientation.z = 0.0;
    obs_marker[o].pose.orientation.w = 1.0;
    obs_marker[o].scale.x = 0.1;
    obs_marker[o].scale.y = 0.1;
    obs_marker[o].scale.z = 0.1;
    obs_marker[o].color.a = 1.0; 
    obs_marker[o].color.r = 0.0;
    obs_marker[o].color.g = 1.0;
    obs_marker[o].color.b = 0.0;
  }

  for(int tt=0; tt < N+1; tt++) {
    tf::Transform transform = tf::Transform(
     tf::createQuaternionFromRPY(0,0,acadoVariables.x[NX*tt + 12]), 
     tf::Vector3(acadoVariables.x[NX*tt],
                 acadoVariables.x[NX*tt + 1],
                 acadoVariables.x[NX*tt + 2]));

    ROS_INFO("velocity: %f %f %f %f", acadoVariables.x[NX*tt+3],
      acadoVariables.x[NX*tt+4], acadoVariables.x[NX*tt+5]);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "baselink"));
    marker.header.stamp = ros::Time::now();
    goalPub.publish(marker);
    for(int o = 0; o < NUM_OBS; o++) {
      obs_marker[o].header.stamp = ros::Time::now();
      obsPub.publish(obs_marker[o]);
    }
    ros::Duration(1.0).sleep();
  }
  return 0;
}
