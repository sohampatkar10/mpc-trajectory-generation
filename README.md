# Quad Arm Trajectory Generation
ROS Wrapper on ACADO for obstacle-free trajectory generation for a quadrotor-arm system

## 1. ACADO

-Install ACADO using instructions listed on http://acado.github.io/install_linux.html

## 2. Package install
Execute "catkin build" or "catkin_make" from your ROS Workspace

## 3. Running examples
There are 2 example scenarios implemented in the current form:
1. quad_example.launch : System navigates through 2 poles to reach goal state.
2. quad_place.launch : System starts with arm under the table and needs to reach a goal position above the table.

Make sure to change the log directory parameters (states_file, controls_file) in the launch files.

## Config parameters
The params.yaml file contains parameters for quad_example.launch and the place_params.yaml file contains parameters for quad_place.launch.


