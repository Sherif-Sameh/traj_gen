# traj_gen
Trajectory Generator for Quadrotor UAVs using ArduCopter

## Trajectory Generator

### Overview
The **Trajectory Generator** node implements a state machine composed of 5 distinct states for generating different trajectories and handling the different requirements of  transitioning between the different states such as arming or disarming motors. The implementation is based on MatLab StateFlow and uses user-triggered events to trigger some transitions between the different states. The five states are labelled as follows:

* Inactive
* Take-off
* Hover
* Trajectory_Tracking
* Landing

**Note:** Only transiitons to the Take-off, Trajectory_Tracking and Landing states must be triggered by the user through the appropriate ROS Service, other transitions are automatically triggered once their transition guards are bypassed.

### Advertised Topics
1. **Name:** `/trajectory_generator/trajectory` **Type:** test_package/DroneTrajectory, **Description:** Publish desired trajectory which includes: position, linear velocity, linear acceleation and desired yaw angle.

### Advertised Services
1. **Name:** `/trajectory_generator/set_mode`, **Type:** std_srvs/Empty, **Description:** Used to request mode transitions from the state-machine.
2. **Name:** `/trajectory_generator/abort_mission` **Type:** test_package/StringSrv, **Description:** Used to request cancellation of the current mission (Taking-off, Traj. Tracking or Landing).

### Subscribed Topics
1. **Name:** `/iris_demo/pose`, **Type:** geometry_msgs/PoseStamped, **Description:** Get current position.

### Service Clients
1. **Name:** `/mavros/cmd/arming`, **Type:** mavros_msgs/CommandBool, **Description:** Request arming or disarming of motors.
2. **Name:** `/mavros/set_mode`, **Type:** mavros_msgs/SetMode, **Description:** Request mode change from Ardupilot (Stabilize, Guided_NOGPS, etc.).

### Additional Information

The available "set_mode" ROS service can be used from the CLI using the ROSService tool where the only input argument to the service is a string which is parsed and interpreted internally by the different modes. 

To take-off:

* rosservice call /trajectory_generator/set_mode "Take-off Delta_z (m) Duration (sec)"

For example: `rosservice call /trajectory_generator/set_mode "Take-off 0.5 5.0"` would request moving to Z = Z_initial + 0.5 (m) with the total maneuver taking 5.0 seconds.

To land:

* rosservice call /trajectory_generator/set_mode "Landing Final_Z(m) Duration (sec)"

For example: `rosservice call /trajectory_generator/set_mode "Landing 0.1 8.0"` would request moving to Z = 0.1 (m) with the total maneuver taking 8.0 seconds.

To track a trajectory:

* rosservice call /trajectory_generator/set_mode "Trajectory_Tracking helical Helix_radius (m) Theta_init (rad) Delta_theta (rad) Delta_Z (m) Duration (sec)"

For example: `rosservice call /trajectory_generator/set_mode "Trajectory_Tracking helical 1.0 0.0 6.2832 0.5 7.5"` would request to track a helical trajectory whose radius equals to 1.0m, where the drone is initially situated on the helix at theta = 0.0 rad, the total rotation is equal to 1 full rotation (2*Pi), the final Z = Z_curr + 0.5 (m) and the whole manuever taking 7.5 seconds.

**OR**

* rosservice call /trajectory_generator/set_mode "Trajectory_Tracking min_snap Num_of_waypoints X1 Y1 Z1 X2 Y2 Z2 .... XN YN ZN T1 T2 .... TN"

For example: `rosservice call /trajectory_generator/set_mode "Trajectory_Tracking min_snap 3 5.0 5.0 5.0 8.0 2.0 7.0 3.0 0.0 3.0 5.0 10.0 15.0"` would request to track the minimum snap trajectory which starts from the current position and passes though the 3 defined waypoints: P1(5.0,5.0,5.0), P2(8.0,2.0,7.0) and P3(3.0,0.0,3.0). Morever, the trajectory must equal those waypoints at their repsective timestamps T1=5.0s, T2=10.0s and T3=15.0s, where this elapsed time is calculated from the time the trajectory is initiated. Lastly, all derivatives whose order is 1 or greater must be equal to 0.0 at the two endpoints (the starting position and the final waypoint).

**Note 1:** Minimum snap trajectories are calculated using the C++ library available at the following repository: https://github.com/icsl-Jeon/traj_gen.git

**Note 2:** This feature is currently not available when testing on the real drone as the minimum snap library requires C++14 at least, which is not supported by the g++ compiler available on the raspberry pi. The version of the gcc and g++ compilers available is 8.3 which **does not** support C++14 or newer.
