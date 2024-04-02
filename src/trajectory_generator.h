#ifndef TRAJECTORY_GENERATOR
#define TRAJECTORY_GENERATOR

#include <cmath>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>

#include "SF_InactiveState.h"
#include "SF_TakeOffState.h"
#include "SF_HoverState.h"
#include "SF_TrajectoryTracking.h"
#include "SF_LandingState.h"

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "traj_gen/DroneTrajectory.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "std_srvs/Empty.h"
#include "traj_gen/StringSrv.h"

using std::vector;
using std::string;
using Eigen::Vector3d;
using Eigen::Matrix3d;

class TrajectoryGenerator
{
public:
    /* Constructor and Public Methods */
    TrajectoryGenerator(ros::NodeHandle &nh);
    void callback_pose(const geometry_msgs::PoseStamped::ConstPtr &msg);
    bool callback_abort_mission(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
    bool callback_set_mode(traj_gen::StringSrv::Request &req, traj_gen::StringSrv::Response &resp);
    void spin();

private:
    /* Private Methods */
    void changeActiveState(string desired_state);

    /* Class Members */

    // Input parameters - configuration
    string pose_topic_name;
    string arming_srv_name;
    string set_flight_mode_srv_name;
    double min_takeoff_height;
    double max_takeoff_height;
    double max_takeoff_rate;
    double min_landing_height;
    double max_landing_rate;
    double trajectory_publish_rate;
    bool yaw_tracking_enable;
    
    // Pose and generated trajectory related
    Vector3d position_latest;
    Matrix3d prev_trajectory;

    // StateFlow related
    vector<std::unique_ptr<SFBaseState>> sf_states;
    size_t sf_active_state_index;
    string input_args;

    // ROS related
    ros::Publisher traj_pub;
    ros::Subscriber pose_sub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_flight_mode_client;
    ros::ServiceServer abort_mission_srv;
    ros::ServiceServer set_mode_srv;
};

#endif