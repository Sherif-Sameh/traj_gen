#include "trajectory_generator.h"

/* Constructor and Public Methods */
TrajectoryGenerator::TrajectoryGenerator(ros::NodeHandle &nh) : sf_active_state_index(0)
{
    // Receive input parameters from the parameter server
    nh.param("/trajectory_generator/pose_topic_name", pose_topic_name, string("pose"));
    nh.param("/trajectory_generator/arming_srv_name", arming_srv_name, string("/mavros/cmd/arming"));
    nh.param("/trajectory_generator/set_flight_mode_srv_name", set_flight_mode_srv_name, string("/mavros/set_mode"));
    nh.param("/trajectory_generator/min_takeoff_height", min_takeoff_height, 0.5);
    nh.param("/trajectory_generator/max_takeoff_height", max_takeoff_height, 1.5);
    nh.param("/trajectory_generator/max_takeoff_rate", max_takeoff_rate, 0.2);
    nh.param("/trajectory_generator/min_landing_height", min_landing_height, 0.15);
    nh.param("/trajectory_generator/max_landing_rate", max_landing_rate, 0.1);
    nh.param("/trajectory_generator/trajectory_publish_rate", trajectory_publish_rate, 10.0);
    nh.param("/trajectory_generator/yaw_tracking_enable", yaw_tracking_enable, false);

    // Report values of input parameters
    ROS_INFO("Pose topic name: %s", pose_topic_name.c_str());
    ROS_INFO("Arming service name: %s", arming_srv_name.c_str());
    ROS_INFO("Set flight mode service name: %s", set_flight_mode_srv_name.c_str());;
    ROS_INFO("Min take-off height: %.4fm", min_takeoff_height);
    ROS_INFO("Max take-off height: %.4fm", max_takeoff_height);
    ROS_INFO("Max take-off rate: %.4fm/s", max_takeoff_rate);
    ROS_INFO("Min landing height: %.4fm", min_landing_height);
    ROS_INFO("Max landing rate: %.4fm/s", max_landing_rate);
    ROS_INFO("Trajectory publishing rate: %.4fHz", trajectory_publish_rate);

    // Initialize non-ROS class members
    position_latest = Vector3d::Zero();
    prev_trajectory = Matrix3d::Zero();

    // Initialize ROS publishers, subscribers, service servers and service clients
    traj_pub = nh.advertise<traj_gen::DroneTrajectory>("/trajectory_generator/trajectory", 1, true);
    pose_sub = nh.subscribe(pose_topic_name, 3, &TrajectoryGenerator::callback_pose, this);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>(arming_srv_name);
    set_flight_mode_client = nh.serviceClient<mavros_msgs::SetMode>(set_flight_mode_srv_name);
    abort_mission_srv = nh.advertiseService("/trajectory_generator/abort_mission", &TrajectoryGenerator::callback_abort_mission, this);
    set_mode_srv = nh.advertiseService("/trajectory_generator/set_mode", &TrajectoryGenerator::callback_set_mode, this);

    sf_states.push_back(std::make_unique<SFInactiveState>(string("Inactive"), arming_client, set_flight_mode_client));
    sf_states.push_back(std::make_unique<SFTakeOffState>(string("Take-off"), min_takeoff_height, 
                                                        max_takeoff_height, max_takeoff_rate));
    sf_states.push_back(std::make_unique<SFHoverState>(string("Hover")));
    sf_states.push_back(std::make_unique<SFTrajectoryTrackingState>(string("Trajectory_Tracking")));
    sf_states.push_back(std::make_unique<SFLandingState>(string("Landing"), min_takeoff_height, 
                                                        min_landing_height, max_landing_rate));
}

void TrajectoryGenerator::callback_pose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Copy the position from geometry message to Eigen vector
    position_latest << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
}

bool TrajectoryGenerator::callback_abort_mission(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
    // Determine the appropriate action based on the current active state
    string sf_active_state_label = sf_states[sf_active_state_index]->getStateLabel();
    
    // There's nothing to abort in the inactive or hover states
    if((sf_active_state_label == string("Inactive")) || (sf_active_state_label == string("Hover")))
    {
        return false;
    }
    // For all remaining states, we'll attempt to go into the hover state
    else
    {
        sf_states[sf_active_state_index]->setDesiredState(string("Hover"));
        return true;
    }
}

bool TrajectoryGenerator::callback_set_mode(traj_gen::StringSrv::Request &req, traj_gen::StringSrv::Response &resp)
{
    // Store the input string for parsing later on
    input_args = req.input_arg;

    // Set the desired state according the given state label
    vector<string> split_args;
    boost::split(split_args, input_args, boost::is_any_of(" "));
    sf_states[sf_active_state_index]->setDesiredState(split_args[0]);

    return true;
}

void TrajectoryGenerator::spin()
{
    ros::Rate loop_rate(trajectory_publish_rate);

    while(ros::ok())
    {
        // Spin once to handle any waiting callbacks
        ros::spinOnce();

        // Run an iteration in the current state or transition to different state if needed
        string pending_state, status;
        bool transition_pending = sf_states[sf_active_state_index]->runIteration(prev_trajectory, pending_state, status);
        if(transition_pending)
        {
            changeActiveState(pending_state);
            ROS_INFO("%s", status.c_str());
        }

        // Publish new trajectory
        traj_gen::DroneTrajectory traj_msg;
        traj_msg.header.stamp = ros::Time::now();
        traj_msg.discrete_state = sf_states[sf_active_state_index]->getStateLabel();
        traj_msg.position.x = prev_trajectory(0, 0); traj_msg.velocity.x = prev_trajectory(0, 1); traj_msg.acceleration.x = prev_trajectory(0, 2);
        traj_msg.position.y = prev_trajectory(1, 0); traj_msg.velocity.y = prev_trajectory(1, 1); traj_msg.acceleration.y = prev_trajectory(1, 2);
        traj_msg.position.z = prev_trajectory(2, 0); traj_msg.velocity.z = prev_trajectory(2, 1); traj_msg.acceleration.z = prev_trajectory(2, 2);
        traj_msg.yaw = 0.0;
        
        traj_pub.publish(traj_msg);

        // Sleep till next iteration
        loop_rate.sleep();
    }
}

/* Private Methods */
void TrajectoryGenerator::changeActiveState(string desired_state)
{
    // Look for the state whose label matches with the desired state
    size_t num_of_states = sf_states.size();
    for(size_t i = 0; i < num_of_states; i++)
    {
        if(sf_states[i]->getStateLabel() == desired_state)
        {
            sf_active_state_index = i;
            ROS_INFO("Active state: %s", sf_states[i]->getStateLabel().c_str());
            break;
        }
    }

    // Call the entry method for the new state
    Vector3d prev_desired_position = prev_trajectory.col(0);
    sf_states[sf_active_state_index]->entry(position_latest, prev_desired_position, input_args);
}

/* End of class definition */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle nh;

    TrajectoryGenerator trajectory_generator(nh);
    trajectory_generator.spin();

    return 0;
}