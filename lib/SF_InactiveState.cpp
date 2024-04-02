#include "SF_InactiveState.h"

/* Public Methods and Constructor */
SFInactiveState::SFInactiveState(string state_label, ros::ServiceClient &arming_service_client, 
                                ros::ServiceClient &set_flight_mode_client) : 
                                SFBaseState(state_label), arming_service_client(arming_service_client), 
                                set_flight_mode_client(set_flight_mode_client), start_position(Vector3d::Zero())
{
    // Add the neighbor states
    addNeighborState(string("Take-off"), [this]() { return defaultTransitionGuard(); });
}

void SFInactiveState::entry(const Vector3d &curr_position, const Vector3d &prev_desired_position, string &args)
{
    // Reset the desired state
    setDesiredState(getStateLabel());

    // Set start position
    start_position = curr_position;

    // Switch to Stabilize flight mode
    mavros_msgs::SetMode flight_mode_srv;
    flight_mode_srv.request.base_mode = 0;//flight_mode_srv.request.MAV_MODE_STABILIZE_ARMED;
    flight_mode_srv.request.custom_mode = string("STABILIZE"); //string("");
    uint16_t attempt_counter = 0;
    while(!(set_flight_mode_client.call(flight_mode_srv) && flight_mode_srv.response.mode_sent) && (attempt_counter < 10))
    {
        attempt_counter++;
        ROS_INFO("Failed to change flight mode, reattempting in 100ms");
        ros::Duration(0.1).sleep();
    }

    // If unsuccessful then wait for manual flight mode change
    if(attempt_counter == 10)
    {
        double wait_time = 15.0;
        ROS_INFO("Please change flight mode to STABILIZE manually");
        while(wait_time > 0)
        {
            ROS_INFO("Attemping to disarm in %.2f seconds", wait_time);
            wait_time -= 1.0;
            ros::Duration(1.0).sleep();
        }
    }
    // Mode change successful
    else
    {
        ROS_INFO("Changed flight mode successfully, active mode: STABILIZE");
        ROS_INFO("Disarming in %.2f seconds", 1.0);
        ros::Duration(1.0).sleep();
    }

    // Disarm the motors
    mavros_msgs::CommandBool arming_srv;
    arming_srv.request.value = false;
    attempt_counter = 0;
    while(!(arming_service_client.call(arming_srv) && arming_srv.response.success) && (attempt_counter < 20))
    {
        attempt_counter++;
        ROS_INFO("Failed to disarm, reattempting in 250ms");
        ros::Duration(0.25).sleep();
    }

    // Failed to disarm after all attempts
    if(attempt_counter == 20)
    {
        ROS_ERROR("Cannot disarm, please disarm manually!!");
    }
    // Disarmed successfully
    else
    {
        ROS_INFO("Disarmed successfully");
    }
}

/* Protected Methods */

Matrix3d SFInactiveState::during()
{
    // Construct the desired trajectory matrix
    Matrix3d desired_traj;
    Vector3d zero_vect = Vector3d::Zero();
    desired_traj << start_position, zero_vect, zero_vect;
    
    return desired_traj;
}

bool SFInactiveState::exit(string &exit_status)
{
    // Switch to GUIDED_NOGPS flight mode
    mavros_msgs::SetMode flight_mode_srv;
    flight_mode_srv.request.base_mode = 0;
    flight_mode_srv.request.custom_mode = string("GUIDED_NOGPS");
    uint16_t attempt_counter = 0;
    while(!(set_flight_mode_client.call(flight_mode_srv) && flight_mode_srv.response.mode_sent) && (attempt_counter < 10))
    {
        attempt_counter++;
        ROS_INFO("Failed to change flight mode, reattempting in 100ms");
        ros::Duration(0.1).sleep();
    }

    // If unsuccessful then wait for manual flight mode change
    if(attempt_counter == 10)
    {
        double wait_time = 20.0;
        ROS_INFO("Please change flight mode to GUIDED_NOGPS manually");
        while(wait_time > 0)
        {
            ROS_INFO("Attemping to arm in %.2f seconds", wait_time);
            wait_time -= 1.0;
            ros::Duration(1.0).sleep();
        }
    }
    // Mode change successful
    else
    {
        ROS_INFO("Changed flight mode successfully, active mode: GUIDED_NOGPS");
        ROS_INFO("Arming in %.2f seconds", 2.0);
        ros::Duration(2.0).sleep();
    }

    // Arm the motors
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    attempt_counter = 0;
    while(!(arming_service_client.call(srv) && srv.response.success) && (attempt_counter < 20))
    {
        attempt_counter++;
        srv.request.value = true;
        ROS_INFO("%s", "\nFailed to arm, reattempting in 2s");
        ros::Duration(2.0).sleep();
    }

    // Failed to Arm after all attempts
    if(attempt_counter == 20)
    {
        setDesiredState(getStateLabel());
        ROS_INFO("%s", "Cannot arm, state transition aborted!!");
        return false;
    }
    // Armed successfully
    else
    {   
        exit_status += string("\nArmed successfully");
        ros::Duration(2.0).sleep();
        return true;
    }
}