#include "SF_LandingState.h"

/* Public Methods and Constructor */
SFLandingState::SFLandingState(string state_label, double min_abort_landing_height, double min_landing_height, double max_landing_rate) : 
                SFBaseState(state_label), min_abort_landing_height(min_abort_landing_height), max_landing_rate(max_landing_rate),
                min_landing_height(min_landing_height)
{
    // Add the neighbor states
    addNeighborState(string("Inactive"), [this]() { return transitionGuardInactive(); });
    addNeighborState(string("Hover"), [this]() { return transitionGuardHover(); });
}

void SFLandingState::entry(const Vector3d &curr_position, const Vector3d &prev_desired_position, string &args)
{
    // Set desired state to the current state
    setDesiredState(getStateLabel());

    // Set the current starting position
    start_position = prev_desired_position;

    // Split the input arguments into a vector of strings splitting at whitespaces
    vector<string> split_args;
    boost::split(split_args, args, boost::is_any_of(" "));

    // Parse the desired landing height (arg 1) and time(seconds) (arg 2), Note: arg 0 is the state label
    if(split_args.size() >= 3)
    {
        desired_landing_height = std::stod(split_args[1]);
        if(desired_landing_height < min_landing_height)
        {
            desired_landing_height = min_landing_height;
        }
        desired_landing_rate = (start_position(2) - desired_landing_height) / std::stod(split_args[2]);

        if(desired_landing_rate > max_landing_rate)
        {
            desired_landing_height = max_landing_rate;
        }
    }
    else
    {
        setDesiredState(string("Hover"));
    }

    // Intialize the start time
    t_init = std::chrono::steady_clock::now();
}

/* Protected Methods */
Matrix3d SFLandingState::during()
{
    // Calculate the current desired height
    std::chrono::duration<double> elapsed_time = std::chrono::steady_clock::now() - t_init;
    double current_desired_height = start_position(2) - desired_landing_rate * elapsed_time.count();

    // If you have reached the desired landing height then update the desired state to inactive
    bool trajectory_completed = false;
    if(current_desired_height <= desired_landing_height)
    {
        current_desired_height = desired_landing_height;
        trajectory_completed = true;
        setDesiredState(string("Inactive"));
    }

    // Construct the desired trajectory matrix
    Matrix3d desired_traj;
    Vector3d desired_pos = Vector3d(start_position(0), start_position(1), current_desired_height);
    Vector3d desired_vel = Vector3d(0.0, 0.0, (!trajectory_completed) ? -desired_landing_rate : 0.0);
    desired_traj << desired_pos, desired_vel, Vector3d::Zero();
    
    return desired_traj;
}

bool SFLandingState::transitionGuardInactive()
{
    std::chrono::duration<double> elapsed_time = std::chrono::steady_clock::now() - t_init;
    if((start_position(2) - desired_landing_rate * elapsed_time.count()) > desired_landing_height)
    {
        return false;
    }
    return true;
}

bool SFLandingState::transitionGuardHover()
{
    std::chrono::duration<double> elapsed_time = std::chrono::steady_clock::now() - t_init;
    if((start_position(2) - desired_landing_rate * elapsed_time.count()) < min_abort_landing_height)
    {
        return false;
    }
    return true;
}