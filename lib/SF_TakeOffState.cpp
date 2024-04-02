#include "SF_TakeOffState.h"

/* Public Methods and Constructor */
SFTakeOffState::SFTakeOffState(string state_label, double min_takeoff_height, double max_takeoff_height, double max_takeoff_rate) : 
                SFBaseState(state_label), min_takeoff_height(min_takeoff_height), 
                max_takeoff_height(max_takeoff_height), max_takeoff_rate(max_takeoff_rate)
{
    // Add the neighbor states
    addNeighborState(string("Hover"), [this]() { return transitionGuardHover(); });
}

void SFTakeOffState::entry(const Vector3d &curr_position, const Vector3d &prev_desired_position, string &args)
{
    // Set desired state to the current state
    setDesiredState(getStateLabel());

    // Set the current starting position
    start_position = curr_position;

    // Split the input arguments into a vector of strings splitting at whitespaces
    vector<string> split_args;
    boost::split(split_args, args, boost::is_any_of(" "));

    // Parse the desired take-off height (arg 1) and time(seconds) (arg 2), Note: arg 0 is the state label
    if(split_args.size() >= 3)
    {
        desired_takeoff_height = std::stod(split_args[1]);
        desired_takeoff_rate = (desired_takeoff_height / std::stod(split_args[2]));
        
        // Enforce max and min limits on both take-off height and rate
        if(desired_takeoff_height > max_takeoff_height)
        {
            desired_takeoff_height = max_takeoff_height;
        }
        else if(desired_takeoff_height < min_takeoff_height)
        {
            desired_takeoff_height = min_takeoff_height;
        }

        if(desired_takeoff_rate > max_takeoff_rate)
        {
            desired_takeoff_rate = max_takeoff_rate;
        }
    }
    else
    {
        desired_takeoff_height = 0.5 * max_takeoff_height;
        desired_takeoff_rate = 0.5 * max_takeoff_rate;
    }

    // Intialize the start time
    t_init = std::chrono::steady_clock::now();
}

/* Protected Methods */
Matrix3d SFTakeOffState::during()
{
    // Calculate the current desired height
    std::chrono::duration<double> elapsed_time = std::chrono::steady_clock::now() - t_init;
    double current_delta_height = desired_takeoff_rate * elapsed_time.count();
    
    // If you have reached the desired takeoff height then update the desired state to go to hovering
    bool trajectory_completed = false;
    if(current_delta_height >= desired_takeoff_height)
    {
        current_delta_height = desired_takeoff_height;
        trajectory_completed = true;
        setDesiredState(string("Hover"));
    }

    // Construct the desired trajectory matrix
    Matrix3d desired_traj;
    Vector3d desired_pos = Vector3d(start_position(0), start_position(1), start_position(2) + current_delta_height);
    Vector3d desired_vel = Vector3d(0.0, 0.0, (!trajectory_completed) ? desired_takeoff_rate : 0.0);
    desired_traj << desired_pos, desired_vel, Vector3d::Zero();
    
    return desired_traj;
}

bool SFTakeOffState::transitionGuardHover()
{
    std::chrono::duration<double> elapsed_time = std::chrono::steady_clock::now() - t_init;
    if((start_position(2) + desired_takeoff_rate*elapsed_time.count()) >= min_takeoff_height)
    {
        return true;
    }
    else
    {
        return false;
    }
}