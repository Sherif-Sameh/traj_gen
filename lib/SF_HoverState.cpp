#include "SF_HoverState.h"

/* Public Methods and Constructor */
SFHoverState::SFHoverState(string state_label) : SFBaseState(state_label)
{
    // Add the neighbor states
    addNeighborState(string("Landing"), [this]() { return defaultTransitionGuard(); });
    addNeighborState(string("Trajectory_Tracking"), [this]() { return defaultTransitionGuard(); });
}

void SFHoverState::entry(const Vector3d &curr_position, const Vector3d &prev_desired_position, string &args)
{
    // Set desired state to the current state
    setDesiredState(getStateLabel());

    // Set the last generated desired position
    this->prev_desired_position = prev_desired_position;
}

/* Protected Methods */
Matrix3d SFHoverState::during()
{
    // Construct the desired trajectory matrix
    Matrix3d desired_traj;
    Vector3d zero_vect = Vector3d::Zero();
    desired_traj << prev_desired_position, zero_vect, zero_vect;
    
    return desired_traj;
}