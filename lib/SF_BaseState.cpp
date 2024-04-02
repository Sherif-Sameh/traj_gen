#include "SF_BaseState.h"

/* Public Methods and Constructor */

SFBaseState::SFBaseState(string state_label) : state_label(state_label), desired_state_label(state_label)
{

}

string SFBaseState::getStateLabel()
{
    return state_label;
}

void SFBaseState::setDesiredState(string desired_state)
{
    desired_state_label = desired_state;
}

void SFBaseState::addNeighborState(string state_label, transition_guard_func_t transition_guard_ptr)
{
    state_tuple added_state(state_label, transition_guard_ptr);
    neighbor_states.push_back(added_state);
}

bool SFBaseState::runIteration(Matrix3d &desired_traj, string &pending_state, string &status)
{
    pending_state = status = string("");
    bool transition_pending = isTransitionPending(pending_state, status);
    if(transition_pending && exit(status))
    {
        return true;
    }

    desired_traj = during();
    
    return false;
}

// tuple(current_postion, prev_desired_position, input_args)
void SFBaseState::entry(const Vector3d &curr_position, const Vector3d &prev_desired_position, string &args)
{
    return;
}

/* Protected Methods */

Matrix3d SFBaseState::during()
{
    return Matrix3d::Zero();
}

bool SFBaseState::exit(string &exit_status)
{
    exit_status += string("\nExit successful");
    return true;
}

bool SFBaseState::defaultTransitionGuard() 
{
    return true;
}

/* Private Methods */

bool SFBaseState::isTransitionPending(string &pending_state, string &status)
{
    if(desired_state_label != state_label)
    {
        for(const state_tuple &tuple : neighbor_states)
        {
            if(std::get<0>(tuple) == desired_state_label)
            {
                if((std::get<1>(tuple))())
                {
                    pending_state = desired_state_label;
                    status += string("Transitioning to ") + desired_state_label;
                    return true;
                }
                else
                {
                    desired_state_label = state_label;
                    return false;
                }
            }
        }
    }
    return false;
}