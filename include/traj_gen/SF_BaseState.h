#ifndef SF_BASE_STATE
#define SF_BASE_STATE

#include <cmath>
#include <vector>
#include <string>
#include <tuple>
#include <functional>
#include <Eigen/Dense>

using std::vector;
using std::string;
using std::tuple;
using Eigen::Vector3d;
using Eigen::Matrix3d;

class SFBaseState
{
public:
    using transition_guard_func_t = std::function<bool()>;
    using state_tuple = tuple<string, transition_guard_func_t>;
    /* Public Methods and Constructor */
    
    SFBaseState(string state_label);
    string getStateLabel();
    void setDesiredState(string desired_state);
    void addNeighborState(string state_label, transition_guard_func_t transition_guard);
    bool runIteration(Matrix3d &current_desired_position, string &pending_state, string &status);
    virtual void entry(const Vector3d &curr_position, const Vector3d &prev_desired_position, string &args);

protected:
    /* Protected Methods */
    virtual Matrix3d during();
    virtual bool exit(string &exit_status);
    bool defaultTransitionGuard();

private:
    /* Private Methods */
    bool isTransitionPending(string &pending_state, string &status);

    /* Private Class Members */
    string state_label;
    string desired_state_label;
    vector<state_tuple> neighbor_states;
};

#endif