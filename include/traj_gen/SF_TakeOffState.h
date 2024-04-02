#ifndef SF_TAKEOFF_STATE
#define SF_TAKEOFF_STATE

#include "SF_BaseState.h"
#include <chrono>
#include <boost/algorithm/string.hpp>

class SFTakeOffState : public SFBaseState
{
public:
    /* Public Methods and Constructor */
    SFTakeOffState(string state_label, double min_takeoff_height, double max_takeoff_height, double max_takeoff_rate);
    void entry(const Vector3d &curr_position, const Vector3d &prev_desired_position, string &args) override;

protected:
    /* Protected Methods */
    Matrix3d during() override;
    bool transitionGuardHover();

private:
    /* Private Class Members */
    std::chrono::time_point<std::chrono::steady_clock> t_init;
    double min_takeoff_height;
    double max_takeoff_height;
    double max_takeoff_rate;
    double desired_takeoff_height;
    double desired_takeoff_rate;
    Vector3d start_position;
};

#endif