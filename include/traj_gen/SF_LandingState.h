#ifndef SF_LANDING_STATE
#define SF_LANDING_STATE

#include "SF_BaseState.h"
#include <chrono>
#include <boost/algorithm/string.hpp>

class SFLandingState : public SFBaseState
{
public:
    /* Public Methods and Constructor */
    SFLandingState(string state_label, double min_abort_landing_height, double min_landing_height, double max_landing_rate);
    void entry(const Vector3d &curr_position, const Vector3d &prev_desired_position, string &args) override;

protected:
    /* Protected Methods */
    Matrix3d during() override;
    bool transitionGuardInactive();
    bool transitionGuardHover();

private:
    /* Private Class Members */
    std::chrono::time_point<std::chrono::steady_clock> t_init;
    double min_abort_landing_height;
    double min_landing_height;
    double max_landing_rate;
    double desired_landing_height;
    double desired_landing_rate;
    Vector3d start_position;
};

#endif