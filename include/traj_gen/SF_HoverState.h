#ifndef SF_HOVER_STATE
#define SF_HOVER_STATE

#include "SF_BaseState.h"

class SFHoverState : public SFBaseState
{
public:
    /* Public Methods and Constructor */
    SFHoverState(string state_label);
    void entry(const Vector3d &curr_position, const Vector3d &prev_desired_position, string &args) override;

protected:
    /* Protected Methods */
    Matrix3d during() override;

private:
    /* Private Class Members */
    Vector3d prev_desired_position;
};

#endif