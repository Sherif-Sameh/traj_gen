#ifndef SF_INACTIVE_STATE
#define SF_INACTIVE_STATE

#include "SF_BaseState.h"
#include "ros/ros.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"

class SFInactiveState : public SFBaseState
{
public:
    /* Public Methods and Constructor */
    SFInactiveState(string state_label, ros::ServiceClient &arming_service_client, ros::ServiceClient &set_flight_mode_client);
    void entry(const Vector3d &curr_position, const Vector3d &prev_desired_position, string &args) override;

protected:
    /* Protected Methods */

    Matrix3d during() override;
    bool exit(string &exit_status) override;

private:
    /* Private Class Members */
    Vector3d start_position;
    ros::ServiceClient arming_service_client;
    ros::ServiceClient set_flight_mode_client;;
};

#endif