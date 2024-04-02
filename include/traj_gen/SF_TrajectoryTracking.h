#ifndef SF_TRAJECTORY_TRACKING_STATE
#define SF_TRAJECTORY_TRACKING_STATE

#include "SF_BaseState.h"
#include <chrono>
#include <boost/algorithm/string.hpp>
// #include "traj_gen2/TrajGen.hpp"

// using namespace trajgen;

typedef struct helix_t
{
    double center_x;
    double center_y;
    double radius;
    double theta_init;
    double omega;
    double vel_z;
} helix_t;

typedef struct helix_varying_t
{
    double center_x;
    double center_y;
    double radius;
    double omega_min;
    double omega_max;
    double vel_z_acc;
    double vel_z_dec;
    double time_acc;
    double time_hold;
    double time_dec;
    double theta_f1;
    double theta_f2;
} helix_varying_t;

class SFTrajectoryTrackingState : public SFBaseState
{
public:
    /* Public Methods and Constructor */
    SFTrajectoryTrackingState(string state_label);
    ~SFTrajectoryTrackingState();
    void entry(const Vector3d &curr_position, const Vector3d &prev_desired_position, string &args) override;

protected:
    /* Protected Methods */
    Matrix3d during() override;

private:
    /* Private Methods */
    void helixTrajInit(const vector<string> &args);
    void helixVaryingTrajInit(const vector<string> &args);
    void minSnapTrajInit(const vector<string> &args);
    Matrix3d calcHelixTraj(double delta_t, bool trajectory_completed);
    Matrix3d calcHelixVaryingTraj(double delta_t, bool trajectory_completed);
    Matrix3d calcMinSnapTraj(double delta_t);
    // void deleteWaypoints();

    /* Private Class Members */
    std::chrono::time_point<std::chrono::steady_clock> t_init;
    double delta_t_final;
    Vector3d start_position;
    string trajectory_type;
    // std::unique_ptr<PolyTrajGen<double, 3>> min_snap_traj_ptr;
    // vector<Pin<double, 3>*> waypoints_pin_set;
    helix_t helix_traj;
    helix_varying_t helix_var_traj;
};

#endif