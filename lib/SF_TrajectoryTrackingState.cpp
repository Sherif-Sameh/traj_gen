#include "SF_TrajectoryTracking.h"

/* Public Methods, Constructor and Destructor */
SFTrajectoryTrackingState::SFTrajectoryTrackingState(string state_label) : SFBaseState(state_label)
{
    // Add the neighbor states
    addNeighborState(string("Hover"), [this]() { return defaultTransitionGuard(); });
}

SFTrajectoryTrackingState::~SFTrajectoryTrackingState()
{
    // deleteWaypoints();
}

void SFTrajectoryTrackingState::entry(const Vector3d &curr_position, const Vector3d &prev_desired_position, string &args)
{
    // Set desired state to the current state
    setDesiredState(getStateLabel());

    // Set the starting position
    start_position = prev_desired_position;

    // Split the input arguments into a vector of strings splitting at whitespaces
    vector<string> split_args;
    boost::split(split_args, args, boost::is_any_of(" "));

    // Check the type of trajectory requested and process the rest of its arguments
    trajectory_type = split_args[1];
    // if(trajectory_type == string("min_snap"))
    // {
    //     minSnapTrajInit(split_args);
    // }
    if(trajectory_type == string("helical_var"))
    {
        helixVaryingTrajInit(split_args);
    }
    else
    {
        helixTrajInit(split_args);
    }

    // Intialize the start time
    t_init = std::chrono::steady_clock::now();
}

/* Protected Methods */
Matrix3d SFTrajectoryTrackingState::during()
{
    // Calculate the elapsed time
    std::chrono::duration<double> elapsed_time = std::chrono::steady_clock::now() - t_init;
    double delta_t = elapsed_time.count();

    // Update desired state to hovering if trajectory is over
    bool trajectory_completed = false;
    if(delta_t >= delta_t_final)
    {
        setDesiredState(string("Hover"));
        delta_t = delta_t_final;
        trajectory_completed = true;
    }

    // Call the appropriate method based on the trajectory type
    // if(trajectory_type == string("min_snap"))
    // {
    //     return calcMinSnapTraj(delta_t);
    // }
    if(trajectory_type == string("helical_var"))
    {
        return calcHelixVaryingTraj(delta_t, trajectory_completed);
    }
    else
    {
        return calcHelixTraj(delta_t, trajectory_completed);
    }
}

/* Private Methods */
void SFTrajectoryTrackingState::helixTrajInit(const vector<string> &args)
{   
    // Parse all input arguments
    helix_traj.radius       = std::stod(args[2]);
    helix_traj.theta_init   = std::stod(args[3]);
    double delta_theta      = std::stod(args[4]);
    double delta_z          = std::stod(args[5]);
    delta_t_final           = std::stod(args[6]);
    
    // Calculating the remaining helix parameters
    helix_traj.center_x = start_position(0) - helix_traj.radius * cos(helix_traj.theta_init);
    helix_traj.center_y = start_position(1) - helix_traj.radius * sin(helix_traj.theta_init);
    helix_traj.omega = delta_theta / delta_t_final;
    helix_traj.vel_z = delta_z / delta_t_final;
}

void SFTrajectoryTrackingState::helixVaryingTrajInit(const vector<string> &args)
{
    // Parse all input arguments
    helix_var_traj.radius       = std::stod(args[2]);
    helix_var_traj.omega_min    = std::stod(args[3]);
    helix_var_traj.omega_max    = std::stod(args[4]);
    double delta_z              = std::stod(args[5]);
    helix_var_traj.time_acc     = std::stod(args[6]);
    helix_var_traj.time_hold    = std::stod(args[7]);
    helix_var_traj.time_dec     = std::stod(args[8]);

    // Calculate the remaining parameters of the trajectory
    helix_var_traj.center_x = start_position(0) - helix_var_traj.radius;
    helix_var_traj.center_y = start_position(1);
    helix_var_traj.vel_z_acc = delta_z / helix_var_traj.time_acc;
    helix_var_traj.vel_z_dec = -delta_z / helix_var_traj.time_dec;
    double delta_omega = helix_var_traj.omega_max - helix_var_traj.omega_min;
    helix_var_traj.theta_f1 = (helix_var_traj.omega_min + 0.5 * delta_omega) * helix_var_traj.time_acc;
    helix_var_traj.theta_f2 = helix_var_traj.omega_max * helix_var_traj.time_hold + helix_var_traj.theta_f1;
    delta_t_final = helix_var_traj.time_acc + helix_var_traj.time_hold + helix_var_traj.time_dec;
}

// void SFTrajectoryTrackingState::minSnapTrajInit(const vector<string> &args)
// {
//     // Parse the number of waypoints and initialize vectors to store inputs
//     size_t num_of_waypoints = static_cast<size_t>(std::stoi(args[2]) + 1);
//     vector<Vector3d> waypoints(num_of_waypoints);
//     time_knots<double> ts(num_of_waypoints);

//     // Parse the waypoints
//     waypoints[0] = start_position;
//     for(size_t i = 1; i < num_of_waypoints; i++)
//     {
//         waypoints[i] = Vector3d(std::stod(args[3 + 3 * (i - 1)]),
//                                 std::stod(args[4 + 3 * (i - 1)]),
//                                 std::stod(args[5 + 3 * (i - 1)]));
//     }
    
//     // Parse the time at each waypoint
//     size_t first_time_index = 3 + 3 * (num_of_waypoints - 1);
//     ts[0] = 0.0;
//     for(size_t i = 1; i < num_of_waypoints; i++)
//     {
//         ts[i] = std::stod(args[first_time_index + (i - 1)]);
//     }
//     delta_t_final = ts[num_of_waypoints - 1];

//     // Initialize the PolyTrajGen object
//     const size_t dim = 3; uint poly_order = 7, max_conti = 4;
//     PolyParam pp(poly_order, max_conti, ALGORITHM::POLY_COEFF);
//     Vector4d objDerivativeWeights(0, 0, 0, 1);
//     min_snap_traj_ptr = std::make_unique<PolyTrajGen<double, dim>>(ts, pp);

//     // Add pin constraints on positions at waypoints
//     deleteWaypoints();
//     for(size_t i = 0; i < num_of_waypoints; i++)
//     {
//         waypoints_pin_set.push_back(new FixPin<double, dim>(ts[i], 0.0, waypoints[i]));
//     }
//     min_snap_traj_ptr->addPinSet(waypoints_pin_set);

//     // Add pin constraints on derivatives of endpoints
//     FixPin<double, dim> p0_dot(0.0, 1, Vector3d::Zero());
//     FixPin<double, dim> p0_dotdot(0.0, 2, Vector3d::Zero());
//     FixPin<double, dim> p0_dotdotdot(0.0, 3, Vector3d::Zero());
//     FixPin<double, dim> p0_dotdotdotdot(0.0, 4, Vector3d::Zero());
//     vector<Pin<double, dim>*> p0_derivatives_pin_set = {&p0_dot, &p0_dotdot, &p0_dotdotdot, &p0_dotdotdotdot};
//     min_snap_traj_ptr->addPinSet(p0_derivatives_pin_set);

//     FixPin<double, dim> pf_dot(delta_t_final, 1, Vector3d::Zero());
//     FixPin<double, dim> pf_dotdot(delta_t_final, 2, Vector3d::Zero());
//     FixPin<double, dim> pf_dotdotdot(delta_t_final, 3, Vector3d::Zero());
//     FixPin<double, dim> pf_dotdotdotdot(delta_t_final, 4, Vector3d::Zero());
//     vector<Pin<double, dim>*> pf_derivatives_pin_set = {&pf_dot, &pf_dotdot, &pf_dotdotdot, &pf_dotdotdotdot};
//     min_snap_traj_ptr->addPinSet(pf_derivatives_pin_set);

//     // Attempt to solve for the polynomial's coefficients
//     min_snap_traj_ptr->setDerivativeObj(objDerivativeWeights);
//     bool verbose = false;
//     bool is_solved = min_snap_traj_ptr->solve(verbose);

//     // Return to hovering if trajectory is not feasible
//     if(!is_solved)
//     {
//         setDesiredState(string("Hover"));
//     }
// }

Matrix3d SFTrajectoryTrackingState::calcHelixTraj(double delta_t, bool trajectory_completed)
{
    Matrix3d desired_traj;
    double theta = helix_traj.theta_init + helix_traj.omega * delta_t;
    double sin_theta = sin(theta);
    double cos_theta = cos(theta);

    // Calculate the position vector
    Vector3d desired_pos(helix_traj.center_x + helix_traj.radius * cos_theta, 
                        helix_traj.center_y + helix_traj.radius * sin_theta,
                        start_position(2) + helix_traj.vel_z * delta_t);
    
    // Calculate the velocity vector
    Vector3d desired_vel(-helix_traj.radius * helix_traj.omega * sin_theta,
                        helix_traj.radius * helix_traj.omega * cos_theta,
                        helix_traj.vel_z);
    
    // Calculate the acceleration vector
    Vector3d desired_acc(-helix_traj.radius * pow(helix_traj.omega, 2) * cos_theta,
                        -helix_traj.radius * pow(helix_traj.omega, 2) * sin_theta,
                        0.0);

    // Combine all three vectors into the desired trajectory
    if(!trajectory_completed)
    {
        desired_traj << desired_pos, desired_vel, desired_acc;
    }
    else
    {
        desired_traj << desired_pos, Vector3d::Zero(), Vector3d::Zero();
    }

    return desired_traj;
}

Matrix3d SFTrajectoryTrackingState::calcHelixVaryingTraj(double delta_t, bool trajectory_completed)
{
    Matrix3d desired_traj;
    double theta;
    double omega, omega_dot;
    double prev_z, vel_z;
    double delta_omega = helix_var_traj.omega_max - helix_var_traj.omega_min;

    // Calculate the trajectory parameters based on the active stage
    if(delta_t <= helix_var_traj.time_acc)
    {
        theta = helix_var_traj.omega_min * delta_t + 0.5 * delta_omega * delta_t * delta_t / helix_var_traj.time_acc;
        omega = helix_var_traj.omega_min + delta_omega * delta_t / helix_var_traj.time_acc;
        omega_dot = delta_omega / helix_var_traj.time_acc;
        prev_z = 0.0;
        vel_z = helix_var_traj.vel_z_acc;
    }
    else if (delta_t <= (helix_var_traj.time_acc + helix_var_traj.time_hold))
    {
        theta = helix_var_traj.omega_max * (delta_t - helix_var_traj.time_acc) + helix_var_traj.theta_f1;
        omega = helix_var_traj.omega_max;
        omega_dot = 0.0;
        prev_z = helix_var_traj.vel_z_acc * helix_var_traj.time_acc;
        vel_z = 0.0;
    }
    else
    {
        double start_t = helix_var_traj.time_acc + helix_var_traj.time_hold;
        theta = helix_var_traj.omega_min * (delta_t - start_t) + (delta_omega/helix_var_traj.time_dec) * 
                (delta_t_final * (delta_t - start_t) - 0.5 * (delta_t * delta_t - start_t * start_t)) + helix_var_traj.theta_f2;
        omega = helix_var_traj.omega_min + (delta_omega/helix_var_traj.time_dec) * (delta_t_final - delta_t);
        omega_dot = -delta_omega / helix_var_traj.time_dec;
        prev_z = helix_var_traj.vel_z_acc * helix_var_traj.time_acc - helix_var_traj.vel_z_dec * start_t;
        vel_z = helix_var_traj.vel_z_dec;
    }

    double sin_theta = sin(theta);
    double cos_theta = cos(theta);

    // Calculate the position vector
    Vector3d desired_pos(helix_var_traj.center_x + helix_var_traj.radius * cos_theta, 
                        helix_var_traj.center_y + helix_var_traj.radius * sin_theta,
                        start_position(2) + vel_z * delta_t + prev_z);
    
    // Calculate the velocity vector
    Vector3d desired_vel(-helix_var_traj.radius * omega * sin_theta,
                        helix_var_traj.radius * omega * cos_theta,
                        vel_z);
    
    // Calculate the acceleration vector
    Vector3d desired_acc(-helix_var_traj.radius * omega * omega * cos_theta - helix_var_traj.radius * omega_dot * sin_theta,
                        -helix_var_traj.radius * omega * omega * sin_theta + helix_var_traj.radius * omega_dot * cos_theta,
                        0.0);

    // Combine all three vectors into the desired trajectory
    if(!trajectory_completed)
    {
        desired_traj << desired_pos, desired_vel, desired_acc;
    }
    else
    {
        desired_traj << desired_pos, Vector3d::Zero(), Vector3d::Zero();
    }

    return desired_traj;
}

// Matrix3d SFTrajectoryTrackingState::calcMinSnapTraj(double delta_t)
// {
//     Matrix3d desired_traj;

//     // Evaluate the 7th order spline for the position
//     Vector3d desired_pos = min_snap_traj_ptr->eval(delta_t, 0);

//     // Evaluate the 7th order spline for the velocity
//     Vector3d desired_vel = min_snap_traj_ptr->eval(delta_t, 1);

//     // Evaluate the 7th order spline for the acceleration
//     Vector3d desired_acc = min_snap_traj_ptr->eval(delta_t, 2);

//     // Combine all three vectors into the desired trajectory
//     desired_traj << desired_pos, desired_vel, desired_acc;

//     return desired_traj;
// }

// void SFTrajectoryTrackingState::deleteWaypoints()
// {
//     size_t num_of_waypoints = waypoints_pin_set.size();
//     for(size_t i = 0; i < num_of_waypoints; i++)
//     {
//         delete waypoints_pin_set[i];
//     }
//     waypoints_pin_set.clear();
// }