/*
This cpp file contains the equations of motion for the cartpole simulator.
The input to this function are the previous state and control input.
The output of this function are the derivatives of the input state and the acceleration from the EOM
*/
#include <cartpole/state.hpp>
#include <cartpole/dynamics.hpp>
#include <cmath>
#include <Eigen/Dense>

namespace cartpole
{
    State dynamics(const State prev_state, const double u, const Params params)
    {
        auto s = sin(prev_state.theta);
        auto c = cos(prev_state.theta);
        auto m = params.m1;
        auto mp = params.m2;
        auto L = params.L;
        auto g = params.g;
        auto theta_d_2 = prev_state.theta_dot * prev_state.theta_dot; // theta dot squared

        // EOM of the cartpole system
        Eigen::MatrixXd M{
            {m + mp, mp * L * c},
            {mp * c, mp * L}};

        Eigen::VectorXd F{
            {u + L * theta_d_2 * s * mp},
            {mp * g * s}};

        Eigen::VectorXd xdd = M.inverse() * F;

        /*
        The returned state is of the form [x_dot, x_ddot, theta_dot, theta_ddot]
        State notation is somewhat confusing
        TODO: make it less confusing
        */

        State derivative;

        derivative.x = prev_state.x_dot + xdd(0) * 0.001;         // New x_dot
        derivative.x_dot = xdd(0);                                // New x_ddot
        derivative.theta = prev_state.theta_dot + xdd(1) * 0.001; // New theta_dot
        derivative.theta_dot = xdd(1);                            // New theta_ddot

        return derivative;
    }
}