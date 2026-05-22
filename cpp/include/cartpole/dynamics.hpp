
// This header file contains the interface for the dynamics function.

#pragma once
#include <cartpole/state.hpp>
#include <cartpole/params.hpp>

namespace cartpole
{
    /*
    This function takes in the previous state, current control input and physical parameters
    It returns a 4x1 state that contains the derivatives at the current state
    Form [x_dot, x_ddot, theta_dot, theta_ddot]

    It does NOT return the next state. The return from this function should be used in an iterative euler-forward system
    to determine the next state.
    */
    State dynamics(const State prev_state, const double u, const Params params);
}
