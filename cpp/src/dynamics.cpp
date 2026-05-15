/*
This cpp file contains the equations of motion for the cartpole simulator.
The input to this file are the previous state and control input.
The output of this file are the acceleration and velocity of the linear and angular components of the state
*/
#include <cartpole/state.hpp>
#include <cartpole/dynamics.hpp>

namespace cartpole{
State dynamics(const State prev_state, const float u, const Params params){
    State return_state;

    return_state.x = 100.0;
    return_state.x_dot = 0.5;
    return_state.theta = 1.5;
    return_state.theta_dot = 1.9;

    return return_state;
}
}