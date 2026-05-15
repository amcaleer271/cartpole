/*
This cpp file contains the equations of motion for the cartpole simulator.
The input to this function are the previous state and control input.
The output of this function are the derivatives of the input state and the acceleration from the EOM
*/
#include <cartpole/state.hpp>
#include <cartpole/dynamics.hpp>
#include <cmath>

namespace cartpole{
State dynamics(const State prev_state, const double u, const Params params){
    State derivative;

    auto s = sin(prev_state.theta);
    auto c = cos(prev_state.theta);
    auto m = params.m1;
    auto mp = params.m2;
    auto mpc2 = mp* c * c;
    auto L = params.L;
    auto g = params.g;
    auto theta_dot2 = prev_state.theta_dot*prev_state.theta_dot;

    auto denom = -1 * L * mpc2 + L * m + L * mp;

    derivative.x = prev_state.x_dot;
    derivative.x_dot = ((L * mp * s * (theta_dot2) + u) - (g * mp * c * s)) / (-1 * mpc2 + m + mp);
    derivative.theta = prev_state.theta_dot;
    derivative.theta_dot = ((g*mp*s*(m+mp))/(mp*denom))-((c*(L*mp*s*(theta_dot2)+u))/(denom));

    return derivative;
}
}