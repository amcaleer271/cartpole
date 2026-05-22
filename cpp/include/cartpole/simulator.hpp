// This header file defines the interface of the simulate function

#pragma once
#include <cartpole/state.hpp>
#include <cartpole/params.hpp>
#include <cartpole/control.hpp>

namespace cartpole
{
    /*
    Simulate runs the physics simulator starting from the passed state state0, with time increment dt
    for n number of simulation iterations.
    The physical parameters of the system (gravity, cart/pendulum mass) are defined by the passed params
    The controller can also be specified
    */
    void simulate(const State &state0, double dt, int n, const Params &params, Controller &controller);
}