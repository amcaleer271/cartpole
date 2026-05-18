#pragma once
#include <cartpole/state.hpp>
#include <cartpole/params.hpp>
#include <cartpole/control.hpp>

namespace cartpole{
    State simulate(const State& state0, double dt, int n, const Params& params, Controller& controller);
}