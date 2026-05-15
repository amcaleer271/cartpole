#pragma once
#include <cartpole/state.hpp>
#include <cartpole/params.hpp>

namespace cartpole{
    State simulate(State state0, double dt, int n, Params params);
}