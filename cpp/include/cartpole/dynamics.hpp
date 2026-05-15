#pragma once
#include <cartpole/state.hpp>
#include <cartpole/params.hpp>

namespace cartpole
{
    State dynamics(const State prev_state, const double u, const Params params);
}
