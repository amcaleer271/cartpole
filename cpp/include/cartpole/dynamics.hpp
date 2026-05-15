#pragma once
#include <cartpole/state.hpp>
#include <cartpole/params.hpp>

namespace cartpole
{
    State dynamics(const State prev_state, const float u, const Params params);
}
