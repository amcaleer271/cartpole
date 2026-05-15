#include <cartpole/state.hpp>
#include <iostream>
#include <cartpole/dynamics.hpp>
#include <cartpole/params.hpp>

using namespace cartpole;

State my_state;
State new_state;
Params my_params;

int main()
{
    my_state.x = 1.5;
    my_state.x_dot = 0.0;
    my_state.theta = 0.1;
    my_state.theta_dot = 0.0;

    my_params.L = 1.0;
    my_params.m1 = 0.5;
    my_params.m2 = 0.5;
    new_state = dynamics(my_state, 0.0, my_params);

    std::cout << new_state.x;

    return 0;
}

