#include <cartpole/state.hpp>
#include <iostream>
#include <cartpole/dynamics.hpp>
#include <cartpole/params.hpp>

using namespace cartpole;

State state;
Params my_params;

int main()
{
    state.x = 0.0;
    state.x_dot = 0.0;
    state.theta = 0.1;
    state.theta_dot = 0.0;

    my_params.L = 1.0;
    my_params.m1 = 0.5;
    my_params.m2 = 0.5;
    my_params.g = 9.81;

    for(int i = 0; i < 50; i++){
        state = state + dynamics(state, 0.0, my_params) * 0.01;

        std::cout << "x: " <<state.x << " theta: " << state.theta << "\n";

    }
    
    return 0;
}

