#include <cartpole/state.hpp>
#include <iostream>

cartpole::state my_state;

int main()
{
    my_state.x = 1.5;
    my_state.x_dot = 0.0;
    my_state.theta = 0.1;
    my_state.theta_dot = 0.0;

    std::cout << "position is " << my_state.x;

    return 0;
}


