#include <cartpole/state.hpp>
#include <cartpole/dynamics.hpp>
#include <cartpole/params.hpp>
#include <cartpole/simulator.hpp>
#include <Eigen/Dense>

using namespace cartpole;

State state0;
Params sys_params;

int main()
{
    state0.x = 0.0;
    state0.x_dot = 0.0;
    state0.theta = 0.1;
    state0.theta_dot = 0.0;

    sys_params.g = 9.81;
    sys_params.L = 0.5;
    sys_params.m1 = 1.0;
    sys_params.m2 = 0.5;

    Eigen::Vector2d kp(2,1);
    Eigen::Vector2d ki(2,1);
    Eigen::Vector2d kd(2,1);

    kp << 1.0, 2.0;
    ki << 1.0, 2.0;
    ki << 1.0, 2.0;


    PID pid_controller(kp, ki, kd);
    simulate(state0, 0.001, 1000, sys_params, pid_controller);
    
    return 0;
}

