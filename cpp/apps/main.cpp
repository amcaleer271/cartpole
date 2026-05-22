#include <cartpole/state.hpp>
#include <cartpole/dynamics.hpp>
#include <cartpole/params.hpp>
#include <cartpole/simulator.hpp>
#include <cartpole/control.hpp>
#include <Eigen/Dense>

using namespace cartpole;

int main()
{
    State state0;
    Params sys_params;

    //Set the initial state of the system, units are m for x and rads for theta
    state0.x = 0.0;
    state0.x_dot = 0.0;
    state0.theta = 0.1;
    state0.theta_dot = 0.0;

    //Set physical parameters of the system
    sys_params.g = 9.81;
    sys_params.L = 0.5;
    sys_params.m1 = 1.0;
    sys_params.m2 = 0.5;

    //Create and set gains for PID controller
    Eigen::Vector2d kp(2,1);
    Eigen::Vector2d ki(2,1);
    Eigen::Vector2d kd(2,1);

    //First entry is for x, second entry is for theta
    kp << 14.5, 50.0;
    ki << 0.0, 0.0;
    kd << 9.0, 9.0;

    //Create and set gains for LQR controller
    Eigen::Vector4d q(15.0, 3.0, 30.0, 4.0);
    auto Q = q.asDiagonal(); 

    Eigen::Matrix<double, 1, 1> R;
    R(0,0) = 0.5;

    //Create controller object and run simulation loop
    //Note that the simulate() function writes to log.csv, will be overwritten by multiple calls
    PID pid_controller(kp, ki, kd);
    simulate(state0, 0.001, 5000, sys_params, pid_controller);

    LQR lqr_controller(Q, R, sys_params);
    simulate(state0, 0.001, 5000, sys_params, lqr_controller);
    return 0;
}

