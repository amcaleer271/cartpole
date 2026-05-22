// This file contains the implementation of the controllers and some useful tools

#include <cartpole/control.hpp>
#include <iostream>
namespace cartpole
{
    // PID Constuctor, takes in gains
    PID::PID(Eigen::Vector2d kp, Eigen::Vector2d ki, Eigen::Vector2d kd)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    };

    double PID::control(State &x)
    {
        return kp(0) * x.x + kd(0) * x.x_dot + kp(1) * x.theta + kd(1) * x.theta_dot;
    };

    // create state space A and B matricies based on physical params. Form derived from EOM
    void Controller::set_system(Params &params)
    {
        this->A(0, 1) = 1.0;
        this->A(1, 2) = -1.0 * params.g * params.m2 / params.m1;
        this->A(2, 3) = 1.0;
        this->A(3, 2) = params.g * (params.m1 + params.m2) / (params.L * params.m1);

        this->B(1, 0) = 1.0 / params.m1;
        this->B(3, 0) = -1.0 / (params.m1 * params.L);

        // discretize system dynamics
        this->A = Eigen::Matrix4d::Identity(4, 4) + this->A * 0.001;
        this->B = this->B * 0.001;
    };

    // LQR Constructor, takes in gains
    LQR::LQR(Eigen::MatrixXd Q, Eigen::MatrixXd R, Params params)
    {
        this->Q = Q;
        this->R = R;

        set_system(params);

        this->P = this->Q; // initialize p matrix with gains from Q matrix

        Eigen::MatrixXd prev_P = Eigen::Matrix4d::Zero(4, 4); // Store a reference matrix

        /*
        use an iterative method for finding values for the P matrix in the discrete time algebraic Riccati equation
        The iterative equation used comes from https://en.wikipedia.org/wiki/Algebraic_Riccati_equation
        stop iterating when successive differences between matricies are minimal, or max iterations exceeded
        */

        int iteration = 0;
        int max_iterations = 10000;

        while (abs((this->P - prev_P).norm()) > 0.001 && iteration < max_iterations)
        {
            prev_P = this->P;
            this->P = this->A.transpose() * this->P * this->A - this->A.transpose() * this->P * this->B * (this->R + this->B.transpose() * this->P * this->B).inverse() * this->B.transpose() * this->P * this->A + this->Q;

            iteration += 1;
        }

        // Compute the K gain using the determined P values, system matricies, and gains
        this->K = (this->R + this->B.transpose() * this->P * this->B).inverse() * this->B.transpose() * this->P * this->A;
    };

    double LQR::control(State &x)
    {

        this->current_state(0) = x.x;
        this->current_state(1) = x.x_dot;
        this->current_state(2) = x.theta;
        this->current_state(3) = x.theta_dot;

        // Apply the optimal control law u = -Kx
        Eigen::MatrixXd u = (-1.0 * this->K * this->current_state);

        return u(0, 0);
    };

    double MPC::control(State &x)
    {
        // Todo
        return 4.5;
    };
};