#include <cartpole/control.hpp>
#include <limits>
#include <iostream>
namespace cartpole
{
    //PID Constuctor, takes in gains
    PID::PID(Eigen::Vector2d kp, Eigen::Vector2d ki, Eigen::Vector2d kd){
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    };

    double PID::control(State& x){
        return kp(0)*x.x + kd(0)*x.x_dot + kp(1)*x.theta + kd(1)*x.theta_dot;
    };

    //create state space A and B matricies based on physical params
    void Controller::set_system(Params& params){
        this->A(0,1) = 1.0;
        this->A(1,2) = -1.0 * params.g * params.m2 / params.m1;
        this->A(2,3) = 1.0;
        this->A(3,2) = params.g * (params.m1 + params.m2) / (params.L * params.m1);

        this->B(1,0) = 1.0 / params.m1;
        this->B(3,0) = -1.0 / (params.m1 * params.L);
    };

    //LQR Constructor, takes in gains
    LQR::LQR(Eigen::MatrixXd Q, Eigen::MatrixXd R){
        this->Q = Q;
        this->R = R;

        double inf = std::numeric_limits<double>::infinity();

        this->P = this->Q;
        Eigen::MatrixXd prev_P = Eigen::MatrixXd::Zero(4,4);

        while(abs((this->P - prev_P).norm()) > 0.001){
            prev_P = this->P;
            this->P = this->A.transpose() * this->P * this->A - this->A.transpose() * this->P * this->B * (this->R + this->B.transpose() * this->P * this->B).inverse() * this->B.transpose() * this->P * this->A + this->Q;
        }

        this->K = this->R.inverse() * this->B.transpose()* this->P;
    };

    double LQR::control(State& x){
        
        this->current_state(0) = x.x;
        this->current_state(1) = x.x_dot;
        this->current_state(2) = x.theta;
        this->current_state(3) = x.theta_dot;

        Eigen::MatrixXd u = (-1.0 * this->K * this->current_state);

        return u(0,0);
    };

    double MPC::control(State& x){
        return 4.5;
    };
};