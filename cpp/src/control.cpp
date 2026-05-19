#include <cartpole/control.hpp>

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
    };

    double LQR::control(State& x){
        return 3.5;
    };

    double MPC::control(State& x){
        return 4.5;
    };
};