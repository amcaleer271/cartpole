#include <cartpole/control.hpp>

namespace cartpole
{

    PID::PID(Eigen::Vector2d kp, Eigen::Vector2d ki, Eigen::Vector2d kd){
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    };

    double PID::control(State& x){
        return kp(0)*x.x + kd(0)*x.x_dot + kp(1)*x.theta + kd(1)*x.theta_dot;
    };

    double LQR::control(State& x){
        return 3.5;
    };

    double MPC::control(State& x){
        return 4.5;
    };
};