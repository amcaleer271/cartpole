#include <cartpole/control.hpp>

namespace cartpole
{

    PID::PID(Eigen::Vector2d kp, Eigen::Vector2d ki, Eigen::Vector2d kd){
        this->kp = kp(0,0);
        this->ki = ki(0,0);
        this->kd = kd(0,0);
    };

    double PID::control(State& x){
        return kp*x.x + kd*x.x_dot;
    };

    double LQR::control(State& x){
        return 3.5;
    };

    double MPC::control(State& x){
        return 4.5;
    };
};