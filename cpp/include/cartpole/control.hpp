#pragma once
#include <cartpole/state.hpp>
#include <Eigen/Dense>

namespace cartpole
{
    class Controller{
        public:
            virtual double control(State& x) = 0;
            virtual ~Controller() = default;
    };

    class PID : public Controller{
        private:
            Eigen::Vector2d kp, ki, kd;

        public:
            PID(Eigen::Vector2d kp, Eigen::Vector2d ki, Eigen::Vector2d kd);
            double control(State& x);
    };

    class LQR : public Controller{
        public:
            double control(State& x);
    };

    class MPC : public Controller{
        public:
            double control(State& x);
    };
};

