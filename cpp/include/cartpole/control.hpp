#pragma once
#include <cartpole/state.hpp>
#include <Eigen/Dense>
#include <cartpole/params.hpp>

namespace cartpole
{
    class Controller{
        private:
            Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4,4);
            Eigen::MatrixXd B = Eigen::MatrixXd::Zero(4,1);
        public:
            virtual double control(State& x) = 0;
            virtual ~Controller() = default;

            void set_system(Params& params);  //creates the A and B state space matricies
    };

    class PID : public Controller{
        private:
            Eigen::Vector2d kp, ki, kd;

        public:
            PID(Eigen::Vector2d kp, Eigen::Vector2d ki, Eigen::Vector2d kd);
            double control(State& x);
    };

    class LQR : public Controller{
        private:
            Eigen::MatrixXd Q;
            Eigen:: MatrixXd R;

        public:
            double control(State& x);
            LQR(Eigen::MatrixXd Q, Eigen::MatrixXd R);
    };

    class MPC : public Controller{
        public:
            double control(State& x);
    };
};

