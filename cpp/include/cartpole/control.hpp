
// This header file contains the interfaces for the controller classes

#pragma once
#include <cartpole/state.hpp>
#include <Eigen/Dense>
#include <cartpole/params.hpp>

namespace cartpole
{
    class Controller
    {
    protected:
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4, 4);
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(4, 1);

    public:
        virtual double control(State &x) = 0; // each controller type child class must have a control function
        virtual ~Controller() = default;

        void set_system(Params &params); // creates the A and B state space matricies of x_dot=Ax+Bu
    };

    class PID : public Controller
    {
    private:
        Eigen::Vector2d kp, ki, kd;

    public:
        // Gains should be passed to the constructor as 2x1 vectors. The first entry corresponds to x and the second to theta
        PID(Eigen::Vector2d kp, Eigen::Vector2d ki, Eigen::Vector2d kd);
        double control(State &x);
    };

    class LQR : public Controller
    {
    private:
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
        Eigen::MatrixXd P;
        Eigen::MatrixXd K;
        Eigen::Vector4d current_state;

    public:
        double control(State &x);
        LQR(Eigen::MatrixXd Q, Eigen::MatrixXd R, Params params);
    };

    class MPC : public Controller
    {
    public:
        double control(State &x);
    };
};
