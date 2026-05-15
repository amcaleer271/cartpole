#pragma once
namespace cartpole
{
    struct State
    {
        double x;
        double x_dot;
        double theta;
        double theta_dot;
    };

    //define an operation for addition between two states
    inline State operator+ (State s1, State s2){
        State sum_state;
        sum_state.x = s1.x + s2.x;
        sum_state.x_dot = s1.x_dot + s2.x_dot;
        sum_state.theta = s1.theta + s2.theta;
        sum_state.theta_dot = s1.theta_dot + s2.theta_dot;

        return sum_state;
    }

    //define an operation for multiplication between a state and a scalar
    inline State operator* (State s1, double scalar){
        State prod_state;
        prod_state.x = s1.x*scalar;
        prod_state.x_dot = s1.x_dot*scalar;
        prod_state.theta = s1.theta*scalar;
        prod_state.theta_dot = s1.theta_dot*scalar;

        return prod_state;
    }

}