/*
This file contains the loop that runs the simulator according to dynamics in dynamics.cpp

Inputs:
state0 (State) - initial state of the simulation
dt (double) - timestep of simulation
n (int) - number of loops of simulation to run
params (Params) - physical parameters of the simulator

Outputs:
x (state) - current state of the system
*/

#include <cartpole/state.hpp>
#include <cartpole/dynamics.hpp>
#include <cmath>
#include <iostream>

namespace cartpole
{
    State simulate(State state0, double dt, int n, Params params){
        State currentState = state0;

        double u = 0.0; //placeholder
        for(int i = 0; i < n; i++){
            currentState = currentState + dynamics(currentState, u, params) * dt;

            std::cout << "x: " << currentState.x << " theta: " << currentState.theta << "\n";
        }
        
        return currentState;
    }
} 
