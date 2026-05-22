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
#include <cartpole/logging.hpp>
#include <cartpole/control.hpp>

namespace cartpole
{
    void simulate(const State &state0, double dt, int n, const Params &params, Controller &controller)
    {
        State currentState = state0;
        Logger logger;

        double t = 0.0;
        double u = 0.0;

        logger.store(currentState, u, t);

        // Iteratevly call the simlulate function to advance the simulation by dt
        for (int i = 0; i < n; i++)
        {
            u = controller.control(currentState);
            currentState = currentState + dynamics(currentState, u, params) * dt;
            t += dt;
            logger.store(currentState, u, t);
        }
    }
}
