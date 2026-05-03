This is my ongoing cartpole project! The goal of this project is to repeat the "have problem -> make solution -> make problem harder -> make new solution"
loop as many times as possible.

Here's the log of the project so far:

0. Created a simulator for the cartpole inverted pendulum problem, with the goal of balancing the pole upright in a limited horizontal space
1. Implemented a PID controller that could balance the pendulum. Works well for small masses! Struggles with larger masses
2. Implemented an LQR controller that can use the dynamics of the system to inform the controller! Works well for a large range of masses
3. Added noise to the measurements for position and velocity (both linear and angular). The LQR controller can still handle it, but much greater control input is required