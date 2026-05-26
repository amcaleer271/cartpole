# Cartpole Control Simulator

This cartpole project was made with a few free weeks I had between finishing undergrad and starting my summer internsihp! It includes a simple euler-forward implementation to model the dynamics of a cartpole/inverted pendulum. Three controllers (PID, LQR, MPC) are implemented with the objective of keeping the pendulum upright and the cart close to the origin. The results, including the cart position, pendulum angle, and controller input are saved and plotted.

The simulator was first made in Python, then recreated in C++ (mainly for me to refresh my C++ skills). 

<img width="1166" height="596" alt="image" src="https://github.com/user-attachments/assets/bd4ba114-7b9f-450e-a2db-dc32b4700ed6" />


### Short project log:

0. Created a simulator for the cartpole inverted pendulum problem, with the goal of balancing the pole upright in a limited horizontal space
1. Implemented a PID controller that could balance the pendulum. Works well for small masses! Struggles with larger masses
2. Implemented an LQR controller that can use the dynamics of the system to inform the controller! Works well for a large range of masses
3. Added noise to the measurements for position and velocity (both linear and angular). The LQR controller can still handle it, but much greater control input is required
4. Added a kalman filter to estimate the cart position given the noisy measurements. Works great! Using this estimated position greatly reduces controller input
5. Created an MPC controller (just wanted to try it). Took a little while to get working and learned a bunch, now working pretty well!
6. Recreated the physics simulator in C++
7. Implemented PID and LQR control in C++
8. TODO: implement MPC control in C++
