# unicycle_sociallycompliant_nav
An MPC socially compliant autonomous navigation strategy for unicycle robots.

This repository contains the Matlab/Simulink code related to the paper

Luca Bascetta, Maria Prandini, Alessandro Febbraro. Socially compliant autonomous navigation of a unicycle robot in human crowded environments. IEEE Transactions on Control Systems Technology

To run the simulator, first execute the init script that defines all the paramters for the planner, the controller and the robot simulator, and calls RRTX to plan the preliminary trajectory, then open the Simulink simulator wheelchair_navigation and run it.

Different maps are available in map folder, and different environment configurations (including pedestrian trajectories) are available in sim_congifurations folder.
