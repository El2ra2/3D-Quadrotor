# 3D-Quadrotor
Implemented a PD controller to control the motion of the quadrotor in the 3D plane, and traversed it on different trajectories.

To test different trajectories, you need to modify the variable trajhandle (uncomment the desired 'trajhandle' variable and comment the undesired ones) inside runsim.m to point to the appropriate function. The PD controller was initiated in the file controller.m so that the simulated quadrotor can follow a desired trajectory.
