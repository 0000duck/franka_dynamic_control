# 7 dof Panda Robot Dynamic Control

## Admittance controller using DQ log mapping

This repository contains matlab code for the simulations in CoppeliaSim via RemoteApi functions of an admittance controller using Dual Quaternion framework of a 7 dof Panda robot.

The *admittance_control.m* contains the code for computing the desired compliant trajectory using the logaritm mapping of a dual quaternion.

The reference trajectory changes in accordance with the external forces acting on the end-effector, and modifies the desired trajectory to be tracked by an inner motion control loop.

The inner motion control loop is a task-space controller that realizes a complete feedback linearization by means of inverse dynamics (see https://github.com/marcocognetti/FrankaEmikaPandaDynModel for dynamics model). 

The controller is tested on 4 four simple demos:

1. Free motion tasks: *gen_traj.m* (fuse = 1) and *int_traj.m* (fuse = 2);
2. Interaction tasks: *int_traj.m* (fuse = 3) and *grasp_traj.m* (fuse = 4).

Each demo can be set by changing the value of fuse in *Main.m* to the corresponding value.

The file *panda_dynamic_control* contains the code for starting the simulation.

Before running the file, open the corresponding CoppeliaSim scene (.ttt) inside the folder.

Instructions for running the code are inside the *reas_me.txt* file.









