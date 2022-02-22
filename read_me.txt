## READ ME
 
Description: simulation of 4 simple tasks in CoppeliaSim of a dynamic controlled 7 dof Panda Robot using DQ framework.

Instructions to run the simulation: 

1) Open the corresponding CoppeliaSim scene inside the folder:

   - franka_scene.ttt --> to simulate free motion tasks;
   - franka_int.ttt --> to simulate an interaction/grasping task with the environment;

2) Set the value of flag fuse inside Main.m file to the corresponding demo:

   fuse = 1 --> free motion tracking of a minimum jerk trajectory (x-z plane);
   fuse = 2 --> free motion tracking of a circular trajectory (y-z plane);
   fuse = 3 --> interaction task with the environment;
   fuse = 4 --> grasping task of an object.

3) run Main.m

4) run panda_dynamic_control.m 

NB: to change the simulation time and sampling time simply modify the values of time and cdt respectivately in main.m file.
    to try different trajectories with the same interpolation simply modify values of time and pos_i,pos_f inside the for cicles in the correspondin .m files. 