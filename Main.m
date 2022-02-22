%% Main script

%% Clear workspace
clear all;
close all;
clc; 

%% define simulation 
% fuse = 1,2 %free motion simulation (open franka_scene.ttt)
% fuse = 3 %interaction task with table (open franka_int.ttt)
% fuse = 4 %grasping task (open franka_grap.ttt)
fuse = 3; 

%% Addpath 

p1 = genpath('DQ');
p2 = genpath('functions');
p3 = genpath('Vrep_utils');
addpath(p1,p2,p3); 

disp('Loading data..')

%% Robot joint limits

q_min = [-2.8973   -1.7628   -2.8973   -3.0718   -2.8973   -0.0175  -2.8973];
q_max = [ 2.8973    1.7628    2.8973  -0.0698    2.8973    3.7525    2.8973];

%% Parameters
I = eye(6);
C8 = DQ.C8;
cdt = 0.01; %sampling time
time = 0:cdt:2.5; %simulation time
tt = time; 

%% Build robot

FEp_DH_theta = [0, 0, 0, 0, 0, 0, 0];
FEp_DH_d = [0.333, 0, 0.316, 0, 0.384, 0, 0.107];
FEp_DH_a = [0, 0, 0.0825, -0.0825, 0, 0.088 0.0003];
FEp_DH_alpha = [-pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2 0];
FEp_DH_matrix = [FEp_DH_theta; FEp_DH_d; FEp_DH_a; FEp_DH_alpha]; 
franka_dh_matrix = FEp_DH_matrix; 
franka = DQ_SerialManipulator(FEp_DH_matrix,'standard');

%% Initial conditions
%% Joint angles
if fuse == 1 || fuse == 2
  q_in = [ 1.1519 0.38397 0.2618 -1.5708 0 1.3963 0]'; %rad
  pose_joint = DQ(1) + 0.5*DQ.E*(DQ([0;0.0413;0;0]));
else
  q_in = [0  0.1745 0  -1.7453  0.3491 1.5708  0]; %rad
  pose_joint = DQ(1) + 0.5*DQ.E*(DQ([0;0.0413;0;0]));
end
% franka.set_reference_frame(pose_joint);


%% End-effector pose
x_in = franka.fkm(q_in);
p0_in = vec4(x_in.translation);
r0_in = vec4(x_in.P);
dx_in = zeros(8,1);

%% Impedance gains
if fuse == 1 || fuse == 2
    Md1 = 1.5*I; %desired mass
    Kd1 = 1000*I; %desired stiffness
    Bd1 = sqrt(4*Kd1*Md1); %desired damping
else
    Md1 = 1.5*I; %desired mass
    Kd1 = 300*I; %desired stiffness
    Bd1 = 4*sqrt(4*Kd1*Md1); %desired damping
end


disp('Loading done!')
