%% Main script

%% Clear workspace
clear all;
close all;
clc; 

%% define simulation 
% fuse = 1,2 %free motion simulation (open franka_scene.ttt)
fuse = 3;  %interaction task with table (open franka_int.ttt)
% fuse = 4 %grasping task (open franka_grap.ttt)

%% Addpath 

p1 = genpath('DQ');
p2 = genpath('functions');
p3 = genpath('Vrep_utils');
p4 = genpath('impedance_admittance'); 
p5 = genpath('Data');
addpath(p1,p2,p3,p4,p5); 

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
else
  q_in = [0  0.1745 0  -1.7453  0.3491 1.5708  0]; %rad
end


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

%% Variable gain utils
berta = 0.98;
inc = 1.08;
csi = 1;
a0 = 0.99;
k_default = 1000; %N/m
k_int = 100; %N/m arbitrarly low gain for interaction 
k_track = 1000; %N/m arbitrarly high gain
F_max = 2; %N
e_max = 0.01; %m
F_int_max = 5; %N
mass = 1.5; %kg


disp('Loading done!')



