%% Main scipt

%% Addpath 

addpath /home/geriatronics/github/panda_dynamic_control/functions
addpath /home/geriatronics/github/panda_dynamic_control/Vrep_utils

disp('Loading data..')

%% Robot joint limits

q_min = [-2.8973   -1.7628   -2.8973   -3.0718   -2.8973   -0.0175  -2.8973];
q_max = [ 2.8973    1.7628    2.8973  -0.0698    2.8973    3.7525    2.8973];

%% Parameters

cdt = 0.01; %sampling time
time = 0:cdt:2.5; %simulation time

%% Build robot

FEp_DH_theta = [0, 0, 0, 0, 0, 0, 0];
FEp_DH_d = [0.333, 0, 0.316, 0, 0.384, 0, 0.107];
FEp_DH_a = [0, 0, 0.0825, -0.0825, 0, 0.088 0.0003];
FEp_DH_alpha = [-pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2 0];
FEp_DH_matrix = [FEp_DH_theta; FEp_DH_d; FEp_DH_a; FEp_DH_alpha]; 
franka_dh_matrix = FEp_DH_matrix; 
franka = DQ_SerialManipulator(FEp_DH_matrix,'standard');

%% Initial conditions
%%Joint angles
q_in = [ 1.1515    0.3950    0.2619   -1.5722   -0.0002    1.3958    0.0001]'; %rad
% q_in = [0 0 0 -1.5708 0 1.5708 0]'; %rad
%%End-effector pose
x_in = franka.fkm(q_in); 
p0_in = vec4(x_in.translation);
r0_in = vec4(P(x_in));
dx_in = zeros(8,1);


disp('Loading done!')
