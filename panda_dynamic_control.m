%% Panda dynamic control

%% Description: simulation file for dynamic control of a 7 dof Panda Robot in Vrep using Dual Quaternions.
%%Admittance controller: impedance controller in the outer loop +
%%task-space motion control with feedback linearization.

%% Clear workspace
clear;
clc;
close all;

%% Addpath 
include_namespace_dq;

%% Load parameters
run Main.m 
fuse = 1; 

%% Load desired trajectory
switch fuse
    case 1
        % gen_traj: task-space minimum jerk trajectory free-motion.
        [xd1, dxd1, ddxd1] = gen_traj(x_in,time);
    case 2
        % circ_traj: circular trajectory plane y-z
        [xd1, dxd1, ddxd1] = gen_traj(x_in,time);
    case 3
        % int_traj: trajectory for interaction task test
        [xd1, dxd1, ddxd1] = int_traj(x_in,time);
    case 4
        % grasp_traj: trajectory for simple grasping task test
        [xd1,dxd1,ddxd1,grasp_data] = grasp_traj(x_in,time); 
end

%% Connect to vrep

disp('Program started');
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections

clientID=sim.simxStart('127.0.0.1',19997,true,true,5000,5);

i=1;

vi = DQ_VrepInterface;

%% Initialize VREP Robots

fep_vreprobot = FEpVrepRobot('Franka',vi);

%% Load DQ Robotics kinematics

if (clientID>-1)
    disp('Connected to remote API server');
    
    handles = get_joint_handles(sim,clientID);
    joint_handles = handles.armJoints;
    
    
    fep  = fep_vreprobot.kinematics(); 
    
    % get initial state of the robot
    qstr = '[ ';
    qdotstr = '[ ';
    
    for j=1:7
        [res,q(j)] = sim.simxGetJointPosition(clientID,joint_handles(j),sim.simx_opmode_buffer);
        [res,qdot(j)] = sim.simxGetObjectFloatParameter(clientID,joint_handles(j),2012,sim.simx_opmode_buffer);
        qstr = [qstr,num2str(q(j)),' '];
        qdotstr = [qdotstr,num2str(qdot(j)),' '];
    end
    
    qstr = [qstr,']'];
    qdotstr = [qdotstr,']'];
    disp('Initial Joint positions: ');
    disp(qstr);
    disp('Initial Joint Velocities: ');
    disp(qdotstr);
    
    
    %% Setting to synchronous mode
    %---------------------------------------
    sim.simxSynchronous(clientID,true)   
    sim.simxSynchronousTrigger(clientID)
    sim.simxSetFloatingParameter(clientID, sim.sim_floatparam_simulation_time_step, cdt, sim.simx_opmode_oneshot)
    
    %  start our simulation in lockstep 
    sim.simxStartSimulation(clientID, sim.simx_opmode_blocking);
    %---------------------------------------
    
    %% Get joint positions
    %---------------------------------------
    for j=1:7
        [~,tauOrig(j)] = sim.simxGetJointForce(clientID,joint_handles(j),sim.simx_opmode_blocking);
        [~,qmread(j)]  = sim.simxGetJointPosition(clientID,joint_handles(j),sim.simx_opmode_blocking);
    end      
    qm = double([qmread])';

    % Saving data to analyze later
    sres.qm = [];  sres.qm_dot = []; 
    sres.qd = [];  sres.qd_dot = [];  sres.qd_ddot = [];
    sres.T = [];   sres.xdq = []; sres.cdq = [];
    %---------------------------------------    
    
    % time
    inittime = sim.simxGetLastCmdTime(clientID)
    
%% Control loop   
     while sim.simxGetConnectionId(clientID)~=-1
        
        if i>size(qDesired,1)
            break
        end
        
        % Getting joint-position
        %---------------------------------------    
        for j=1:7
            [~,tauOrig(j)] = sim.simxGetJointForce(clientID,joint_handles(j),sim.simx_opmode_blocking);
            [~,qmread(j)]  = sim.simxGetJointPosition(clientID,joint_handles(j),sim.simx_opmode_blocking);
        end      
        qmOld = qm;
        % Current joint configuration 
        disp('Measured joint position');
        qm = double([qmread])';
        
        % Current EE configuration
        disp('Current EE configuration');
        xdq = fep.fkm(qm); %DQ pose
        
        % Pose Jacobian
        Jp = fep.pose_jacobian(qm);
        
        % Geometric Jacobian
        J = geomJ(fep,qm);
        
        % Translation Jacobian
        J_t = J(4:6,:);
        
        % Current joint derivative (Euler 1st order derivative)
        qm_dot = (qm-qmOld)/cdt; %computed as vrep function 
        
        % Pose Jacobian first-time derivative 
        Jp_dot = fep.pose_jacobian_derivative(qm,qm_dot);
        %---------------------------------------    
        
        % Desired joint positions
        q = qDesired(i,:);
        
        % Desired Cartesian pose
        cdq = fep.fkm(q); 
       
        % Printing the time step of the simulation and the error
        % -----------------------
        %disp(['it: ',num2str(i),' time(',num2str(i*cdt),') - error:',num2str(norm( q'-qm ))])  
        disp(['it: ',num2str(i),' time(',num2str(i*cdt),') - error:',num2str(norm(vec8(cdq-xdq)))])
        disp(['it: ',num2str(i),' time(',num2str(i*cdt),') - pos error:',num2str(norm(vec4(cdq.translation-xdq.translation)))])
        
        % Vector with both joints: desired vs real (simulated)
        %disp('Vector with both joints: desired and real (simulated)')
        [q; double([qmread])];
        % ----------------------
        
        % Desired joint velocities
        dq = qdotDesired(i,:);
        
        % Desired joint accelerations
        ddq = qddotDesired(i,:);
        
        % Saving data to analyze later
        % -----------------------
        sres.qm(:,i) = qm;  sres.qm_dot(:,i) = qm_dot;  
        sres.qd(:,i) = q';  sres.qd_dot(:,i) = dq';  sres.qd_ddot(:,i) = ddq'; 
        sres.xdq(:,i) = vec4(xdq.translation); sres.cdq(:,i) = vec4(cdq.translation);
        
        % -----------------------        
        %% Motion Control
        % Using the dynamic model
        O = zeros(1,6);
        I = eye(3);
        A = [O; I zeros(3,1) zeros(3,1) zeros(3,1); ...
             O;zeros(3,1) zeros(3,1)  zeros(3,1) I;]; %mapping
        
        % Using the dynamic model
        g = get_GravityVector(qm);
        c = get_CoriolisVector(qm,qm_dot);
        M = get_MassMatrix(qm);
        % The vrep model does not account for friction torques. Including
        % them leads to bad behaviour. 
        tauf = get_FrictionTorque(qm_dot);                
        
        %% Controller gains;
        % Stiffness matrix
        K = eye(8)*1000; 
        % Damping matrix
        D = eye(8)*5;                                                                    

%%       Task-space inverse dynamics with fb linearization
         kp = 100;
         kd = 10;
         ki = 50; %integral gain 
        
        %% define error 
        xe = vec8(cdq - xdq);
        dxe = Jp*dq' - Jp*qm_dot;
        e = haminus8(DQ(C8*vec8(cdq)))*xe;
        e_dot = haminus8(DQ(C8*Jp*dq'))*xe + haminus8(DQ(C8*vec8(cdq)))*dxe;
        
        %% define cl dynamics
        y =  kd*e_dot +kp*e;
        ddxr = Jp_dot*dq' + Jp*ddq';
        J = geomJ(fep,qm);
        Jg2 = A*J;
        
        %% control input task space
        ax = haminus8(DQ(ddxr)')*xe + 2*haminus8(DQ(C8*Jp*dq'))*dxe + haminus8(DQ(C8*vec8(cdq)))*(ddxr - Jp_dot*qm_dot) + y; 
        J_inv = pinv(haminus8(DQ(C8*vec8(cdq)))*Jp);
        %control input joint space
        aq = J_inv*ax; 
%       aq = Jg2'*ax;

        %% fb linearization
        tau = M*aq + c + g ;
        N = haminus8(cdq)*DQ.C8*Jp;
        robustpseudoinverse = N'*pinv(N*N' + 0.1*eye(8));
        %%%%%%%%% null space control %%%%%%%%%
        P = eye(7) - pinv(N)*N;
        D_joints = eye(7)*2;
        tau_null = P*(-D_joints*qm_dot);
        tau = tau + tau_null;
        
        %store for later
        tau_send = tau;
        sres.tau_send(:,i) = tau_send;
        
        %% Send torques to Vrep
        for j=1:7
            if tau(j)<0
                set_vel = -99999;
            else
                set_vel = 9999;
            end
            % blocking mode
            %---------------------------------         
            sim.simxSetJointTargetVelocity(clientID,joint_handles(j),set_vel,sim.simx_opmode_blocking);            
            sim.simxSetJointForce(clientID,joint_handles(j),abs(tau(j)),sim.simx_opmode_blocking);
            [~,tau_read] = sim.simxGetJointForce(clientID,joint_handles(j),sim.simx_opmode_blocking);
            tau_read_data(:,j) = tau_read;
%             sres.tau_read(:,j) = tau_read_data;

            %---------------------------------  
        end
        sres.tau_read(i,:) = tau_read_data';
        %---------------------------------
        sim.simxSynchronousTrigger(clientID);
        %---------------------------------
        i = i+1;        
    end
   
    % Now close the connection to V-REP:
    sim.simxStopSimulation(clientID,sim.simx_opmode_blocking);
    sim.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
sim.delete(); % call the destructor!

%% PLots if wanted
figure(); 
plot(tt,sres.qd_dot(1,:),'m--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm_dot(1,:),'m','LineWidth',2);
legend('qdotDesired','qdotMeasured'); 
hold on, grid on
plot(tt,sres.qd_dot(2,:),'b--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm_dot(2,:),'b','LineWidth',2);
legend('qdotDesired','qdotMeasured'); 
hold on, grid on
plot(tt,sres.qd_dot(3,:),'g--','LineWidth',3);
hold on, grid on
plot(tt,sres.qm_dot(3,:),'g','LineWidth',2);
legend('qdotDesired','qdotMeasured'); 
hold on, grid on
plot(tt,sres.qd_dot(4,:),'k--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm_dot(4,:),'k','LineWidth',2);
legend('qdotDesired','qdotMeasured'); 
hold on, grid on
plot(tt,sres.qd_dot(5,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm_dot(5,:),'r','LineWidth',2);
legend('qdotDesired','qdotMeasured'); 
hold on, grid on
plot(tt,sres.qd_dot(6,:),'c--','LineWidth',3);
hold on, grid on
plot(tt,sres.qm_dot(6,:),'c','LineWidth',2);
legend('qdotDesired','qdotMeasured'); 
hold on, grid on
plot(tt,sres.qd_dot(7,:),'y--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm_dot(7,:),'y','LineWidth',2);
legend('qdotDesired','qdotMeasured'); 

figure(); 
plot(tt,sres.qd(1,:),'m--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm(1,:),'m','LineWidth',2);
legend('qDesired','qMeasured'); 
hold on, grid on
plot(tt,sres.qd(2,:),'b--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm(2,:),'b','LineWidth',2);
legend('qDesired','qMeasured'); 
hold on, grid on
plot(tt,sres.qd(3,:),'g--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm(3,:),'g','LineWidth',2);
hold on, grid on
legend('qDesired','qMeasured'); 
plot(tt,sres.qd(4,:),'k--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm(4,:),'k','LineWidth',2);
legend('qDesired','qMeasured'); 
hold on, grid on
plot(tt,sres.qd(5,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm(5,:),'r','LineWidth',2);
legend('qDesired','qMeasured'); 
hold on, grid on
plot(tt,sres.qd(6,:),'c--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm(6,:),'c','LineWidth',2);
legend('qDesired','qMeasured'); 
hold on, grid on
plot(tt,sres.qd(7,:),'y--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm(7,:),'y','LineWidth',2);
legend('qDesired','qMeasured'); 

figure(); 
plot(tt,sres.tau_read(:,1),'m--','LineWidth',3); 
hold on, grid on
plot(tt,sres.tau_send(1,:),'m','LineWidth',2);
hold on, grid on
plot(tt,sres.tau_send(2,:),'b--','LineWidth',3); 
hold on, grid on
plot(tt,sres.tau_read(:,2),'b','LineWidth',2);
hold on, grid on
plot(tt,sres.tau_send(3,:),'g--','LineWidth',3); 
hold on, grid on
plot(tt,sres.tau_read(:,3),'g','LineWidth',2);
hold on, grid on
plot(tt,sres.tau_send(4,:),'k--','LineWidth',3); 
hold on, grid on
plot(tt,sres.tau_read(:,4),'k','LineWidth',2);
hold on, grid on
plot(tt,sres.tau_send(5,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,sres.tau_read(:,5),'r','LineWidth',2);
hold on, grid on
plot(tt,sres.tau_send(6,:),'c--','LineWidth',3); 
hold on, grid on
plot(tt,sres.tau_read(:,6),'c','LineWidth',2);
hold on, grid on
plot(tt,sres.tau_send(7,:),'y--','LineWidth',3); 
hold on, grid on
plot(tt,sres.tau_read(:,7),'y','LineWidth',2);
legend('tsend','tread'); 

%%position plot
figure();
plot(tt,sres.cdq(2,:),'c--','LineWidth',3); 
hold on, grid on
plot(tt,sres.xdq(2,:),'c','LineWidth',2);
legend('xd','x')
figure();
plot(tt,sres.cdq(3,:),'c--','LineWidth',3); 
hold on, grid on
plot(tt,sres.xdq(3,:),'c','LineWidth',2);
legend('yd','y')
figure()
plot(tt,sres.cdq(4,:),'c--','LineWidth',3); 
hold on, grid on
plot(tt,sres.xdq(4,:),'c','LineWidth',2);
legend('zd','z')
        