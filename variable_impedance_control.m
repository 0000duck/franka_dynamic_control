
%% Panda dynamic admittance control with variable impedance gains.

%% Description: simulation file for dynamic control of a 7 dof Panda Robot in Vrep using Dual Quaternions.
%%Admittance controller: impedance controller in the outer loop +
%%task-space motion control with feedback linearization. 

%% Addpath 
include_namespace_dq;

%% Initialize variables
%%admittance controller
xc_data = zeros(size(time,2),8);
dxc_data = zeros(size(time,2),8);
ddxc_data = zeros(size(time,2),8);
yr_data = zeros(size(time,2),6);
dyr_data =  zeros(size(time,2),6);

k_data = zeros(size(time,2),3);
d_data = zeros(size(time,2),3);

%%wrench vector
w_ext_data = zeros(size(time,2),6); %external wrench on EE (world_frame)
psi_ext_data = zeros(size(time,2),6); %external wrench on EE (complinat_reference_frame)


%% Connect to vrep

disp('Program started');
%% Initialize V-REP interface

vi = DQ_VrepInterface;
vi.disconnect_all();
vi.connect('127.0.0.1',19997);
clientID = vi.clientID;
sim = vi.vrep;

i=1;


%% Initialize VREP Robots

fep_vreprobot = FEpVrepRobot('Franka',vi);

%% Load DQ Robotics kinematics

if (clientID>-1)
    disp('Connected to remote API server');
    
    handles = get_joint_handles(sim,clientID);
    joint_handles = handles.armJoints;
    fep  = fep_vreprobot.kinematics(); 
  
    %% get initial state of the robot
    qstr = '[ ';
    qdotstr = '[ ';
    
    for j=1:7
        [res,q(j)] = sim.simxGetJointPosition(clientID,joint_handles(j),sim.simx_opmode_buffer);
        [res,qdot(j)] = sim.simxGetObjectFloatParameter(clientID,joint_handles(j),2012,sim.simx_opmode_buffer);
        qstr = [qstr,num2str(q(j)),' '];
        qdotstr = [qdotstr,num2str(qdot(j)),' '];
    end
    
    q_in = double([q])'; 

    qstr = [qstr,']'];
    qdotstr = [qdotstr,']'];
    disp('Initial Joint positions: ');
    disp(qstr);
    disp('Initial Joint Velocities: ');
    disp(qdotstr);
    
    x_in = fep.fkm(q_in); 

   %% Load desired trajectory
switch fuse
    case 1
        % gen_traj: task-space minimum jerk trajectory free-motion.
        [xd1, dxd1, ddxd1] = gen_traj(x_in,time);
    case 2
        % circ_traj: circular trajectory plane y-z.
        [xd1, dxd1, ddxd1] = circ_traj(x_in,time);
    case 3
        % int_traj: trajectory for interaction task test.
        [xd1,dxd1,ddxd1,phase_data] = var_int_traj(x_in,time); 
        
    case 4
        % grasp_traj: trajectory for simple grasping task test.
        [xd1,dxd1,ddxd1,grasp_data] = grasp_traj(x_in,time); 
end



    %% Setting to synchronous mode
    %---------------------------------------
    sim.simxSynchronous(clientID,true);  
    sim.simxSynchronousTrigger(clientID);
    sim.simxSetFloatingParameter(clientID, sim.sim_floatparam_simulation_time_step, cdt, sim.simx_opmode_oneshot);
    
    %  start simulation
    sim.simxStartSimulation(clientID, sim.simx_opmode_blocking);
    %---------------------------------------
    
    %% Get joint positions
    %---------------------------------------
    for j=1:7
        [~,tauOrig(j)] = sim.simxGetJointForce(clientID,joint_handles(j),sim.simx_opmode_blocking);
        [~,qmread(j)]  = sim.simxGetJointPosition(clientID,joint_handles(j),sim.simx_opmode_blocking);
    end      
    qm = double([qmread])';
    
    % Saving data to struct analyze later
    sres.qm = [];  sres.qm_dot = []; sres.tau_send = []; sres.kd = [];
    sres.xd = [];  sres.xd_dot = [];  sres.xd_ddot = []; sre.bd = []; 
    sres.x = []; sres.xref = []; sres.r = []; sres.norm = []; 
    %---------------------------------------    
    
    % time
    inittime = sim.simxGetLastCmdTime(clientID)
    
%% Control loop   
     while sim.simxGetConnectionId(clientID)~=-1
        
        if i>size(time,2)
            break
        end
        
        % Getting joint-position
        %---------------------------------------    
        for j=1:7
            [~,tauOrig(j)] = sim.simxGetJointForce(clientID,joint_handles(j),sim.simx_opmode_blocking);
            [~,qmread(j)]  = sim.simxGetJointPosition(clientID,joint_handles(j),sim.simx_opmode_blocking);
        end   
        
        qmOld = qm; %update value
        
        %Current joint configuration
        qm = double([qmread])';
        
        % Current EE configuration
        x = vec8(fep.fkm(qm)); %EE pose
        pos = vec4(DQ(x).translation); %EE position
        r0 = vec4(DQ(x).P); %EE rotation
        
        % Pose Jacobian
        Jp = fep.pose_jacobian(qm);
        
        % Geometric Jacobian
        J = geomJ(fep,qm);
        
        % Translation Jacobian
        J_t = J(4:6,:);
        
        % Current joint derivative (Euler 1st order derivative)
        qm_dot = (qm-qmOld)/cdt; %computed as vrep function 
        
        %Current 1st-time derivative of EE pose
        x_dot = Jp*qm_dot;
        
        % Pose Jacobian first-time derivative 
        Jp_dot = fep.pose_jacobian_derivative(qm,qm_dot);
        %---------------------------------------    
        
        %% Admittance control
        % initialize variable
        if i~=1
            xr = xc_data(i-1,:)';
            yr_in = yr_data(i-1,:)';
            dyr_in = dyr_data(i-1,:)';
        else
            xr = vec8(x_in);
            e_in = vec8(DQ(xr)'*DQ(xd1(1,:)));
            yr_in = vec6(log(DQ(e_in)));
            dyr_in = zeros(6,1);
        end
        
        %% Model ext forces
        
        switch fuse
            case 1
                % free-motion trajectory
                psi_ext = zeros(1,6); %external wrench (compliant frame)
                psi_ext_data(i,:) = psi_ext; 
            case 2
                % free-motion trajectory
                psi_ext = zeros(1,6); %external wrench (compliant frame)
                psi_ext_data(i,:) = psi_ext; 
            case 3
                % interaction task: model of external forces
                wrench_ext = ext_forces(x);
                w_ext_data(i,:) = wrench_ext;
                psi_ext = vec6(DQ(r0)'*DQ(wrench_ext)*DQ(r0)); %external wrench (compliant frame)
                psi_ext = psi_ext';
                psi_ext_data(i,:) = psi_ext;
            case 4
                 % grasping task: model of external forces task test.
                grasp = grasp_data(i,:);
                wrench_ext = fext_grasp(x,grasp);
                w_ext_data(i,:) = wrench_ext;
                psi_ext = vec6(DQ(r0)'*DQ(wrench_ext)*DQ(r0)); %external wrench (compliant frame)
                psi_ext = psi_ext'; 
                psi_ext_data(i,:) = psi_ext;
        end
      
      %% Modulate impedance gains  

      %%Compute position error
        xe = vec8(DQ(x)'* DQ(xd1(i,:))); %pose displacement
        e = vec4(2*D(log(DQ(xe)))); %position error
        e_pos = [e(2); e(3); e(4)];
        curr_pos = [pos(2);pos(3);pos(4)];  

     %%retrieve external forces
       f_ext = w_ext_data(i,1:3)'; 
    
     %%compute stiffness and damping 
       [kx,dx] = modulator(time,curr_pos(1),e_pos(1),f_ext(1),phase_data(i,:));
       [ky,dy] = modulator(time,curr_pos(2),e_pos(2),f_ext(2),phase_data(i,:));
       [kz,dz] = modulator(time,curr_pos(3),e_pos(3),f_ext(3),phase_data(i,:));
       
%        Bd_var(4:6,4:6) = diag([dx, dy, dz]);
%        Bd_var(1:3,1:3) = diag([2*sqrt(300) 2*sqrt(300) 2*sqrt(300)]); %rot damping
%        Kd_var(4:6,4:6) = diag([kx, ky, kz]);
%        Kd_var(1:3,1:3) = diag ([300 300 300]); %rot stiffness
       kt = diag([kx;ky;kz]); 
       dt = diag([dx;dy;dz]); 

       Kd_var = blkdiag(kt,kt);
       Bd_var = blkdiag(dt,dt); 
       
       k_data(i,:) = [kx; ky; kz];
       d_data(i,:) = [dx; dy; dz];
       
       %store values
       sres.kd = k_data(i,:)';
       sres.bd = d_data(i,:)';

      %% Compute compliant trajectory
        
        [xd,dxd,ddxd,yr,dyr] = admittance_control(xd1(i,:),dxd1(i,:),ddxd1(i,:),psi_ext,xr,yr_in,dyr_in,Md1,Kd_var,Bd_var,time);
        
        xc_data(i,:) = xd; 
        dxc_data(i,:) = dxd;
        ddxc_data(i,:) = ddxd;
        yr_data(i,:) = yr; 
        dyr_data(i,:) = dyr; 
        
        % Compliant trajectory position,velocity acceleration
        xd_des = xc_data(i,:)';
        dxd_des = dxc_data(i,:)';
        ddxd_des = ddxc_data(i,:)'; 
        
        %Desired trajectory
        xd1_str = xd1(i,:);
        dx1_str = dxd1(i,:);
        ddxd1_str = ddxd1(i,:);
         
        %Ext force (%world_frame)
        fext = w_ext_data(i,1:3)';
        
        % Printing the time step of the simulation and the error
        % -----------------------
        disp(['it: ',num2str(i),' time(',num2str(i*cdt),') - error:',num2str(norm(xd_des-x))])
        disp(['it: ',num2str(i),' time(',num2str(i*cdt),') - pos error:',num2str(norm(vec4(DQ(xd_des).translation-DQ(x).translation)))])
        sres.norm(:,i) = norm(vec4(DQ(xd_des).translation-DQ(x).translation)); 
        
        % Saving data to analyze later
        % -----------------------
        sres.qm(:,i) = qm;  sres.qm_dot(:,i) = qm_dot;  
        sres.xd(:,i) = vec4(DQ(xd_des).translation);  sres.xd_dot(:,i) = dxd_des;  sres.xd_ddot(:,i) = ddxd_des;
        sres.x(:,i) = vec4(DQ(x).translation); 
        sres.xref(:,i) = vec4(DQ(xd1_str).translation);
        sres.fext(:,i) = fext; 
        sres.r(:,i) = r0; 
        
        % -----------------------   
        
        %% Motion Control
        %%Task space inverse dynamics with fb linearization:
        %% u = M(q)*y + c(q,dq) + g(q) 
        
        % Retrieve dyanmics
        g = get_GravityVector(qm);
        c = get_CoriolisVector(qm,qm_dot);
        M = get_MassMatrix(qm);
        tauf = get_FrictionTorque(qm_dot);                
        
        %% Controller gains;
         kp = 1000*0.5;
         kd = 100*0.5;
         ki = 500; %integral gain
         
%          e = xd_des - x;
%          de = dxd_des - dx;
%          ei = de*cdt + e;
%          y = pinv(Jp)*(ddxd_des - Jp_dot*qm_dot  + kp*eye(8)*e + kd*eye(8)*de + 0*ki*eye(8)*ei);
%          tau = M*y + c + g;
                 
        %% Define error 
        %% Invariant error defintiion
        %% e = 1 - x*xd' = (xd - x)*xd' (DQ)
        xe = xd_des - x;
        dxe = dxd_des - x_dot;
        e = haminus8(DQ(C8*xd_des))*xe;
        e_dot = haminus8(DQ(C8*dxd_des))*xe + haminus8(DQ(C8*xd_des))*dxe;
        ei = e_dot*cdt + e; 

        %% define desired closed-loop dynamics
        y =  kd*eye(8)*e_dot +kp*eye(8)*e;

        %% control input task space
        ax = haminus8(DQ(C8*ddxd_des))*xe + 2*haminus8(DQ(C8*dxd_des))*dxe + haminus8(DQ(C8*xd_des))*(ddxd_des - Jp_dot*qm_dot) + y; 
        J_inv = pinv(haminus8(DQ(C8*xd_des))*Jp);
        
        %% control input joint space
        aq = J_inv*ax; 

        %% fb linearization
        tau = M*aq + c + g ;
%         tau = g; 

        %% Null-space controller
        N = haminus8(DQ(xd_des))*DQ.C8*Jp;
        robustpseudoinverse = N'*pinv(N*N' + 0.1*eye(8));
        P = eye(7) - pinv(N)*N; %null-space projector
        D_joints = eye(7)*2;
        tau_null = P*(-D_joints*qm_dot);
        
        %% Torque command
        tau = tau + tau_null;
        
        %store sent torque commands for later
        tau_send = tau;
        sres.tau_send(:,i) = tau_send;
        
        %% Send torques to Vrep
        for j=1:7
            if tau(j)<0
                set_vel = -99999;
            else
                set_vel = 99999;
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

%% Plots results
%% Controller torques commands
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

%% EE position comparison
figure();
plot(tt,sres.xd(2,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,sres.x(2,:),'c','LineWidth',2);
hold on, grid on
plot(tt,sres.xref(2,:),'b','LineWidth',2)
legend('xc','x','xd')
figure();
plot(tt,sres.xd(3,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,sres.x(3,:),'c','LineWidth',2);
hold on,grid on
plot(tt,sres.xref(3,:),'b','LineWidth',2)
legend('yc','y','yd')
figure()
plot(tt,sres.xd(4,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,sres.x(4,:),'c','LineWidth',2);
hold on,grid on
plot(tt,sres.xref(4,:),'b','LineWidth',2)
legend('zc','z','zd')

%% Ext forces
figure()
plot(tt,sres.fext(1,:),'LineWidth',2);
hold on, grid on
plot(tt,sres.fext(2,:),'LineWidth',2);
hold on,grid on
plot(tt,sres.fext(3,:),'LineWidth',2);
