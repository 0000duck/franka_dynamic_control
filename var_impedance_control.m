%% Panda dynamic impedance control with variable impedance gains.

%% Description: simulation file for dynamic control of a 7 dof Panda Robot in Vrep using Dual Quaternions.
%%Impedance controller: enforce virtual mass-damper-spring system using DQ
%%log mapping of pose displacement.

%% Modulator: change stiffness and damping online to guarantee safe interaction and better tracking perfomances

%% Addpath 
include_namespace_dq;

%% Initialize variables
%%impedance controller (log mapping)
yr_data = zeros(size(time,2),6);
dyr_data =  zeros(size(time,2),6);
ddyr_data =  zeros(size(time,2),6);

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

    %% Load nominal desired trajectory
    [xd1, dxd1, ddxd1] = int_traj(x_in,time);
    
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
        
        %% Impedance control
        % initialize variables
        if i~=1
            x_hat = vec8(DQ(x)'*DQ(xd1(i,:)));
            dx_hat = vec8(DQ(x_dot)'*DQ(xd1(i,:))) + vec8(DQ(x)'*DQ(dxd1(i,:))); 
            yr = vec6(log(DQ(x_hat)));
            Q8 = getQ8(DQ(x_hat));
            dyr = pinv(Q8)*dx_hat; 
%             yr_in = yr_data(i-1,:)';
%             dyr_in = dyr_data(i-1,:)';
            time_prec = time(i-1);
        else %first iteration
            x_hat = vec8(DQ(x)'*DQ(xd1(1,:)));
            dx_hat = zeros(8,1); 
            yr = vec6(log(DQ(x_hat)));
            dyr = zeros(6,1); 
%             yr_in = vec6(log(DQ(x_hat)));
%             dyr_in = zeros(6,1);
            time_prec = 0; 
        end
        
        %% Model ext forces
        wrench_ext = ext_forces(x); %world frame
        w_ext_data(i,:) = wrench_ext;
        %External wrench (w.r.t to EE frame)
        psi_ext = vec6(DQ(r0)'*DQ(wrench_ext)*DQ(r0)); 
        psi_ext = psi_ext'; 
        psi_ext_data(i,:) = psi_ext;
        %%Mapping external wrench to be consisten with log definition 
        Q8 = getQ8(DQ(x_hat));
        Ibar = [zeros(3,1), eye(3), zeros(3,1), zeros(3,3);...
                zeros(3,1), zeros(3,3), zeros(3,1), eye(3)];
        G = getG(DQ(x_hat));
        Glog = Ibar*G*Q8;
        flog = (Glog)'*(psi_ext');

        %% Modulate impedance gains
        %%compute stiffness and damping 
%        [kx,dx] = modulator(time(i),time_prec,curr_pos(1),e_pos(1),f_ext(1),phase_data(i,1));
%        [ky,dy] = modulator(time(i),time_prec,curr_pos(2),e_pos(2),f_ext(2),phase_data(i,2));
%        [kz,dz] = modulator(time(i),time_prec,curr_pos(3),e_pos(3),f_ext(3),phase_data(i,3));

       kx = 100; %N/m
       ky = kx;
       kz = kx; 
       dx = 4*sqrt(kx*1.5);
       dy = dx;
       dz = dx; 
       kt = diag([kx;ky;kz]); 
       dt = diag([dx;dy;dz]); 

       k_rot = diag([300;300;300]); 
       d_rot = diag([42.4264;42.4264;42.4264]); 

       Kd_var = blkdiag(k_rot,kt);
       Bd_var = blkdiag(d_rot,dt); 
       
       k_data(i,:) = [kx; ky; kz];
       d_data(i,:) = [dx; dy; dz];
       
       %store values
       sres.kd = k_data(i,:)';
       sres.bd = d_data(i,:)';

      %% Compute desired cl dyn, 
        ddyr = inv(Md1)*(-Bd_var*dyr - Kd_var*yr-flog);
%         dyr  = ddyr*cdt + dyr_in;
%         yr = dyr*cdt + yr_in;
%         
%         %Update values
%         yr_in = yr;
%         dyr_in = dyr; 
        
        %% Store data
        yr_data(i,:) = yr; 
        dyr_data(i,:) = dyr; 
        ddyr_data(i,:) = ddyr;  
        %Desired trajectory
        xd1_str = xd1(i,:)';
        dxd1_str = dxd1(i,:)';
        ddxd1_str = ddxd1(i,:)'; 
        %Ext force (%world_frame)
        fext = w_ext_data(i,1:3)';
        
        % Saving data to analyze later
        % -----------------------
        sres.qm(:,i) = qm;  sres.qm_dot(:,i) = qm_dot;  
        sres.x(:,i) = vec4(DQ(x).translation); 
        sres.xref(:,i) = vec4(DQ(xd1_str).translation);
        sres.fext(:,i) = fext; 
        sres.r(:,i) = r0; 
        
        % -----------------------   
        
        %%Compute position error
        xe = vec8(DQ(x)'* DQ(xd1(i,:))); %pose displacement
        e = vec4(2*D(log(DQ(xe)))); %position error
        e_pos = [e(2); e(3); e(4)];
        curr_pos = [pos(2);pos(3);pos(4)];  

        %%retrieve external forces
        f_ext = w_ext_data(i,1:3)'; 

        % Printing the time step of the simulation and the error
        % -----------------------
        disp(['it: ',num2str(i),' time(',num2str(i*cdt),') - error:',num2str(norm(xd1_str-x))])
        disp(['it: ',num2str(i),' time(',num2str(i*cdt),') - pos error:',num2str(norm(vec4(DQ(xd1_str).translation-DQ(x).translation)))])
        sres.norm(:,i) = norm(vec4(DQ(xd1_str).translation-DQ(x).translation)); 
        
        %% IMPEDANCE CONTROLLER
        % (can be interpreted as a system producing equivalent end-effector
        % forces (outputs) form the measurements of the motion variables (inputs), thus
        % corresponding to a mechanical impedance.)   

        % Retrieve dynamics
        g = get_GravityVector(qm);
        c = get_CoriolisVector(qm,qm_dot);
        M = get_MassMatrix(qm);
        tauf = get_FrictionTorque(qm_dot);  

        %% Compute desired cl dynamics
        Q8 = getQ8(DQ(x_hat));
        Q8_dot = getQ8_dot(DQ(x_hat),DQ(dx_hat));
        ddx_hat = Q8_dot*dyr + Q8*ddyr; 
        
        %Task space
        ax = hamiplus8(DQ(ddxd1_str))*C8*x_hat + 2*hamiplus8(DQ(dxd1_str))*C8*dx_hat + hamiplus8(DQ(xd1_str))*C8*ddx_hat - Jp_dot*qm_dot; 
        J_inv = pinv(Jp);
        
        %% control input joint space
        aq = J_inv*ax; 

        %% fb linearization
        tau = M*aq + c + g ;

        %% Null-space controller
        N = haminus8(DQ(xd1_str))*DQ.C8*Jp;
        robustpseudoinverse = N'*pinv(N*N' + 0.1*eye(8));
%         P = eye(7) - pinv(N)*N; %null-space projector
        P = eye(7) - pinv(Jp)*Jp; %null-space projector
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
plot(tt,sres.x(2,:),'c','LineWidth',2);
hold on, grid on
plot(tt,sres.xref(2,:),'b','LineWidth',2)
legend('x','xd')
figure();
plot(tt,sres.x(3,:),'c','LineWidth',2);
hold on,grid on
plot(tt,sres.xref(3,:),'b','LineWidth',2)
legend('y','yd')
figure()
plot(tt,sres.x(4,:),'c','LineWidth',2);
hold on,grid on
plot(tt,sres.xref(4,:),'b','LineWidth',2)
legend('z','zd')

%% Ext forces
figure()
plot(tt,sres.fext(1,:),'LineWidth',2);
hold on, grid on
plot(tt,sres.fext(2,:),'LineWidth',2);
hold on,grid on
plot(tt,sres.fext(3,:),'LineWidth',2);

%% Admittance gains
figure()
plot(tt,k_data(:,1)','LineWidth',2);
hold on, grid on
plot(tt,k_data(:,2)','LineWidth',2);
hold on,grid on
plot(tt,k_data(:,3)','LineWidth',2);
legend('kx','ky','kz')
