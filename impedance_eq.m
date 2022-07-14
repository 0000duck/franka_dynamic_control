function [yr,dyr,ddyr,x_hat] = impedance_eq(xd,psi_ext,x,yr,dyr,Md,Kd,Bd,time)

%% Description: impedance controller using DQ log mapping to enforce desired impedance behaviour.

%%Inputs: xd,dxd,ddxd = desired reference trajectory; [8x1]
%         psi_ext = ext wrench on EE with respect to compliant frame; [1x6] 
%         time = simulation time;
%         x = current EE-pose;
%         yr,dyr = previous computed log mapping;
%         Md,Kd,Bd = desired impedance matrices [6x6]

%%Outputs: xc,dxc,ddxc = compliant trajectory using DQ representation [8x1]

cdt = time(2) - time(1); %sampling time 

%% Initialize

x_hat = vec8(DQ(x)'*DQ(xd)); %pose displacement between desired and compliant frame
y_hat = yr; %log mapping of pose displacement
dy_hat = dyr; %1st time derivative of y

%% Mapping external wrench to be consistent with DQ log definition

Q8 = getQ8(DQ(x_hat));
Ibar = [zeros(3,1), eye(3), zeros(3,1), zeros(3,3);...
    zeros(3,1), zeros(3,3), zeros(3,1), eye(3)];
G = getG(DQ(x_hat));
Glog = Ibar*G*Q8;
flog = (Glog)'*(psi_ext');

%% Admittance equation

ddy_hat = inv(Md)*(-Bd*dy_hat-Kd*y_hat-flog);
dy_hat  = ddy_hat*cdt + dy_hat;
y_hat = dy_hat*cdt + y_hat;
x_hat = vec8(exp(DQ(y_hat)));

%Update values
yr = y_hat;
dyr = dy_hat;
ddyr = ddy_hat; 

end



