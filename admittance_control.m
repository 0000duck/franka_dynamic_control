function [xc,dxc,ddxc,yr,dyr] = admittance_control(xd,dxd,ddxd,psi_ext,xr,yr,dyr,Md,Kd,Bd,time)

%% Description: admittance controller using DQ log mapping to enforce desired impedance behaviour.
%%It computes variables of compliant frame, which is a suitable reference frame describing the ideal behaviour of the end-
% effector under impedance control.

%%Inputs: xd,dxd,ddxd = desired reference trajectory; [8x1]
%         psi_ext = ext wrench on EE with respect to compliant frame; [1x6] 
%         time = simulation time;
%         xr= previous computed reference pose;
%         yr,dyr = previous computed log mapping;
%         Md,Kd,Bd = desired impedance matrices [6x6]

%%Outputs: xc,dxc,ddxc = compliant trajectory using DQ representation [8x1]

cdt = time(2) - time(1); %sampling time 

%% Initialize

x_hat = vec8(DQ(xr)'*DQ(xd)); %pose displacement between desired and compliant frame
y_hat = yr; %log mapping of pose displacement
dy_hat = dyr; %1-st time derivative of y

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

%Update values
yr = y_hat;
dyr = dy_hat;

%retrieve pose displacement
x_hat = vec8(exp(DQ(y_hat)));
Q8 = getQ8(DQ(x_hat));
dx_hat = Q8*dy_hat;
Q8_dot = getQ8_dot(DQ(x_hat),DQ(dx_hat));
ddx_hat = Q8*ddy_hat + Q8_dot*dy_hat;

%retrieve compliant trajectory
xc = hamiplus8(DQ(xd))*DQ.C8*(x_hat);
dxc = hamiplus8(DQ(dxd))*DQ.C8*(x_hat) + hamiplus8(DQ(xd))*DQ.C8*(dx_hat);
ddxc = hamiplus8(DQ(ddxd))*DQ.C8*(x_hat) + 2*hamiplus8(DQ(dxd))*DQ.C8*(dx_hat) + hamiplus8(DQ(xd))*DQ.C8*ddx_hat;


end



