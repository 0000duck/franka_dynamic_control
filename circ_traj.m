function [xd,dxd,ddxd] = circ_traj(x_in,t)

%% Description: generate circular trajectory
xd = [zeros(size(t,2),8)];
dxd = [zeros(size(t,2),8)];
ddxd = [zeros(size(t,2),8)];
%retrieve initial pose
r0 = vec4(P(x_in));
p0 = vec4(x_in.translation);
x0 = p0(2);
y0 = p0(3);
z0 = p0(4);
%define trajectory

i = 1;
for i = 1:size(t,2)
    p = [0; x0; y0 + cos(t(i))/8; z0 + sin(t(i))/8];
    dp = [0; 0; -sin(t(i))/8; cos(t(i))/8];
    ddp = [0; 0; -cos(t(i))/8; - sin(t(i))/8];
    
    %2DQ
    x = vec8(DQ(r0) + 0.5*DQ.E*(DQ(p)*DQ(r0)));
    dx = vec8(0.5*DQ.E*(DQ(dp)*DQ(r0)));
    ddx = vec8(0.5*DQ.E*(DQ(ddp)*DQ(r0)));
    
    xd(i,:) = x;
    dxd(i,:) = dx;
    dxd(i,:) = ddx;
    
    i = i+1;
end