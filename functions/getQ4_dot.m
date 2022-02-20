function Q4_dot = getQ4_dot(r,dr)
%% description: computes the derivative of Q4 matrix
%inputs : r,dr unit quaternion expressing rotation and its derivative
%Q4_dot = zeros(4,3);

r = vec4(r);
dr = vec4(dr);
phi = double(rotation_angle(DQ(r)));
n = vec3(rotation_axis(DQ(r)));
[dn, ~] = getdn_dphi(r,dr);
[~,dphi] = getdn_dphi(r,dr);

if phi == 0
    theta = 1;
    dtheta = 0;
else
    theta = (sin(phi*0.5))/(phi*0.5);
    dtheta = (cos(phi/2)*dphi)/phi - (2*sin(phi/2)*dphi)/phi^2;
end
gamma = r(1)- theta;
dgamma = dr(1)- dtheta;
Q4_dot = [  -dr(2),                                                 -dr(3),                                                  -dr(4);...
    n(1)^2*dgamma + dtheta + 2*gamma*n(1)*dn(1),            gamma*n(1)*dn(2) + gamma*n(2)*dn(1) + n(1)*n(2)*dgamma, gamma*n(1)*dn(3) + gamma*n(3)*dn(1) + n(1)*n(3)*dgamma;...
    gamma*n(1)*dn(2) + gamma*n(2)*dn(1) + n(1)*n(2)*dgamma, n(2)^2*dgamma + dtheta + 2*gamma*n(2)*dn(2),            gamma*n(2)*dn(3) + gamma*n(3)*dn(2) + n(2)*n(3)*dgamma;...
    gamma*n(1)*dn(3) + gamma*n(3)*dn(1) + n(1)*n(3)*dgamma, gamma*n(2)*dn(3) + gamma*n(3)*dn(2) + n(2)*n(3)*dgamma, n(3)^2*dgamma + dtheta + 2*gamma*n(3)*dn(3)];
end


