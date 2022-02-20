 
%Dual Quaternion to Matrix
% Takes input of dual quaternion Qm  = Qr+ E Qd

function [T] = DQuaternionToMatrix(Qm)
for i =1 : size(Qm,1)
    
q0 = Qm(i,1);
q1 = Qm(i,2);
q2 = Qm(i,3);
q3 = Qm(i,4);
M (1,1) = q0*q0 + q1*q1 - q2*q2 - q3*q3;
M (1,2) = (2*q1*q2 - 2*q0*q3);
M (1,3) = 2*q1*q3 + 2*q0*q2;
M (2,1) = (2*q1*q2 + 2*q0*q3);
M (2,2) = q0*q0 + q2*q2 - q1*q1 - q3*q3;
M (2,3) = 2*q2*q3 - 2*q0*q1;
M (3,1) = 2*q1*q3 - 2*q0*q2;
M (3,2)=  2*q2*q3 + 2*q0*q1;
M (3,3)= q0*q0 + q3*q3 - q1*q1 - q2*q2;

p = Qm(i,5);
u = Qm(i,6);
r = Qm(i,7);
s = Qm(i,8);

q_d = [p u r s];
q_d_v = [u r s];
q_r_conj = [q0 -q1 -q2 -q3];
q_r_conj_v = [-q1 -q2 -q3];
% q_1 = (p*w - dot(q_d_v, q_r_conj_v))
% q_2 = p*q_r_conj_v + w*q_d_v +cross(q_d_v,q_r_conj_v)
f = QMult(q_d, q_r_conj); % q = q_1 +e q_2


t = 2*f; 

t = t';
t = t(2:4,:);

T(:,:,i) = [M t;0 0 0 1];
% Q_t = 2 * [q_d * q_r_conjugate]
end
end




