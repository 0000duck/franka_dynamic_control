function [q] = QMult( q1,q2)
% q1, q2 are unit quaternions with scalar and vector parts
% q1 = w + xi + yj + zk 
% q2 = s + ti + uj + vk

w = q1(1,1);
q1_vector = q1(1,2:4);

s = q2(1,1);
q2_vector = q2(1,2:4);

% Multiplication of unit quaternion

q_real = w*s - dot(q1_vector,q2_vector);
q_vector = w*q2_vector + s*q1_vector + cross (q1_vector,q2_vector);
q = [q_real q_vector];
end

