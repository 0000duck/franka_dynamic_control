function Q8_dot = getQ8_dot(x_E,dx_E)
%description: computes the derivative of Q8 matrix
%inputs : x_E , dx_E dual pose quaternion and its derivative
r = vec4(P(x_E));
dr = vec4(P(dx_E));
p = vec4(2*D(x_E)*P(x_E)');
dp = 2*haminus4(P(DQ(x_E)'))*vec4(D(DQ(dx_E))) + 2*hamiplus4(D(DQ(x_E)))*vec4(P(DQ(dx_E)'));

Q4 = getQ4(DQ(r));
Q4_dot = getQ4_dot(DQ(r),DQ(dr));
Qp = zeros(4,3);
Qp(1,:) = 0;
Qp(2:4,:) = eye(3);

Q8_dot = zeros(8,6);
Q8_dot(1:4,1:3) = Q4_dot;
Q8_dot(5:8,1:3) = 0.5*hamiplus4(DQ(dp))*Q4 + 0.5*hamiplus4(DQ(p))*Q4_dot;
Q8_dot(5:8,4:6) = haminus4(DQ(dr))*Qp;
end