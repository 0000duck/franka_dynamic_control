function G = getG(x)
%description: computes the generalized Jacobian relating the dual twist to the derivative of the dual pose of end-effector.
% vec(psi) = G*vec(dx)
% vec(tE) = G'vec(fe) --> tE generalized wrench, fE pure dq wrench at the end-effector
G = zeros(8,8); 
PSI = getPsi(x); 
OMEGA = getOmega(x);
G = [PSI;OMEGA];
end
