function [k,d] = test_var_gains(phase,F_ext)

%% Description: decrease stiffness (and damping consequently) to test stability of the system.
%%Parameters: F_threshold --> represent the threshold for external
%%disturbances

mass = 1.5; %[kg]
k_default = 1000; %initial stiffness value [N/m]
F_threshold = 0.5; % [N]
lambda = 0.2; %exponential decay value
pc = 0.28; 

persistent ki

if isempty(ki)
    ki = k_default;
end

if phase == 1
    ki = lambda*ki;
else
    ki = k_default; 
end

k = ki;
d = sqrt(4*mass*k);

end




