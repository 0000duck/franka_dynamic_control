function [k,d] = modulator(time,pos,e_pos,F_ext,phase)

%% Description: Modulate stiffness and damping on each cartesian dof of the impedance controller to guarantee good 
%%              tracking perfomance and safe interaction. 

%%Inputs:     pos = current position [1x1]
%             e_pos = position erro [1x1]
%             F_ext = external force on EE; [6x1]
%             phase = flag acknowledging the current desired trajectory phase
%%Outputs:    k = value of stiffness on single d.o.f;
%             d = value of damping on single d.o.f.

%%Parameters: F_max = treshold of disturbance
%             F_int_max = maximum interaction force
%             k_default = default value of stiffness
%             a0,berta,csi = parameters from stability conditions
%             mass = apparent mass

cdt = time(2) - time(1); 

%% Initialization
persistent ki int time_prec

% persistent initialization
if isempty(ki)
    ki = k_default;
end

if isempty(int)
    int = 1000;
end

if isempty(time_prec)
    time_prec = 0;
end


%% Compute k
if phase == 0
    if abs(F_ext) > F_max %contact is detected
        ki = 0.995*ki; %arbitrarly decrease k 
        if int == 1000 %contact position not set yet
            int = pos;
        end
        % if interaction force is higher than threshold
        if ki*abs(e_pos) > F_int_max
            % set k
            ktemp_x = F_int_max/abs(e_pos);
            if ktemp_x < ki
                ki = ktemp_x;
            end
        end
    end
else
    if pos > int + 0.005 %safe to increase k
        k_dot = berta*(4*a0*sqrt(ki/mass)*(ki)^(3/2))/(sqrt(ki) + 2*a0*csi*sqrt(ki)); %maximum variation within stability conditions 
        k_tempx = ki + k_dot*(time(i) -time_prec);
        ki = k_tempx;
        if k_tempx > k_default
            ki = k_default;
        end
    end
end

k = ki;
time_prec = time(i);

d = sqrt(4*mass*k); 

end

