function wrench_ext = fext_grasp(x,grasp)
    %% Description: model external forces acting on the end effectors
    %% Assumptions: elastic reaction of the environment; negligible external torques
    % Output: wrench_ext = vector 6x1 representing the external wrench on EE (world frame)
    % Inputs: x = current EE pose;
    %         grasp = flag acknowleding grasping phase; 
    
    %Intialize
    F_ext = 0; 
    
    %Parameters
    k_table = 5000; %N/m, environment stiffness
    pc = 0.35; %contact position with object (z axis)
    mass_obj = 0.5; %kg (object's mass)
    g = 9.81; %m/s^2 (gravity vector)

    %retrieve position 
    x_pos = vec4(DQ(x).translation); %EE position
    z = [x_pos(2); x_pos(3); x_pos(4)];

    if grasp == 0
        if z(3) < pc
            F_ext = - k_table*(z(3) - pc);   % reaction force of the table
        end
    else 
        F_ext = -mass_obj*g;
        if z(3) < pc
            F_ext = - k_table*(z(3)- pc);       % in grasping on the table
        else 
            if (abs(k_table*(z(3)-pc)) < abs(mass_obj*g))
                F_ext = - k_table*(z(3)- pc); 
            else 
                F_ext = - mass_obj*g;
            end    
        end
    end
    
    wrench_ext = [0;0;F_ext;0;0;0];
    
end