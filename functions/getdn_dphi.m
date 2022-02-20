function [dn,dphi] = getdn_dphi(r,dr)
    phi = rotation_angle(DQ(r));
    n = rotation_axis(DQ(r));
    if phi == 0 
       dphi = 0; 
       dn = zeros(3,1);
    else
        dphi = -double((2*Re(DQ(dr)))/sin(0.5*phi));
        a = vec3(Im(DQ(dr)));
        dn = (a - cos(0.5*phi)*dphi*vec3(n))/sin(0.5*phi);
    end
end


