function Q4 = getQ4(rot)
% Return the partial derivative of the unit quaternion r with respect to
% log(r).

    r = vec4(rot);
    phi = double(rotation_angle(rot));
    n = vec3(rotation_axis(rot));
    nx = n(1); ny = n(2); nz = n(3);

    if phi == 0
        theta = 1;
    else
        theta = sin(phi/2)/(phi/2);
    end
 
    gamma = r(1) - theta; 

    Q4 = [           -r(2),            -r(3),            -r(4);
           gamma*nx^2+theta,      gamma*nx*ny,      gamma*nx*nz; 
                gamma*nx*ny, gamma*ny^2+theta,      gamma*ny*nz;
                gamma*nz*nx,      gamma*nz*ny, gamma*nz^2+theta];
end