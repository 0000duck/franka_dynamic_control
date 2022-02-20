function Omega = getOmega(x)
    %get matrix omega mapping between derivative of unit dual qaut x and pure quat representing angular velociry
    % vec(w) = Omega*vec(dot_x)
    r = P(x);
    O = zeros(4,4);
    Omega = [2*haminus4(r'), O];
end
