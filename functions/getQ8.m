function Q8 = getQ8(x)

    if ~is_unit(x)
        error('Q8 function is defined only for unit dual quaternions');
    end

    r = rotation(x);
    p = translation(x);

    Q = getQ4(r);
    Qp = [zeros(1,3);
          eye(3)];

    Q8 = [Q, zeros(4,3);
           0.5*hamiplus4(p)*Q, haminus4(r)*Qp];

end