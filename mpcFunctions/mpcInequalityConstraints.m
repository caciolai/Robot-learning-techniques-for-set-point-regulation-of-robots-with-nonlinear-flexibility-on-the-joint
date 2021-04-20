function cineq = mpcInequalityConstraints(x, u, e, data, Ts, B, K1, K2, D, x_ref, p)
    %Function that returns a vector of values, one for each inequality
    %constraints, satisfied if the value is less than 0
    
     C1 =  abs(x(end-10:end,3) - x_ref(1)) - 0.01;
     C2 =  abs(x(end-10:end,4) - x_ref(2)) - 0.01;
     cineq = [C1;C2];
    
end

