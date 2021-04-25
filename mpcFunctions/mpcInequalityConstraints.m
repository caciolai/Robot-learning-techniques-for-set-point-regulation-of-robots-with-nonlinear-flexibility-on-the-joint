function cineq = mpcInequalityConstraints(x, u, e, data, params)
    %Function that returns a vector of values, one for each inequality
    %constraints, satisfied if the value is less than 0
    
    x_ref = params.x_ref;
    k = params.lastSteps;
     
    C1 =  abs(x(end-k:end,3) - x_ref(1)) - 0.01;
    C2 =  abs(x(end-k:end,4) - x_ref(2)) - 0.01;
    cineq = [C1;C2];
    
end

