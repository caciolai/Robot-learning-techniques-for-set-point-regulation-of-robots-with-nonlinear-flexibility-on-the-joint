function cineq = mpcInequalityConstraints(x, u, e, data, params)
% Function that returns a vector of values, one for each inequality
% constraints, satisfied if the value is less than 0
    
x_ref = params.x_ref;
k = params.lastSteps;
eps = params.terminalConstraintEps;

% |theta_k - theta_d| < eps + e
C1 =  abs(x(end-k:end,3) - x_ref(1)) - (eps + e);
C2 =  abs(x(end-k:end,4) - x_ref(2)) - (eps + e);

cineq = [C1;C2];
    
end

