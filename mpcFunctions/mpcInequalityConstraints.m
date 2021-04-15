function cineq = mpcInequalityConstraints(x, u, e, data, Ts, B, K1, K2, x_ref, m)
    %Function that returns a vector of values, one for each inequality
    %constraints, satisfied if the value is less than 0
    
%     C1 =  x(end,3) - (theta_ref(1) + 0.2);
%     C2 = -x(end,3) + (theta_ref(1) - 0.2);
%     C3 =  x(end,4) - (theta_ref(2) + 0.2);
%     C4 = -x(end,4) - (theta_ref(2) - 0.2);
    
    % imponiamo che le theta stiano in un quadrato di lato 0.2
    % centrato nella reference del motore
    
    theta_ref = x_ref(3:4)';

    % abs(theta_1 - theta^d_1) < 0.2
    C1 =  abs(x(end,3) - theta_ref(1)) - 0.2;
    
    % abs(theta_2 - theta^d_2) < 0.2
    C2 =  abs(x(end,4) - theta_ref(2)) - 0.2;
    
%     cineq = [C1;C2;C3;C4];
    
    cineq = [C1;C2];
    
end

