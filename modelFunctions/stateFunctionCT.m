function x_dot = stateFunctionCT(x, u, params)

    % Unpack parameters
    B = params.B;
    K1 = params.K1;
    K2 = params.K2; 
    D = params.D;
    
    % Unpack state variables
    q = x(1:2);
    theta = x(3:4);
    q_dot = x(5:6);
    theta_dot = x(7:8);

    % Compute elastic term
%     psi = linearElasticity(x, K1);
    psi = nonlinearElasticity(x, K1, K2);

    q_ddot = M(q)\(-psi - c(q, q_dot) - g(q) -D*q_dot);
    theta_ddot = B\(psi + u -D*theta_dot);
    
    x_dot = [q_dot; theta_dot; q_ddot; theta_ddot];

end

