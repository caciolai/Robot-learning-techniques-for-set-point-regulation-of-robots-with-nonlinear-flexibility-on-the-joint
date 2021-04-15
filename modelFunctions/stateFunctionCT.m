function x_dot = stateFunctionCT(x,u,B,K1,K2)

    % Unpack state variables
    q = x(1:2);
    theta = x(3:4);
    q_dot = x(5:6);
    theta_dot = x(7:8);
    
    % Compute elastic term
    psi = linearElasticity(x, K1);
%     psi = nonlinearElasticity(x, K1, K2);

    % Dynamic equations of 2R
    q_ddot = M(q)\(-psi - c(q, q_dot) - g(q));
    theta_ddot = B\(psi + u);
    
    x_dot = [q_dot; theta_dot; q_ddot; theta_ddot];

end

