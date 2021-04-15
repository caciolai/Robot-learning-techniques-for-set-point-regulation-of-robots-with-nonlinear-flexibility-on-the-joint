function x_dot = mpcStateFunctionCT(x, u, Ts, B, K1, K2, x_ref, m)

    % Unpack state variables
    q = x(1:2);
    theta = x(3:4);
    q_dot = x(5:6);
    theta_dot = x(7:8);
    
    % IMPORTANTE: deve essere messo anche nel modello dell'MPC per
    % "fargli sapere" che stiamo aggiungendo questo termine nominale
    q_ref = x_ref(1:2)';
    u_nominal = g(q_ref);

    % Compute elastic term
    psi = linearElasticity(x, K1);
%     psi = nonlinearElasticity(x, K1, K2);

    q_ddot = M(q)\(-psi - c(q, q_dot) - g(q));
    theta_ddot = B\(psi + u + u_nominal);
    
    x_dot = [q_dot; theta_dot; q_ddot; theta_ddot];

end

