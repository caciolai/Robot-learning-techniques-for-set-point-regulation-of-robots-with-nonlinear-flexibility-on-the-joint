function S = computeLQR(params, Q, R, N)
    syms x [8 1] 'real'         % x1 = q1
                                % x2 = q2
                                % x3 = theta1
                                % x4 = theta2
    q = [x1; x2];               % x5 = q1_dot
    theta = [x3; x4];           % x6 = q2_dot
    q_dot = [x5; x6];           % x7 = theta1_dot
    theta_dot = [x7; x8];       % x8 = theta2_dot            

    x_ref = params.x_ref;
    B = params.B;
    K1 = params.K1;
    K2 = params.K2;
    D = params.D;
    
    %     psi = linearElasticity(x, K1);
    psi = nonlinearElasticity(x, K1, K2);

    % Vector fields
    fx = [q_dot; theta_dot; M(q)\(-psi - c(q, q_dot) - g(q) -D*(q_dot-theta_dot)); B\(psi-D*(theta_dot-q_dot))];
    gx = [zeros(6,2); inv(B)];

    x_bar = x_ref';  % Equilibrium point
    % u_bar = 0

    A_sys = double(subs(jacobian(fx,x), x, x_bar));
    B_sys = double(subs(gx, x, x_bar));

    % lqr for terminal cost
    [~,S,~] = lqr(A_sys,B_sys,Q,R,N);
    
    clear x q theta q_dot theta_dot psi fx gx x_bar A_sys B_sys;
end