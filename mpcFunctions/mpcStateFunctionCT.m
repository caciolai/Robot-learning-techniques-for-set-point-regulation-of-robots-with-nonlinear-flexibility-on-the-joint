function x_dot = mpcStateFunctionCT(x, u, params)
    
    tic
    
    % Unpack params
    B = params.B;
    K1 = params.K1;
    K2 = params.K2;
    D = params.D;
    x_ref = params.x_ref;
    model = params.model;

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
%     psi = linearElasticity(x, K1);
%     psi = nonlinearElasticity(q-theta, params);
    
    %% Using gp prediction
    psi = gpPredict(x, model);
    
    %% Using NN prediction
%     psi = model([q;theta]); 

    %% Dynamics
    q_ddot = M(q)\(-psi - c(q, q_dot) - g(q) -D*q_dot);
    theta_ddot = B\(psi + u + u_nominal -D*theta_dot);
    
    x_dot = [q_dot; theta_dot; q_ddot; theta_ddot];
    
    toc
end

