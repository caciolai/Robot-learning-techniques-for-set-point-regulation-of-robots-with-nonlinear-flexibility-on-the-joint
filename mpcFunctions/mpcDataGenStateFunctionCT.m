function x_dot = mpcDataGenStateFunctionCT(x, u, params)

    load('data_mpc.mat');
    data_mpc = [input; output];

    % Unpack params
    B = params.B;
    K1 = params.K1;
    K2 = params.K2;
    D = params.D;
    x_ref = params.x_ref;   

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
    psi = nonlinearElasticity(x, K1, K2);
    
    % Adding point to dataset
    data_point = [theta - q; psi];    
    data_mpc(:, end + 1) = data_point;
    input = data_mpc(1:2, :);
    output = data_mpc(3:4, :);
    
    save('..\savedData\data_mpc.mat','input','output');

    %% Dynamics
    q_ddot = M(q)\(-psi - c(q, q_dot) - g(q) -D*q_dot);
    theta_ddot = B\(psi + u + u_nominal -D*theta_dot);
    
    x_dot = [q_dot; theta_dot; q_ddot; theta_ddot];

end

