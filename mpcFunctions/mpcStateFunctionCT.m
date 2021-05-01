function x_dot = mpcStateFunctionCT(x, u, params)

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
    psi = nonlinearElasticity(x, K1, K2);
    
    %% Using gp prediction
%     [psi_pred_1,~,~] = predict(gpMdl{1}, [q;theta]');
%     [psi_pred_2,~,~] = predict(gpMdl{2}, [q;theta]');
%     psi = [psi_pred_1;psi_pred_2];
%      
    
    %% Using NN prediction
%     psi = model([q;theta]);
    
    %% Check error   
%     psi_real = nonlinearElasticity(x, K1, K2);
%     error = sqrt((psi_real - psi)'*(psi_real - psi));
%     
%     if error > 1
%         fprintf("error: %.4f\n", error);
%         fprintf("Bad model :(\n");
%     end  
    
    %% Dynamics
    q_ddot = M(q)\(-psi - c(q, q_dot) - g(q) -D*q_dot);
    theta_ddot = B\(psi + u + u_nominal -D*theta_dot);
    
    x_dot = [q_dot; theta_dot; q_ddot; theta_ddot];

end

