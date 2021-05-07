function [data] = data_gen_single_trajectory(q0, q0_dot,...
    qd, qd_dot, qd_ddot, ...
    params)
    
    % parameters
    T = params.dgT;
    Ts = params.dgTs;
    K1 = params.K1;
    K2 = params.K2;
    B = params.B;
    D = params.D;
    
    nLinks = length(q0);
    nSamples = T*(1/Ts);
    %Initialization
    q=zeros(nLinks, nSamples);
    theta=zeros(nLinks, nSamples);
    q_dot=zeros(nLinks, nSamples);
    theta_dot=zeros(nLinks, nSamples);
    theta_ddot=zeros(nLinks, nSamples);
    psi=zeros(nLinks, nSamples); % Measured elasticity
    psi_real=zeros(nLinks, nSamples); % Real elasticity
    
    q(:,1)=q0;
    theta(:,1)=q0;
    q_dot(:,1)=q0_dot;
    theta_dot(:,1)=q0_dot;
    u(:,1)=[0;0];
    
    xk = [q(:,1); theta(:,1); q_dot(:,1); theta_dot(:,1)];
    
    for k=1:T*(1/Ts)-1        
        % Open loop control (Slide De Luca: LagrangiaDyanmics3 p.18)        
        u(:,k+1) =(M(qd(:,k))+ B)*qd_ddot(:,k) + c(qd(:,k),qd_dot(:,k))+g(qd(:,k)) + D*qd_dot(:,k); 
        psi_real(:, k) = nonlinearElasticity(xk, K1, K2);

        % Dynamics
        paramTs = params.Ts;
        params.Ts = Ts;
        xk = stateFunctionDT(xk, u(:,k), params);
        params.Ts = paramTs;
        
        q(:,k+1) = xk(1:2);
        theta(:,k+1) = xk(3:4);
        q_dot(:,k+1) = xk(5:6);
        theta_dot(:,k+1) = xk(7:8);
        
        % Reconstruct psi
        theta_ddot(:,k+1)=(theta_dot(:,k+1)-theta_dot(:,k))/Ts; %euler derivative
        psi(:,k)=B*theta_ddot(:,k+1)+D*theta_dot(:,k)-u(:,k);
    end
    
    input=[theta - q]; % 2xN ; N=number of sampled points
    output=[psi];      % 2xN
    
    data=[input;output;psi_real];  % 6xN
    
    
end

