function [data] = data_gen_single_trajectory(q0, q0_dot,  qd, qd_dot, qd_dotdot, T, Ts, B, K1, K2, D)
    
    n = length(q0);
    %Initialization
    q=zeros(n,T*(1/Ts));
    theta=zeros(n,T*(1/Ts));
    q_dot=zeros(n,T*(1/Ts));
    theta_dot=zeros(n,T*(1/Ts));
    theta_ddot=zeros(n,T*(1/Ts));
    psi=zeros(n,T*(1/Ts)); % Measured elasticity
    psi_real=zeros(n,T*(1/Ts)); % Real elasticity
    q(:,1)=q0;
    theta(:,1)=q0;
    q_dot(:,1)=q0_dot;
    theta_dot(:,1)=q0_dot;
    u(:,1)=[0;0];
    
    for i=1:T*(1/Ts)-1        
        u(:,i+1) =(M(qd(:,i))+ B)*qd_dotdot(:,i) + c(qd(:,i),qd_dot(:,i))+g(qd(:,i)) + D*qd_dot(:,i); % Open loop control (Slide De Luca: LagrangiaDyanmics3 p.18)        
        psi_real(:, i) = K1*(q(:,i)-theta(:,i))+K2*(q(:,i)-theta(:,i)).^3;
%         u(:,i+1) = B*qd_dotdot(:,i) + Kp*(qd(:,i) - q(:,i)) + Kd*(qd_dot(:,i) - q_dot(:,i));
        q(:,i+1) = Ts*(q_dot(:,i))+ q(:,i);
        theta(:,i+1) = Ts*(theta_dot(:,i))+ theta(:,i);
        q_dot(:,i+1) = Ts*(M(q(:,i))\(-psi_real(:, i) - c(q(:,i),q_dot(:,i)) - g(q(:,i)) - D*q_dot(:,i))) + q_dot(:,i);
        theta_dot(:,i+1) = Ts*(B\(u(:,i) + psi_real(:, i) - D*theta_dot(:,i))) + theta_dot(:,i);
        theta_ddot(:,i+1)=(theta_dot(:,i+1)-theta_dot(:,i))/Ts; %euler derivative
        psi(:,i)=B*theta_ddot(:,i+1)+D*theta_dot(:,i)-u(:,i);
    end
    
    x=[q; theta]; % 4xN ; N=number of sampled points
    y=[psi];     % 2xN
    
    data=[x;y;psi_real];  % 6xN
    
    
end

