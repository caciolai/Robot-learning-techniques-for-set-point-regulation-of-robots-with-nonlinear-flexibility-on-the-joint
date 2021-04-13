function [data] = data_gen_single_trajectory(q0, q0_dot,  qd, qd_dot, qd_dotdot, tf, T, n, Kp, Kd, B, k1, k2)
    
    %Initialization
    q=zeros(n,tf*(1/T));
    theta=zeros(n,tf*(1/T));
    q_dot=zeros(n,tf*(1/T));
    theta_dot=zeros(n,tf*(1/T));
    theta_ddot=zeros(n,tf*(1/T));
    psi=zeros(n,tf*(1/T)); % Measured elasticity
    psi_real=zeros(n,tf*(1/T)); % Real elasticity
    q(:,1)=q0;
    theta(:,1)=q0;
    q_dot(:,1)=q0_dot;
    theta_dot(:,1)=q0_dot;
    u(:,1)=[0;0];
    
    for i=1:tf*(1/T)-1        
        % u(:,i+1) =(M(qd(:,i)+ B)*qd_dotdot(:,i) + c(qd(:,i),qd_dot(:,i))+g(qd(:,i)); % Open loop control (Slide De Luca: LagrangiaDyanmic3 p.18)        
        psi_real(:, i) = k1*(q(:,i)-theta(:,i))+k2*(q(:,i)-theta(:,i)).^3;
        u(:,i+1) = B*qd_dotdot(:,i) + Kp*(qd(:,i) - q(:,i)) + Kd*(qd_dot(:,i) - q_dot(:,i));
        q(:,i+1) = T*(q_dot(:,i))+ q(:,i);
        theta(:,i+1) = T*(theta_dot(:,i))+ theta(:,i);
        q_dot(:,i+1) = T*(M(q(:,i))\(-psi_real(:, i) - c(q(:,i),q_dot(:,i)) - g(q(:,i)))) + q_dot(:,i);
        theta_dot(:,i+1) = T*(B\(u(:,i)+psi_real(:, i))) + theta_dot(:,i);
        theta_ddot(:,i+1)=(theta_dot(:,i+1)-theta_dot(:,i))/T; %euler derivative
        psi(:,i)=B*theta_ddot(:,i+1)-u(:,i);
    end
    
    x=[q; theta]; % 4xN ; N=number of sampled points
    y=[psi];     % 2xN
    
    data=[x;y];  % 6xN
    
    
end

