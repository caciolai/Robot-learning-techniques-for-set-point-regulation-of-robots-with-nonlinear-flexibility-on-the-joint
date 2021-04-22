function J = mpcCostFunction(x, u, e, data, Ts, B, K1, K2, D, x_ref, p, S)
    
    u = u(1:p,:);
    x = x(2:p+1,:);
    
    theta_ref = x_ref(3:4);
    reference = ones(p,2) .* theta_ref;
    
    theta = x(:, 3:4);
    theta_dot = x(:, 7:8);

    J = 0.0;
    
    %% Terminal cost
    k = 10;
    
    % negli ultimi k step forza le theta verso la reference
    J = J + 100*(sum(sum(theta(end-k:end,:)-reference(end-k:end,:).^2,1),2));
    
    % negli ultimi k step forza le theta dot a zero (induci stabilità
    % dell'equilibrio)
    J = J + 100*(sum(sum(theta_dot(end-k:end,:).^2,1),2));
    
    % negli ultimi k step riduci il control effort
    J = J + 100*(sum(sum(u(end-k:end,:).^2,1),2));
    
    J = J + x_ref*S*x_ref'; % trasposto perché è un row vector

    %% Stage cost
    for i=1:p-1
        J = J + 10*sum((theta(i,:)-reference(i,:)).^2);
        J = J + 1*sum((theta_dot(i,:)).^2);
%         J = J + 0.1*sum(u(i,:).^2);
        J = J + 0.1*sum((u(i+1,:) - u(i,:)).^2);
    end

end