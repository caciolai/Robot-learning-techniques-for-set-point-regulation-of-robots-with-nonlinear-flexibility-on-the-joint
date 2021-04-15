function J = mpcCostFunction(x, u, e, data, Ts, B, K1, K2, x_ref, m)
    
    theta_ref = x_ref(3:4);
    reference = ones(m,2) .* theta_ref;
    
    q = x(:, 1:2);
    theta = x(:, 3:4);

    u = u(1:m,:);
    x = x(2:m+1,:);

    J = 0.0;
    % COSTO TERMINALE
    % somma dell'errore al quadrato tra reference del motore e posizione
    % del motore
    % imponiamo che alla fine della traiettoria theta sia uguale alla
    % reference

    J = J + 100*(sum(x(end,3:4)-reference(end,1:2)).^2);

    % imponiamo che gli ultimi k step della traiettoria theta_dot sia nullo
    % per cercare di rendere stabile la configurazione di reference
    k = 2;
    J = J + 100*(sum(sum(x(end-k:end,7:8).^2,1),2));

    %% FUNZIONE DI COSTO
    for i=1:m-1
    %         J = J + 1*sum((x(i,1:2)-reference(i,1:2)).^2);
        J = J + 10*sum((x(i,3:4)-reference(i,1:2)).^2);
        J = J + 1*sum((x(i,7:8)).^2);
    %         J = J + 1*sum(u(i,:).^2);
        J = J + 0.1*sum((u(i+1,:) - u(i,:)).^2); % minimizza il control effort

    end

end