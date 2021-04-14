function J = mpcCostFunction(x, u, e, data, Ts, B, K1, K2, x_ref, p)

 reference = ones(p,2) .* theta_ref;
 
 u = u(1:p,:);
    
 x = x(2:p+1,:);
    
 J = 0.0;
 
 % COSTO TERMINALE
% somma dell'errore al quadrato tra reference del motore e posizione
% del motore
% imponiamo che alla fine della traiettoria theta sia uguale alla
% reference
 J = J + 100*(sum(x(end,3:4)-reference(end,1:2)).^2);

% imponiamo che gli ultimi step della traiettoria theta_dot sia nullo
% per cercare di rendere stabile la configurazione di reference
 J = J + 100*(sum(sum(x(end-5:end,7:8).^2,1),2));

%% FUNZIONE DI COSTO
for i=1:p-1
%         J = J + 1*sum((x(i,1:2)-reference(i,1:2)).^2);
    J = J + 10*sum((x(i,3:4)-reference(i,1:2)).^2);
    J = J + 1*sum((x(i,7:8)).^2);
%         J = J + 1*sum(u(i,:).^2);
    J = J + 0.1*sum((u(i+1,:) - u(i,:)).^2); % minimizza il control effort
    
end

end