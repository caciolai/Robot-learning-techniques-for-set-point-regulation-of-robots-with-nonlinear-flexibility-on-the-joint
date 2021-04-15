function x1 = stateFunctionDT(x, u, Ts, B, K1, K2)

%    % Repeat application of Euler method sampled at Ts/M.
%     nIter = 1;
%     delta = Ts/nIter;
%     x1 = x;
%     for ct=1:nIter
%         x1 = x1 + delta*stateFunctionCT(x1,u,B,K1,K2);
%     end
    x1 = x + Ts*stateFunctionCT(x,u,B,K1,K2);
end