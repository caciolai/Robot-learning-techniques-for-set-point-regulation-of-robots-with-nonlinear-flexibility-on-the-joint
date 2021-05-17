function [x1, x_dot] = stateFunctionDT(x, u, params)
    Ts = params.Ts;
%    % Repeat application of Euler method sampled at Ts/M.
%     nIter = 1;
%     delta = Ts/nIter;
%     x1 = x;
%     for ct=1:nIter
%         x1 = x1 + delta*stateFunctionCT(x1,u,B,K1,K2,D);
%     end
    x_dot = stateFunctionCT(x,u,params);
    x1 = x + Ts*x_dot;
end