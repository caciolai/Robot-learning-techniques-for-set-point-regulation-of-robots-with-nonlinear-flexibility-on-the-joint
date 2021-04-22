function x1 = mpcStateFunctionDT(x, u, Ts, B, K1, K2, D, x_ref, p, S)

%    % Repeat application of Euler method sampled at Ts/M.
%     nIter = 1;
%     delta = Ts/nIter;
%     x1 = x;
%     for ct=1:nIter
%         x1 = x1 + delta*mpcStateFunctionCT(x1, u, Ts, B, K1, K2, x_ref, p);
%     end
    x1 = x + Ts*mpcStateFunctionCT(x, u, Ts, B, K1, K2, D, x_ref, p);
end