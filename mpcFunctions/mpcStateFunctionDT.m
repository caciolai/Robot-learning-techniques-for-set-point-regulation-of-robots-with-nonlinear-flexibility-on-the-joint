function x1 = mpcStateFunctionDT(x, u, Ts, B, K1, K2, x_ref, p)

   % Repeat application of Euler method sampled at Ts/M.
    M = 1;
    delta = Ts/M;
    x1 = x;
    for ct=1:M
        x1 = x1 + delta*mpcStateFunctionCT(x1, u, Ts, B, K1, K2, x_ref, p);
    end
end