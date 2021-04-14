function x1 = stateFunctionDT(x, u, Ts, B, K1, K2)

   % Repeat application of Euler method sampled at Ts/M.
    M = 1;
    delta = Ts/M;
    x1 = x;
    for ct=1:M
        x1 = x1 + delta*stateFunctionCT(x1,u,B,K1,K2);
    end
end