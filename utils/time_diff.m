function dExpr = time_diff(expr, x, x_dot)
% TIME_DIFF  Perform time differentiation of an expression.
%   diffExpr = TIME_DIFF(expr, v, v_dot) performs the time differentiation
%   of expr, which contains variables v and we want to represent their time
%   derivative with variables v_dot.

    % symfun variables, depend on time
    t = sym('t');
    syms xt(t) [length(x) 1]
    xt_dot = diff(xt, t);

    dExpr = subs(diff(subs(expr, x, xt), t), [xt, xt_dot], [x, x_dot]);
end

