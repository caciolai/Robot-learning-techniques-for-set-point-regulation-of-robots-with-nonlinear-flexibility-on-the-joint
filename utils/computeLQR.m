function S = computeLQR(params, Q, R, N)
%% Compute S matrix for the LQR-based terminal cost

%% Define symbolic quantities
syms x [8 1] 'real'         % x1 = q1
                            % x2 = q2
                            % x3 = theta1
                            % x4 = theta2
q = [x1; x2];               % x5 = q1_dot
theta = [x3; x4];           % x6 = q2_dot
q_dot = [x5; x6];           % x7 = theta1_dot
theta_dot = [x7; x8];       % x8 = theta2_dot      

syms u [2 1] 'real'

%% Unpack parameters
x_ref = params.x_ref;
q_ref = x_ref(1:2)';
B = params.B;
K1 = params.K1;
K2 = params.K2;
D = params.D;

%% Pack dynamics
phi = q - theta;
%     psi = linearElasticity(phi, K1);
psi = nonlinearElasticity(phi, params);

% Nonlinear dynamics
f = [  % x_dot = [q_dot, theta_dot, q_ddot, theta_ddot]
    q_dot; 
    theta_dot; 
    M(q)\(-psi - c(q, q_dot) - g(q) -D*q_dot); 
    B\(u + psi - D*theta_dot)
];

%% Linearize around (forced) equilibrium point
x_bar = x_ref';  % Equilibrium point
tau_g = g(q_ref); 
u_bar = tau_g; % (forced)

A_sys = double(subs(jacobian(f,x), [x; u], [x_bar; u_bar]));
B_sys = double(subs(jacobian(f,u), [x; u], [x_bar; u_bar]));

%% Compute S matrix from lqr on linearized system
[~,S,~] = lqr(A_sys,B_sys,Q,R,N);

clear x q theta q_dot theta_dot psi fx gx x_bar A_sys B_sys;
end