% Initialization
clear all
close all
clc

addpath(genpath('./utils'));
addpath(genpath('./modelFunctions'));
addpath(genpath('./mpcFunctions'));

%% Parameters
% Motor inertia matrix
B_motor = eye(2);

% Stiffness matrices (for elasticity)
K1 = eye(2) * 1000;
K2 = eye(2) * 10;

% Damping
D = eye(2) * 10; 

%% Reference
% desired link position
q_ref = [0 0]';

% SET POINT FOR NONLINEAR ELASTICITY
% vogliamo che all'equilibrio il termine elastico compensi la gravità
g_ref = g(q_ref);

prob = eqnproblem;
x = optimvar('x',2);
eqn1 = -K1(1,1) * (q_ref(1) - x(1)) - K2(1,1) * (q_ref(1) - x(1))^3 - g_ref(1) == 0;
eqn2 = -K1(2,2) * (q_ref(2) - x(2)) - K2(2,2) * (q_ref(2) - x(2))^3 - g_ref(2) == 0;
x0.x = [0.0, 0.0];

prob.Equations.eqn1 = eqn1;
prob.Equations.eqn2 = eqn2;

[sol,fval,exitflag] = solve(prob,x0);
theta_ref = sol.x;

% desired equilibrium (no velocity)
x_ref = [q_ref; theta_ref; zeros(4, 1)]'; % must be row vector

%% Initialize symbolic state vector
syms x [8 1] 'real'         % x1 = q1
                            % x2 = q2
                            % x3 = theta1
                            % x4 = theta2
q = [x1; x2];               % x5 = q1_dot
theta = [x3; x4];           % x6 = q2_dot
q_dot = [x5; x6];           % x7 = theta1_dot
theta_dot = [x7; x8];       % x8 = theta2_dot            

%     psi = linearElasticity(x, K1);
psi = nonlinearElasticity(x, K1, K2);

% Vector fields
fx = [q_dot; theta_dot; M(q)\(-psi - c(q, q_dot) - g(q) -D*(q_dot-theta_dot)); B_motor\(psi-D*(theta_dot-q_dot))];
gx = [zeros(6,2); inv(B_motor)];

%% Linearization
x_bar = x_ref';  % Equilibrium point
% u_bar = 0

A = double(subs(jacobian(fx,x), x, x_bar));
B = double(subs(gx, x, x_bar));

%% LQR
% Weight matrices
Q = eye(8);           % to be tuned
Q(1:4, 1:4) = diag([10,10, 1,1]);
Q(5:8, 5:8) = zeros(4,4);
R = eye(2);           % to be tuned 
N = zeros(8,2);

[K,S,e] = lqr(A,B,Q,R,N);

% Check stability
e; % Closed loop eigenvalues (A-B*K)

%% Closed-loop simulation
x0 = x_bar + [0.01; 0.01; 0.01; 0.01; 0; 0; 0; 0];
Ts = 1e-2; 
T = 10;
xk = x0 - x_bar;
tau_g = g(q_ref);

[b,a] = ss2tf(A,B,eye(8),zeros(8,2))



% for ct = 1:(T/Ts)
%         
%     % Compute optimal lqr moves.
%     [K,S,e] = lqr(A,B,Q,R,N);
%     
%     tau = -K*xk + tau_g;
%     
%     % Implement first optimal control move and update plant states.
%     q = [xk(1); xk(2)];              
%     theta = [xk(3); xk(4)];          
%     q_dot = [xk(5); xk(6)];           
%     theta_dot = [xk(7); xk(8)]; 
%     
%     xk(1) = xk(5)*Ts + xk(1);
%     xk(2) = xk(6)*Ts + xk(2);
%     xk(3) = xk(7)*Ts + xk(3);
%     xk(4) = xk(8)*Ts + xk(4);
%     
%     M_temp = M(q);
%     c_temp = c(q, q_dot);
%     g_temp = g(q);
%     psi_temp = subs(psi, [x1,x2,x3,x4], [xk(1),xk(2),xk(3),xk(4)]);
%     
%     xk(5:6) = M_temp\(-psi_temp - c_temp - g_temp -D*(q_dot-theta_dot))*Ts + xk(5:6);
%     xk(7:8) = B_motor\(psi_temp-D*(theta_dot-q_dot) + tau)*Ts + xk(7:8);
%     
%     % Save plant states for display.
%     xHistory(ct,:) = xk';
%     uHistory(ct,:) = tau';
%     
% end
% 
% %% Plot
% t = linspace(0, T, T/Ts);
% 
% figure
% subplot(2,2,1)
% hold on
% plot(t,xHistory(:,1))
% yline(q_ref(1), 'r-');
% xlabel('time')
% ylabel('q1')
% legend('q1', 'qd1');
% title('First link position')
% 
% subplot(2,2,2)
% hold on
% plot(t,xHistory(:,2))
% yline(q_ref(2), 'r-');
% xlabel('time')
% ylabel('q2')
% legend('q2', 'qd2');
% title('Second link position')
% 
% subplot(2,2,3)
% hold on
% plot(t,xHistory(:,3))
% plot(t,xHistory(:,4))
% xlabel('time')
% ylabel('q_{dot}')
% legend('q_{dot}_1', 'q_{dot}_2');
% title('Link velocities')
% 
% subplot(2,2,4)
% hold on
% plot(t,uHistory(:,1))
% plot(t,uHistory(:,2))
% xlabel('time')
% ylabel('tau')
% legend('tau1', 'tau2');
% title('Controlled torque')
% 
