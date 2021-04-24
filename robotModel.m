%% n-R PLANAR JOINT ROBOT MODEL under gravity
addpath(genpath('./utils'));
addpath(genpath('./modelFunctions'));
parameters;

%% Declaration of symbolic variables
% The state q can be partioned in (q1, q2) where 
% q1 are the positions of the two links
% q2 are the positions of the two motors

q = sym('q', [nLinks, 1], 'real');
theta = sym('theta', [nLinks, 1], 'real');
q_dot = sym('q_dot', [nLinks, 1], 'real');
theta_dot = sym('theta_dot', [nLinks, 1], 'real');

%% DYNAMIC PARAMETERS
m = ones([nLinks 1]); % masses
l = ones([nLinks 1]); % link lengths
d = 0.4*ones([nLinks 1]); % distance (>0) of the center of mass from O_RF_i
I = zeros(3,3,nLinks); % the n inertia matrices
for k=1:nLinks
    I(:,:,k) = eye(3);
end

%% Direct kinematics
tic

% FOR 2R
DH_table = [
    0,      0,      l(1),   q(1);
    0,      0,      l(2),   q(2)
];

% Ai homogeneous transformation matrix from RF_{i-1} to RF_i
A = zeros([4 4 nLinks], 'sym');
for i=1:nLinks
    alpha_i = DH_table(i, 1);
    a_i = DH_table(i, 2);
    d_i = DH_table(i, 3);
    theta_i = DH_table(i, 4);
    A(:, :, i) = dh_matrix(alpha_i, a_i, d_i, theta_i);
end

% Rotation matrices
R = zeros([3 3 nLinks], 'sym');
for i=1:nLinks
    R(:, :, i) = A(1:3, 1:3, i);
end

% Positions of the center of masses of link i, expressed in RF_i so that
% they are constant (and negative).

% FOR 2R
rc(:, 1) = [-d(1) 0 0]';
rc(:, 2) = [-d(2) 0 0]';


% Positions of CoM_i expressed in RF_0
rc0 = zeros([3 nLinks], 'sym');
A0i = eye(4);
for i=1:nLinks
    A0i = A0i * A(:, :, i);
    rc0_i_hom = A0i * [rc(:, i); 1];
    rc0(:, i) = rc0_i_hom(1:3);
end

% Distances of RF_i from RF_{i-1}, expressed in RF_i so that they are
% constant

% FOR 2R
r(:, 1) = [l(1) 0 0]'; 
r(:, 2) = [l(2) 0 0]'; 

disp("Relevant kinematic quantities computed.");
toc
%% Derivation of the model: Kinetic energy of the links
tic
% Moving frame algorithm

% Initialization of previous velocities.
w_prev = [0 0 0]';
v_prev = [0 0 0]';

T = 0; % kinetic energy of the links

% constant z vector
z = [0 0 1]';
disp("Deriving kinetic energy of the links...");
for i = 1:nLinks
    Ri = R(:, :, i);    % Rotation matrix from RF_{i-1} to RF_i
    ri = r(:, i);       % Distance of RF_i from RF_{i-1} expressed in RF_i
    rci = rc(:, i);     % Position of CoM of link i expressed in RF_i
    % recursive formula for omega_i
    wi = Ri' * (w_prev + (q_dot(i) * z));

    % recursive formula for v_i
    vi = Ri' * v_prev + cross(wi, ri);

    % formula for vc_i
    vci = vi + cross(wi, rci);

    wi = simplify(collect(wi, q_dot(i)));
    vci = simplify(collect(vci, q_dot(i)));
    
    Ti = (1/2) * m(i) * (vci'*vci) + (1/2) * (wi' * I(:, :, i) * wi);
    Ti = simplify(collect(Ti, q_dot), 'steps', 100);
%     disp(sprintf("The kinetic energy of link %i is:", i));
%     disp(Ti);
    T = T + Ti;
    
    w_prev = wi;
    v_prev = vi;
end
disp("Done.");
% disp("The kinetic energy of the links is:");
% disp(T);
toc

%% Derivation of the model: Potential (gravitational) energy of the links
tic
U_g = 0;
disp("Deriving gravitational potential energy...");
for i = 1:nLinks
    U_gi = -m(i) * (g0' * rc0(:, i));
    U_g = U_g + U_gi;
end
disp("Done.");
% disp("The gravitational potential energy is:");
% disp(U_g);
toc

%% Derivation of the model: Rigid manipulator inertia matrix
% Extract entries of inertia matrix
tic
disp("Deriving rigid manipulator inertia matrix...");
M = zeros([nLinks nLinks], 'sym');
for i = 1:nLinks
    for j = 1:nLinks
        % T = 1/2 q_dot' M q_dot = sum_i sum_j M
        % therefore to extract M_ij we 
        M(i,j) = diff(diff(T,q_dot(i)),q_dot(j));
    end
end
disp("Done.");
toc
% disp("The rigid manipulator inertia matrix is:");
% disp(M);

%% Derivation of the model: Rigid manipulator Coriolis term
tic
disp("Deriving Coriolis and centrifugal term...");
C = zeros([nLinks 1], 'sym');
for k=1:nLinks
    C_k = 0;
    for i=1:nLinks
        for j=1:nLinks
            c_kij = (1/2)*(diff(M(k, j), q(i)) + diff(M(k, i), q(j)) - diff(M(i, j), q(k)));
            C_k = C_k + (c_kij * q_dot(i) * q_dot(j));
        end
    end
    C(k) = C_k;
end
disp("Done.");
toc

%% Derivation of the model: Rigid manipulator Coriolis factorization matrix
tic
disp("Deriving Coriolis factorization matrix...");
S = zeros([nLinks nLinks], 'sym');
for k=1:nLinks
    for j=1:nLinks
        S_kj = 0;
        for i=1:nLinks
           c_kij = (1/2)*( diff(M(k, j), q(i)) + diff(M(k, i), q(j)) - diff(M(i, j), q(k)));
           S_kj = S_kj + (c_kij * q_dot(i));
        end
        S(k, j) = S_kj;
    end
end

%% Derivation of the model: Gravity term
tic
disp("Deriving gravity term...");
G = jacobian(U_g, q)';

G = simplify(G, 'steps', 100);
disp("Done.");
toc
% disp("The rigid manipulator gravity term is:");
% disp(G);

%% Consistency checks
disp("Checking consistency...");

% symbolic x to check for things
syms x [nLinks 1] real

% Check that M is symmetric
check = simplify(M - M', 'steps', 100);
if (check == 0)
    disp("M is symmetric... OK!");
else
    disp("M is symmetric... NOT OK!");
end

% Check that M is positive definite
try chol(M);
    disp('M is symmetric positive definite... OK!');
catch ME
    disp('M is symmetric positive definite... NOT OK!');
end


% Check that q1 does not appear in M
check = simplify(x'*diff(M, q(1))*x, 'steps', 100);
if (check == 0)
    disp("M does not contain q1... OK!");
else
    disp("M does not contain q1... NOT OK!");
end

% Check that M_nn is constant
check = simplify(norm(jacobian(M(nLinks, nLinks), q)));
if (check == 0)
    disp("M_nn is constant... OK!");
else
    disp("M_nn is constant... NOT OK!");
end


% Check that C(q, dq) = S(q, dq) dq
check = simplify(C - S*q_dot, 'steps', 100);
if (check == zeros(nLinks, 1))
    disp("Coriolis factorization... OK!");
else
    disp("Coriolis factorization... NOT OK!");
end

% Check that M_dot - 2S is skew symmetric
check = simplify(x'*(time_diff(M, q, q_dot) - 2*S)*x, 'steps', 100);
if (check == 0)
    disp("Skew symmetry... OK!");
else
    disp("Skew symmetry... NOT OK!");
end

% Check that M_ij is a function of at most (q_k+1, ..., q_n) for k=min(i,j)
check = 0;
for i=1:nLinks
    for j=1:nLinks
        Mij = M(i,j);
        k = min(i, j);
        for l=1:k
            ql = q(l);
            check = diff(Mij, ql);
            if (check ~= 0)
               break 
            end
        end
    end
end

if (check == 0)
   disp("Upper triangular property... OK!");
else
    disp("Upper triangular property... NOT OK!");
end

% Check that kinetic energy adds up
check = simplify(0.5*q_dot'*M*q_dot - T, 'steps', 100);
if (check == 0)
    disp("Kinetic energy... OK!");
else
    disp("Kinetic energy... NOT OK!");
end

disp("Done.")
% disp("The rigid manipulator Coriolis factorization matrix is:");
% disp(S);
toc

%% END BUILD OF THE MODEL
disp("Model derivation complete.");

%% Writing model terms to functions for later use

tic
disp("Writing Matlab functions...");

matlabFunction(M, 'Vars', {q}, 'File', ['modelFunctions/','M'], 'Optimize', true);
matlabFunction(C, 'Vars', {q, q_dot}, 'File',['modelFunctions/','c'], 'Optimize', true);
matlabFunction(G, 'Vars', {q}, 'File', ['modelFunctions/','g'], 'Optimize', true);

disp("Done.");
toc

%% Cleanup
clear all;
close all;