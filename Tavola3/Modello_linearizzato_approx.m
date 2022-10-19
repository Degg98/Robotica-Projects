%% Modello del linearizzato approssimato
close all
clear
clc

Modello_dinamico
g = 9.81; % [m/s^2]
m = 2; % [kg]
Ix = 1.25; % [Nm]
Iy = 1.25; % [Nm]
Iz = 2.5; % [Nm]

dx = f + g1*u1 + g2*u2 + g3*u3 + g4*u4;
y = [x3;x2;x1;x9];

% Matrice dinamica
A_ = jacobian(dx, x);
A_ = subs(A_, x, x_eq);
A_ = subs(A_, u, u_eq);
A = double(subs(A_));
% Matrice degli ingressi
B_ = jacobian(dx, u);
B_ = subs(B_, x, x_eq);
B_ = subs(B_, u, u_eq);
B = double(subs(B_));
% Matrice delle uscite
C_ = jacobian(y, x);
C_ = subs(C_, x, x_eq);
C_ = subs(C_, u, u_eq);
C = double(subs(C_));

D = zeros(4,4);

% FdT sistema
Fs = ss(A, B, C, D);
F = tf(Fs);

%% CONTROLLORI SISO 
C1 = 1/5 * tf([10 1], [0.01 1]);
C2 = 1/784.8 * tf([1e3 3e2 30 1], [1e-6 3e-4 3e-2 1]);
C3 = -1/784.8 * tf([1e3 3e2 30 1], [1e-6 3e-4 3e-2 1]);
C4 = 1/0.4 * tf([10 1], [0.01 1]);

Ct = blkdiag(C1, C2, C3, C4);


