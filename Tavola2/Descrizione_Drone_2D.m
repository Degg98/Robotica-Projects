%% DEFINIZIONE DEL SISTEMA NON LINEARE
syms x1 x2 x3 x4 x5 x6 g m Ixx u1 u2 real
% Definisco il vettore di stato
x = [x1 x2 x3 x4 x5 x6]';
u = [u1 u2]';

% Descrivo il sistema in forma affine di controllo
% dx = f(x) + g1(x)*u1 + g2(x)*u2
f = [x4 x5 x6 0 -g 0]';
g1 = [0 0 0 -sin(x3)/m cos(x3)/m 0]';
g2 = [0 0 0 0 0 1/Ixx]';

% Equilibrio (un eq. è con x3 = 0 e u1 = m*g, l'altro è con 
% x3 = pi e u1 = -m*g)
x_eq = [0 0 0 0 0 0]';  % oppure x_eq = [0 0 pi 0 0 0]'
u_eq = [m*g 0]';    % oppure u_eq = [-m*g 0]'

% Calcolo il sistema linearizzato
dx = f + g1*u1 + g2*u2;
% A
A_ = subs(jacobian(dx, x), u, u_eq);
A = subs(A_, x, x_eq);
% B
B = subs(jacobian(dx,u), x, x_eq);
