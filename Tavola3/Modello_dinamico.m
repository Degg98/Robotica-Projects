%% MODELLO DINAMICO QUADCOPTER
syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 u1 u2 u3 u4 g dwx dwy dwz m Ix Iy Iz real
% Definisco il vettore di stato, quello degli ingressi e quello delle uscite
x = [x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12]';
u = [u1 u2 u3 u4]';
y = [x1 x2 x3 x9]';

% Descrivo il sistema in forma affine nel controllo
% dx = f(x) + g1(x)*u1 + g2(x)*u2 + g3(x)*u3 + g4(x)*u4
f = [x4 x5 x6 0 0 -g x10*cos(x9)-x11*sin(x9) x10*sin(x9)+x11*cos(x9) x12 0 0 0]';
g1 = [0 0 0 -sin(x8)/m (sin(x7)*cos(x8))/m (cos(x7)*cos(x8))/m 0 0 0 0 0 0]';
g2 = [0 0 0 0 0 0 0 0 0 1/Ix 0 0]';
g3 = [0 0 0 0 0 0 0 0 0 0 1/Iy 0]';
g4 = [0 0 0 0 0 0 0 0 0 0 0 1/Iz]';

% Equilibrio (considerando i disturbi del vento trascurabili)
% x_eq = [x1' x2' x3' 0 0 0 k*pi k*pi 0 0 0 0]';
% u_eq = [+-m*g 0 0 0]';
% Se x7 e x8 hanno valore diverso all'equilibrio il segno di u1 diventa
% negativo
% Per semplicità consideriamo il seguente equilibrio
x_eq = [0 0 0 0 0 0 0 0 0 0 0 0]';
u_eq = [m*g 0 0 0]';


%% Modello dinamico con stato aumentato con u1
syms du1 real

x_a = [x' u1]';
u_a = [du1 u2 u3 u4]';
% Dove:
% u1_dot = du1

% Il sistema complessivo diventa
f_a = [x4 x5 x6 -sin(x8)*u1/m +sin(x7)*cos(x8)*u1/m -g+cos(x7)*cos(x8)*u1/m x10*cos(x9)-x11*sin(x9) x10*sin(x9)+x11*cos(x9) x12 0 0 0 0]';
g1_a = [0 0 0 0 0 0 0 0 0 0 0 0 1]';
g2_a = [0 0 0 0 0 0 0 0 0 1/Ix 0 0 0]';
g3_a = [0 0 0 0 0 0 0 0 0 0 1/Iy 0 0]';
g4_a = [0 0 0 0 0 0 0 0 0 0 0 1/Iz 0]';

x_a_eq = [x_eq; m*g];

%% Modello dinamico con stato aumentato con u1 e la sua derivata
syms s1 real

x_aa = [x_a; du1];
u_aa = [s1 u2 u3 u4]';
% Dove:
% u1_dotdot = s1

% Il sistema complessivo diventa
f_aa = [x4 x5 x6 -sin(x8)*u1/m sin(x7)*cos(x8)*u1/m -g+cos(x7)*cos(x8)*u1/m x10*cos(x9)-x11*sin(x9) x10*sin(x9)+x11*cos(x9) ...
    x12 0 0 0 du1 0]';
g1_aa = [0 0 0 0 0 0 0 0 0 0 0 0 0 1]';
g2_aa = [0 0 0 0 0 0 0 0 0 1/Ix 0 0 0 0]';
g3_aa = [0 0 0 0 0 0 0 0 0 0 1/Iy 0 0 0]';
g4_aa = [0 0 0 0 0 0 0 0 0 0 0 1/Iz 0 0]';

x_aa_eq = [x_a_eq; 0];
