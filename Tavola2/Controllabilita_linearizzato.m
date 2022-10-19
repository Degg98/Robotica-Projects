%% CONTROLLABILITA DEL SISTEMA LINEARIZZATO
clear
close all
clc

Descrizione_Drone_2D
g = 9.81; % [m/s^2]
m = 2; % [kg]
Ixx = 1.25; % [Nm]

An = subs(A);

%% CONTROLLABILITA CON f IN INGRESSO
fprintf('CONTROLLABILITA CON f IN INGRESSO\n')
u = u1;
B = subs(jacobian(dx,u), x, x_eq);
Bn = subs(B);
% Matrice di raggiungibilità
R = ctrb(An, Bn);
Rr = rank(R);
fprintf('Il rango di R è %d\n', Rr)
fprintf('Il sistema non è controllabile\n')

fprintf('\n----------------------------------------------------------------------------\n')

%% CONTROLLABILITA CON tau COME INGRESSO
fprintf('CONTROLLABILITA CON tau COME INGRESSO\n')
u = u2;
B = subs(jacobian(dx,u), x, x_eq);
Bn = subs(B);
% Matrice di raggiungibilità
R = ctrb(An, Bn);
Rr = rank(R);
fprintf('Il rango di R è %d\n', Rr)
fprintf('Il sistema non è controllabile\n')

fprintf('\n----------------------------------------------------------------------------\n')

%% CONTROLLABILITA CON f E tau COME INGRESSI
fprintf('CONTROLLABILITA CON f E tau COME INGRESSI\n')
u = [u1 u2]';
B = subs(jacobian(dx,u), x, x_eq);
Bn = subs(B);
% Matrice di raggiungibilità
R = ctrb(An, Bn);
Rr = rank(R);
fprintf('Il rango di R è %d\n', Rr)
fprintf('Il sistema è controllabile\n')