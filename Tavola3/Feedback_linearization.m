%% FEEDBACK LINEARIZATION

clear
close all
clc

Modello_dinamico

% Eseguo il test per verificare se è possibile eseguire la linearizzazione
% esatta in retroazione completa sul sistema in esame (Teorema 12)

Gamma0 = simplify([g1 g2 g3 g4]);
Delta = simplify(f);
% Il test si basa sulla filtrazione definita da Gamma0 e Delta
% Filtrazione
% test(Delta, Gamma0, x, x_eq)

% Calcole derivate prime delle uscite (non compaiono ingressi)
Lfh1 = x4;
Lfh2 = x5;
Lfh3 = x6;
Lfh4 = x12;
Lfh = [Lfh1 Lfh2 Lfh3 Lfh4];
% Calcolo le derivate seconde delle uscite
for i=1:size(Lfh,2)
    Lfh_2(i) = Ddir_scalar(f, Lfh(i), x);
    % Trascuro i disturbi del vento
    %Lfh_2 = subs(Lfh_2, [dwx dwy dwz], [0 0 0]);
end
% Costruisco la matrice E considerando fino alle derivate seconde
for i=1:size(Lfh,2)
    for j=1:size(Gamma0,2)
        E(i,j) = Ddir_scalar(Gamma0(:,j), Lfh(i), x);
    end
end
% Calcolo il rango di E nell'equilibrio 
rank(subs(E, x, x_eq))  % 2 => E non è invertibile

%% Aumento il vettore di stato con l'ingresso u1

% x_a = [x' u1]';
% u_a = [du1 u2 u3 u4]';

% Calcolo le derivate seconde delle uscite
for i=1:size(Lfh,2)
    Lfh_2(i) = Ddir_scalar(f_a, Lfh(i), x_a);
    % Trascuro i disturbi del vento
    %Lfh_2 = subs(Lfh_2, [dwx dwy dwz], [0 0 0]);
end
% % Calcolo le derivate terze delle uscite
% for i=1:size(Lfh_2,2)
%     Lfh_3(i) = Ddir_scalar(f_a, Lfh_2(i), x_a);
%     % Trascuro i disturbi del vento
%     %Lfh_3 = subs(Lfh_3, [dwx dwy dwz], [0 0 0]);
% end

% Costruisco la matrice E considerando fino alle derivate terze
% Rispetto allo stato aumentato x_a
Gamma0_a = [g1_a g2_a g3_a g4_a];
for i=1:size(Lfh_2,2)-1
    for j=1:size(Gamma0_a,2)
        E(i,j) = Ddir_scalar(Gamma0_a(:,j), Lfh_2(i), x_a);
    end
end
for i=1:4
    E(4,i) = Ddir_scalar(Gamma0_a(:,i), Lfh(4), x_a);
end
% Calcolo il rango di E nell'equilibrio 
rank(subs(E, x_a, x_a_eq))  % 2 => E non è invertibile

%% Aumento il vettore di stato con la derivata di u1

% x_aa = [x_a; du1];
% u_aa = [s1 u2 u3 u4]';

% Calcolo le derivate seconde delle uscite
for i=1:size(Lfh,2)
    Lfh_2(i) = Ddir_scalar(f_aa, Lfh(i), x_aa);
    % Trascuro i disturbi del vento
    %Lfh_2 = subs(Lfh_2, [dwx dwy dwz], [0 0 0]);
end
% Calcolo le derivate terze delle uscite
for i=1:size(Lfh_2,2)
    Lfh_3(i) = Ddir_scalar(f_aa, Lfh_2(i), x_aa);
    % Trascuro i disturbi del vento
    %Lfh_3 = subs(Lfh_3, [dwx dwy dwz], [0 0 0]);
end
% Calcolo le derivate quarte delle uscite
for i=1:size(Lfh_3,2)
    Lfh_4(i) = Ddir_scalar(f_aa, Lfh_3(i), x_aa);
    % Trascuro i disturbi del vento
    %Lfh_4 = subs(Lfh_4, [dwx dwy dwz], [0 0 0]);
end

% Costruisco la matrice E considerando fino alle derivate terze
% Rispetto allo stato aumentato x_a
Gamma0_aa = [g1_aa g2_aa g3_aa g4_aa];
for i=1:size(Lfh_3,2)-1
    for j=1:size(Gamma0_aa,2)
        E(i,j) = Ddir_scalar(Gamma0_aa(:,j), Lfh_3(i), x_aa);
    end
end
for i=1:4
    E(4,i) = Ddir_scalar(Gamma0_aa(:,i), Lfh(4), x_aa);
end
% Calcolo il rango di E nell'equilibrio 
rank(subs(E, x_aa, x_aa_eq))  % 4 => E è invertibile
matlabFunction(E, 'File', 'E_fun');

Gamma = [Lfh_4(1) Lfh_4(2) Lfh_4(3) Lfh(4)]';
matlabFunction(Gamma, 'File', 'Gamma_fun');

%% Retroazione linearizzante
syms u v1 v2 v3 v4 real

v = [v1 v2 v3 v4]';
% Calcolo dell'ingresso linearizzante
u_c = -inv(E)*Gamma + inv(E)*v;

F = simplify(f_aa + g1_aa*u_c(1) + g2_aa*u_c(2) + g3_aa*u_c(3) + g4_aa*u_c(4));

%% Cambio di variabili
z1 = x1; z2 = Lfh1; z3 = Lfh_2(1); z4 = Lfh_3(1);  
z5 = x2; z6 = Lfh2; z7 = Lfh_2(2); z8 = Lfh_3(2);
z9 = x3; z10 = Lfh3; z11 = Lfh_2(3); z12 = Lfh_3(3);
z13 = x9; z14 = Lfh4;
z = [z1 z2 z3 z4 z5 z6 z7 z8 z9 z10 z11 z12 z13 z14]';
% A
A1 = [0 1 0 0;0 0 1 0;0 0 0 1;0 0 0 0];
A2 = [0 1;0 0];
A = blkdiag(A1, A1, A1, A2);
% B
B1 = [0; 0; 0; 1];
B2 = [0; 1];
B = blkdiag(B1, B1, B1, B2);
% C
C1 = [1 0 0 0];
C2 = [1 0];
C = blkdiag(C1, C1, C1, C2);
% Sistema linearizzato
zdot = simplify(A*z + B*v);
yz = simplify(C*z);

%% CONTROLLO DEL SISTEMA LINEARIZZATO
% Controllo stabilizzante
p = [-4 -3 -2 -1];
K1 = place(A1, B1, p);
K2 = place(A2, B2, p(3:4));
K = blkdiag(K1, K1, K1, K2);


%% Parametri per simulazione
% Parametri
m = 2; % kg
g = 9.81; % m/s^2
Ix = 1.25; % kg*m^2
Iy = 1.25; % kg*m^2
Iz = 2.5; % kg*m^2
dwx = 0; %m/s^2
dwy = 0; %m/s^2
dwz = 0; %m/s^2







