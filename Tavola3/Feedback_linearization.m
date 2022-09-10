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
    Lfh_2(i) = Ddir_scalar(f+g1*u1+g2*u2+g3*u3+g4*u4, Lfh(i), x);
    % Trascuro i disturbi del vento
    Lfh_2 = subs(Lfh_2, [dwx dwy dwz], [0 0 0]);
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

% Calcolo le derivate terze delle uscite
for i=1:size(Lfh_2,2)
    Lfh_3(i) = Ddir_scalar(f_a+g1_a*du1+g2_a*u2+g3_a*u3+g4_a*u4, Lfh_2(i), x_a);
    % Trascuro i disturbi del vento
    Lfh_3 = subs(Lfh_3, [dwx dwy dwz], [0 0 0]);
end

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

% Calcolo le derivate terze delle uscite
for i=1:size(Lfh_3,2)
    Lfh_4(i) = Ddir_scalar(f_aa+g1_aa*s1+g2_aa*u2+g3_aa*u3+g4_aa*u4, Lfh_3(i), x_aa);
    % Trascuro i disturbi del vento
    Lfh_4 = subs(Lfh_4, [dwx dwy dwz], [0 0 0]);
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
rank(subs(E, x_aa, x_aa_eq))  % 2 => E non è invertibile

Gamma = [Lfh_4(1) Lfh_4(2) Lfh_4(3) Lfh(4)]';

%% Retroazione linearizzante
syms u v1 v2 v3 v4 real

v = [v1 v2 v3 v4]';
u = -inv(E)*Gamma + inv(E)*v;

F = f_aa + g1_aa*u(1) + g2_aa*u(2) + g3_aa*u(3) + g4_aa*u(4);
