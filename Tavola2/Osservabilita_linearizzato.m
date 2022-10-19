%% STUDIO DELL'OSSERVABILITA' DEL SISTEMA LINEARIZZATO
close all
clear
clc

Descrizione_Drone_2D
%%% ---------------------------------------------------------------------
% Verifico l'osservabilità del sistema linearizzato
% Saranno analizzati quattro casi:
% 1) Sensori di posizione
% 2) Sensori di velocità lineare
% 3) Entrambe le tipologie di sensori
% 4) Sensori di velocità sia lineare che angolare
%%% ---------------------------------------------------------------------

g = 9.81;

An = subs(A);

%% 1) Sensori di posizione
fprintf('ANALISI CON SENSORI DI POSIZIONE\n')
h1 = x1;
h2 = x2;
dh1 = jacobian(h1, x);
dh2 = jacobian(h2, x);
C = simplify([dh1; dh2]);
% Calcolo la matrice di osservabilità
O = obsv(An, C);
Or = rank(O);
fprintf('Il rango di O è %d\n', Or)
fprintf('Il sistema è completamente osservabile\n')

txt = '\nPremere invio per continuare\n';
invio = input(txt);
fprintf('----------------------------------------------------------------------------\n')

%% 2) Sensori di velocità lineare
fprintf('ANALISI CON SENSORI DI VELOCITA LINEARE\n')
h1 = x4;
h2 = x5;
dh1 = jacobian(h1, x);
dh2 = jacobian(h2, x);
C = simplify([dh1; dh2]);
% Calcolo la matrice di osservabilità
O = obsv(An, C);
Or = rank(O);
fprintf('Il rango di O è %d\n', Or)
fprintf('Il sistema è non completamente osservabile\n')

txt = '\nPremere invio per continuare\n';
invio = input(txt);
fprintf('----------------------------------------------------------------------------\n')

%% 3) Sensori di posizione e di velocità lineare
fprintf('ANALISI CON SENSORI DI POSIZIONE E DI VELOCITA LINEARE\n')
h1 = x1;
h2 = x2;
h3 = x4;
h4 = x5;
dh1 = jacobian(h1, x);
dh2 = jacobian(h2, x);
dh3 = jacobian(h3, x);
dh4 = jacobian(h4, x);
C = simplify([dh1; dh2; dh3; dh4]);
% Calcolo la matrice di osservabilità
O = obsv(An, C);
Or = rank(O);
fprintf('Il rango di O è %d\n', Or)
fprintf('Il sistema è completamente osservabile\n')

txt = '\nPremere invio per continuare\n';
invio = input(txt);
fprintf('----------------------------------------------------------------------------\n')

%% 4) Sensori di velocità lineare e angolare
fprintf('ANALISI CON SENSORI DI VELOCITA LINEARE E ANGOLARE\n')
h1 = x4;
h2 = x5;
h3 = x6;
dh1 = jacobian(h1, x);
dh2 = jacobian(h2, x);
dh3 = jacobian(h3, x);
C = simplify([dh1; dh2; dh3]);
% Calcolo la matrice di osservabilità
O = obsv(An, C);
Or = rank(O);
fprintf('Il rango di O è %d\n', Or)
fprintf('Il sistema è non completamente osservabile\n')










