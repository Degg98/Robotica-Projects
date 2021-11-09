% controllo di posizione unicilco
close all;
clear;
clc;

% posizione iniziale 
%q0 = [-10; 20; pi/4];
%q0 = [20; -10; pi/2];
q0 = [-10; -10; -pi];

% posizione finale
qf = [0; 0; 0];

% guadagni controllo
k1 = 1; k2 = 1; l2 = 0.25;
Klya = [k1, k2, l2];