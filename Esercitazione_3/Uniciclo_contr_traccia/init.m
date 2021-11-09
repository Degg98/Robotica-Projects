% controllo di posizione unicilco
close all;
clear;
clc;

% posizione iniziale 
%q0 = [-10; 20; pi/4];
%q0 = [20; -10; pi/2];
%q0 = [-10; -10; -pi];
q0 = [1; 5; pi/2];

% posizione finale
qf = [0; 0; 0];

% guadagni controllo
kth = 20;

%velocità prefissata
v0 = 10;