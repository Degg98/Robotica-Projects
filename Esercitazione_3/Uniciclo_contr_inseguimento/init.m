% controllo di posizione unicilco
close all;
clear;
clc;

% posizione iniziale inseguitore
%q0 = [-10; 20; pi/4];
%q0 = [20; -10; pi/2];
%q0 = [-10; -10; -pi];
q0 = [-10; 5; pi/2];

% posizione iniziale fugitivo
%q0 = [-10; 20; pi/4];
%q0 = [20; -10; pi/2];
%q0 = [-10; -10; -pi];
q0_hat = [40; -5; pi];


% guadagni controllo
l1 = 1; l2 = 1; k = 10;
kl = [l1 l2 k];

w_hat = 1;
v_hat = 10;