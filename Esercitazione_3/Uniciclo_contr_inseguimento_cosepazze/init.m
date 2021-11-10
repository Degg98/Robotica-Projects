% controllo di posizione unicilco
%close all;
clear;
clc;

% posizione iniziale inseguitore
%q0 = [-10; 20; pi/4];
%q0 = [20; -10; pi/2];
%q0 = [-10; -10; -pi];
q0 = [40; 5; pi/2];

% posizione iniziale fugitivo (RAS stimata tra 28.375 e 28.38)
%q0 = [-10; 20; pi/4];
%q0 = [20; -10; pi/2];
%q0 = [-10; -10; -pi];
q0_hat = [8; 2; -pi];


% guadagni controllo
l1 = 1; l2 = 1; k = 10;
kl = [l1 l2 k];

% w_hat = 1;
% v_hat = 10;

v0_hat = 10;
kv = 10;
r0 = 1;