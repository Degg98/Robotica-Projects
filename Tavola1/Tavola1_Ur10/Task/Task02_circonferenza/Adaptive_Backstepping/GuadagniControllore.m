%% Adaptive BS
% Circle
Kd = diag([100 500 300 10 2 0.1]);
lambda = diag([30 70 40 300 50 500]);

% Link mass
m1 = L(1).m;
m2 = L(2).m;
m3 = L(3).m;
m4 = L(4).m;
m5 = L(5).m;
m6 = L(6).m;