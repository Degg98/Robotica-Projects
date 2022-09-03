%CT task 1
Kp = 28*diag([14 14 17 10 22 22]);
Kv = 32*diag([14 14 17 10 22 22]);
% Kp = diag([50 50 100 200 250 250]);
% Kv = diag([10 10 10 10 10 10]);

Ad = [zeros(6), eye(6);-Kp, -Kv];
Q = eye(12);
P = lyap(Ad, Q);

pi_param_ = [m1 zeros(1,6) m2 zeros(1,6) m3 zeros(1,6) m4 zeros(1,6) m5 zeros(1,6) m6 zeros(1,6)]';

%convergenza a zero dell'errore