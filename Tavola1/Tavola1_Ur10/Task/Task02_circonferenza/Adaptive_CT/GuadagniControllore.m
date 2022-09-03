%CT task 1
% Kp = 0.1*diag([1 1 5 6 8 8]);
% Kv = 0.1*diag([3 3 8 8 9 10]);
Kp = 10*diag([10 11 12 15 15 18]);
Kv = 20*diag([10 13 13 16 18 20]);

pi_param = [m1 m2 m3 m4 m5 m6 zeros(1,36)]';

%convergenza a zero dell'errore