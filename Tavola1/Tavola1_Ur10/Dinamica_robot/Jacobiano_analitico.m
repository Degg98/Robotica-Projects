%% Jacobiano analitico

syms phi psi theta real

Toa = [0, -sin(psi), cos(psi)*sin(theta); 
       0,  cos(psi), sin(psi)*sin(theta); 
       1,         0,          cos(theta)];


T_A = blkdiag([eye(3), zeros(3); zeros(3), Toa]);

J_A = pinv(T_A)*J;