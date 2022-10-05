%% Jacobiano analitico

syms phi psi theta real

Toa = [0, -sin(psi), cos(psi)*sin(theta); 
       0,  cos(psi), sin(psi)*sin(theta); 
       1,         0,          cos(theta)];


T_A = blkdiag([eye(3), zeros(3); zeros(3), Toa]);

J_A = pinv(T_A)*J;
%%
J_A_inv = inv(J_A); 

for i = 1:6
    J_A_inv_dot(:,i) = jacobian(J_A_inv(:,i)',q)*qd;
end

% J_A_inv_dot = jacobian(inv(J_A),q)*qd;