% This script calculates the robot's Slotine & Li regressor and parameter
% vector. 
% -------------

% =========================================================================
% REGRESSOR
% =========================================================================

% -------
disp('Equations of motion derived');
% Display start regression and start timer
disp('Start regression');

tic;

% The regressor Yreg and the parameter vector pi_param are initially empty

Wreg = [];
pi_param = [];

% Express inertia tensor i in a 3x3x6 tensor E and a vector J. These are
% the components of the tensor E

E_{1} = [1 0 0; 0 0 0; 0 0 0];
E_{2} = [0 1 0; 1 0 0; 0 0 0];
E_{3} = [0 0 1; 0 0 0; 1 0 0];
E_{4} = [0 0 0; 0 1 0; 0 0 0];
E_{5} = [0 0 0; 0 0 1; 0 1 0];
E_{6} = [0 0 0; 0 0 0; 0 0 1];
E = [E_{1} E_{2} E_{3} E_{4} E_{5} E_{6}];

% Skew symmetric matrix S(p_iG_i) is reshaped as an inner product of the
% third order tensor Q with vector p_iG_i. These are the components of Q

Q_{1} = [0 0 0; 0 0 -1; 0 1 0];
Q_{2} = [0 0 1; 0 0 0; -1 0 0];
Q_{3} = [0 -1 0; 1 0 0; 0 0 0];
Q = [Q_{1} Q_{2} Q_ {3}];

% start loop over each link

for ii = 1: length(a)
    dJdq = sym(zeros(length(a),length(a),length(a)));
    for jj = 1: length(a)
        dJdq(:,:,jj) = diff((J_v_{ii}.'*J_v_{ii}),q(jj));
    end
    dJdqT = permute(dJdq ,[1 3 2]);
    dJdq_tot = dJdq + dJdqT;
    
    for jj = 1: length(a)
        dJdq_times_qd(:,jj) = dJdq_tot(:,:,jj)*q_dot;
    end
    
    X_d_0r_{ii} = (J_v_{ii}.'*J_v_{ii})*qr_dot_dot + 0.5*dJdq_times_qd*qr_dot;

% calculate X1dot

    for jj = 1:3
        X_1_{ii}(:,:,jj) = J_omega_{ii}.'*T_0_{ii}(1:3, 1:3)*Q_{jj}*T_0_{ii}(1:3 ,1:3).'*J_v_{ii}...
        -J_v_{ii}.'*T_0_{ii}(1:3, 1:3)*Q_{jj}*T_0_{ii}(1:3 ,1:3).'*J_omega_{ii};
    end
    
    for jj = 1:3
        X_1_times_qrdd(:,jj) = X_1_{ii}(:,:,jj)*qr_dot_dot;
    end

    dX1dq = sym(zeros(length(a),length(a),length(a),3));

    for jj = 1: length(a)
        for kk = 1:3
            dX1dq(:,:,jj,kk) = diff((X_1_{ii}(:,:,kk)),q(jj));
        end
    end

    dX1dqT = permute(dX1dq, [1 3 2 4]);
    dX1dq_tot = dX1dq + dX1dqT;
    dX1dq_times_qd = sym(zeros(length(a),length(a),3));

    for jj = 1: length(a)
        for kk = 1:3
            dX1dq_times_qd(:,jj,kk) = dX1dq_tot(:,:,jj,kk)*q_dot;
        end
    end
    for jj = 1:3
        X_d_1r_{ii}(:,jj) = X_1_times_qrdd(:,jj) + 0.5*dX1dq_times_qd(:,:,jj)*qr_dot;
    end

% calculate X2dot
    for jj = 1:6
        X_2_{ii}(:,:,jj) = J_omega_{ii}.'*T_0_{ii }(1:3 ,1:3)*E_{jj}*T_0_{ii}(1:3 ,1:3).'*J_omega_{ii};
    end

    for jj = 1:6
        X_2_times_qrdd(:,jj) = X_2_{ii}(:,:,jj)*qr_dot_dot;
    end
    dX2dq = sym(zeros(length(a),length(a),length(a),6));

    for jj = 1: length(a)
        for kk = 1:6
            dX2dq(:,:,jj,kk) = diff((X_2_{ii}(:,:,kk)),q(jj));
        end
    end

    dX2dqT = permute(dX2dq,[1 3 2 4]);
    dX2dq_tot = dX2dq + dX2dqT;
    dX2dq_times_qd = sym(zeros(length(a),length(a),6));

    for jj = 1: length(a)
        for kk = 1:6
            dX2dq_times_qd(:,jj,kk) = dX2dq_tot(:,:,jj,kk)*q_dot;
        end
    end
    
    for jj = 1:6
        X_d_2r_{ii}(:,jj) = X_2_times_qrdd(:,jj) + 0.5*dX2dq_times_qd(:,:,jj)*qr_dot;
    end

% pi

pi_0_{ii} = m(ii);
pi_1_{ii} = [m(ii)*o_c_{ii}(1) m(ii)*o_c_{ii}(2) m(ii)*o_c_{ii }(3)].';
iI_{ii} = I_{ii} + m(ii)*[0 -o_c_{ii}(3) o_c_{ii}(2); o_c_{ii}(3) 0 -o_c_{ii }(1); ...
                          -o_c_{ii}(2) o_c_{ii}(1) 0].'*[0 -o_c_{ii}(3) o_c_{ii}(2); o_c_{ii}(3) 0 -o_c_{ii}(1); ...
                          -o_c_{ii}(2) o_c_{ii}(1) 0];
Ji = [iI_{ii}(1 ,1) iI_{ii}(1,2) iI_{ii}(1,3) iI_{ii}(2 ,2) iI_{ii}(2,3) iI_{ii}(3 ,3)];
pi_2_{ii} = Ji.';

% Rewrite the second part of the lagrange equation in W and pi

% (pi already determined above)
% ------------------------------------------------------------
% skew symmetric matrices S_i

s1 = J_omega_{ii}*q_dot; 
S1 = [0 -s1(3) s1(2); s1(3) 0 -s1(1); -s1(2) s1(1) 0];
s2 = J_v_{ii}*q_dot; 
S2 = [0 -s2(3) s2(2); s2(3) 0 -s2(1); -s2(2) s2(1) 0];

% calculate Y
Y_0_{ii} = [];
    for jj = 1: length(a)
        line_{jj} = 0.5*qr_dot.'*( diff((J_v_{ii}.'*J_v_{ii}),q(jj)))*q_dot;
        Y_0_{ii} = [Y_0_{ii}; line_{jj}];
    end
clear line_

Y_1_{ii} = [];

    for jj = 1: length(a)
        line_{jj} = 0.5*(diff((T_0_{ii }(1:3 ,1:3).'*S1.'*J_v_{ii}*qr_dot -T_0_{ii}(1:3 ,1:3).'...
                         *S2.'*J_omega_{ii}*qr_dot),q(jj))).';
        Y_1_{ii} = [Y_1_{ii}; line_{jj}];
    end
clear line_

Y_2_{ii} = [];
    for jj = 1: length(a)
        for kk = 1:6
            line_{jj}(:,kk) = 0.5*qr_dot.'*(diff((J_omega_{ii}.'*T_0_{ii }(1:3 ,1:3)* E_{kk}...
                                            *T_0_{ii}(1:3 ,1:3).'*J_omega_{ii}),q(jj)))*q_dot;
        end
        Y_2_{ii} = [Y_2_{ii}; line_{jj}];
    end
clear line_

% Rewrite the third part of the lagrange equation in Z
% calculate Z
Z_0_{ii} = J_v_{ii}.'*g_vec; % note that the minus sign is dropped ,

% since here the vector g is opposite to

% the true gravitational vector
Z_1_{ii} = [];

for jj = 1: length(a)
    line_{jj} = diff((T_0_{ii}(1:3, 1:3).'*g_vec),q(jj));
    Z_1_{ii} = [Z_1_{ii}; line_{jj}.'];
end

clear line_

% Determine the regressor blocks

% ------------------------------
W_0_{ii} = X_d_0r_{ii} - Y_0_{ii} + Z_0_{ii};
W_1_{ii} = X_d_1r_{ii} - Y_1_{ii} + Z_1_{ii};
W_2_{ii} = X_d_2r_{ii} - Y_2_{ii};

% Determine the regressor and parameter vector for link i

% -------------------------------------------------------
W_{ii} = [W_0_{ii} W_1_{ii} W_2_{ii}];
pi_v_{ii} = [pi_0_{ii}; pi_1_{ii}; pi_2_{ii}];

% Fill total regressor and parameter vector

% -----------------------------------------
Wreg = [Wreg W_{ii}];
pi_param = [pi_param; pi_v_{ii}];

% end loop
end

% Remove columns that result in zero entries
disp('removing columns from regressor that will result in zero and clean up parameter vector');
ind = find(pi_param );
pi_param = pi_param(ind);
Wreg = Wreg(:,ind);

% Display complete and stop timer

disp('Regression complete , type "Wreg" and "pi_param" to displays regressor and parameter vector');
toc;

save('Din_and_regressor.mat')
