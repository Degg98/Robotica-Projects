% This script calculates the robot's Slotine & Li regressor and parameter
% vector. 
% -------------

% =========================================================================
% EQUATIONS OF MOTION
% =========================================================================


% display
% -------

disp('Start deriving equations of motion');


% declare symbolic variables
% --------------------------

syms pi real;
syms q1 q2 q3 q4 q5 q6 real;
syms q1_dot q2_dot q3_dot q4_dot q5_dot q6_dot real;
syms q1des_dot q2des_dot q3des_dot q4des_dot q5des_dot q6des_dot real;
syms q1_dot_dot q2_dot_dot q3_dot_dot q4_dot_dot q5_dot_dot q6_dot_dot;


g=9.81;
a1=L(1,1).a; a2=L(1,2).a; 
d1=L(1,1).d; d4=L(1,4).d; d6=L(1,6).d;

m1=L(1,1).m;
m2=L(1,2).m;
m3=L(1,3).m;
m4=L(1,4).m;
m5=L(1,5).m;
m6=L(1,6).m;

Ixx_1=L(1,1).I(1,1); Iyy_1=L(1,1).I(2,2); Izz_1=L(1,1).I(3,3);
Ixx_2=L(1,2).I(1,1); Iyy_2=L(1,2).I(2,2); Izz_2=L(1,2).I(3,3);
Ixx_3=L(1,3).I(1,1); Iyy_3=L(1,3).I(2,2); Izz_3=L(1,3).I(3,3);
Ixx_4=L(1,4).I(1,1); Iyy_4=L(1,4).I(2,2); Izz_4=L(1,4).I(3,3);
Ixx_5=L(1,5).I(1,1); Iyy_5=L(1,5).I(2,2); Izz_5=L(1,5).I(3,3);
Ixx_6=L(1,6).I(1,1); Iyy_6=L(1,6).I(2,2); Izz_6=L(1,6).I(3,3);


lc1x=L(1,1).r(1); lc1y=L(1,1).r(2); lc1z=L(1,1).r(3);
lc2x=L(1,2).r(1); lc2y=L(1,2).r(2); lc2z=L(1,2).r(3);
lc3x=L(1,3).r(1); lc3y=L(1,3).r(2); lc3z=L(1,3).r(3);
lc4x=L(1,4).r(1); lc4y=L(1,4).r(2); lc4z=L(1,4).r(3);
lc5x=L(1,5).r(1); lc5y=L(1,5).r(2); lc5z=L(1,5).r(3);
lc6x=L(1,6).r(1); lc6y=L(1,6).r(2); lc6z=L(1,6).r(3);



% Generate vectors between origins of the circumjacent coordinate frames
% attached to the robot joints. Generate rotation matrices between the
% circumjacent coordinate frames attached to the robot joints. Generate
% matrices of homogenous transformations between the circumjacent coordinate

% frames attached to the robot joints.
% --------------------------------------------------------------------------

a = [0; l2; l3; 0; 0; 0];
d = [l1; 0; 0; l4; l5; l6];
alfa = [0.5*pi; 0; 0; 0.5*pi; -0.5*pi; 0];
theta = [q1; q2; q3; q4; q5; q6];
theta_dot = [ q1_dot; q2_dot; q3_dot; q4_dot; q5_dot; q6_dot];
thetar_dot=[ q1des_dot; q2des_dot; q3des_dot; q4des_dot; q5des_dot; q6des_dot ];
theta_dot_dot = [ q1_dot_dot; q2_dot_dot; q3_dot_dot; q4_dot_dot; q5_dot_dot; q6_dot_dot];


q=[]; % Vector of generalized coordinates
q_dot=[]; % Vector of generalized velocities
qr_dot =[]; % Vector of generalized accelerations
qr_dot_dot =[]; % Vector of generalized accelerations

for ii= 1:length(a)
    disp(ii);
    q = [q; theta(ii)];
    q_dot = [q_dot; theta_dot(ii)];
    qr_dot = [qr_dot; thetar_dot(ii)];
    qr_dot_dot = [qr_dot_dot; theta_dot_dot(ii)];
end
Null_covector =[0 0 0];

x=[]; R=[]; A=[];

for ii=1: length(a)
    disp(ii);
    x{ii} = [a(ii)*cos(theta(ii)); a(ii)*sin(theta(ii)); d(ii)];
    R{ii} = [cos(theta(ii)), -cos(alfa(ii))*sin(theta(ii)), sin(alfa(ii))*sin(theta(ii))
             sin(theta(ii)), cos(alfa(ii))*cos(theta(ii)), -sin(alfa(ii))* cos(theta(ii))
             0, sin(alfa(ii)), cos(alfa(ii))];
         
    A{ii} = [R{ii}, x{ii}
             Null_covector , 1];
end

% Generate matrices of homogenous transformations representing positions
% x^0_i and orientations R^0_i of the link coordinate frames oixiyizi (i=1,2,3)
% relative to the coordinate frame o0x0y0z0 of the base.
% -----------------------------------------------------------------------------

T_0_{1} = A{1};

for ii = 2:length(a)
    disp(ii);
    T_0_{ii} = simplify(T_0_{ii -1} * A{ii});
end

% Create linear velocity Jacobian Jv and angular velocity Jacobian Jomega
% -----------------------------------------------------------------------
o_0_0 =[0;0;0];
z_0_0 =[0;0;1];

for kk = 1: length(a)
    disp(kk);
    J_v_{kk} = []; 
    J_omega_{kk } = [];
    J_v_{kk}= cross(z_0_0, T_0_{kk}(1:3,4) - o_0_0);
    J_omega_{kk} = z_0_0;
    
    for ii=1:kk -1
        disp(ii);
        J_v_{kk} = [J_v_{kk}, cross(T_0_{ii}(1:3,3), T_0_{kk}(1:3,4)- T_0_{ii}(1:3, 4))];
        J_omega_{kk} = [J_omega_{kk}, T_0_{ii}(1:3,3)];
    end
    
    J_v_{kk} = [J_v_{kk} zeros(3,length(a)-kk)];
    J_v_{kk} = simplify(J_v_{kk});
    J_omega_{kk} = [sym(J_omega_{kk}) sym(zeros(3,length(a)-kk))];
    J_omega_{kk} = simplify(J_omega_{kk});
end


% Coordinates of the i-th link center of mass expressed relative to the
% i-th coordinate frame
% ---------------------------------------------------------------------
o_c_{1} = [lc1x; lc1y; lc1z];
o_c_{2} = [lc2x; lc2y; lc2z];
o_c_{3} = [lc3x; lc3y; lc3z];
o_c_{4} = [lc4x; lc4y; lc4z];
o_c_{5} = [lc5x; lc5y; lc5z];
o_c_{6} = [lc6x; lc6y; lc6z];


% Origins o_0_{i} (i=1,... ,n) of the link coordinate frames expressed
% relative to the coordinate frame of the base and coordinates o_0_c_{i}
% of the i-th link center of mass expressed relative to the coordinate

% frame of the base

% ----------------------------------------------------------------------

o_0_ = [];
o_0_c_ = [];

for ii=1: length(a)
    o_0_{ii} = T_0_{ii}(1:3,4);
    o_0_c_{ii} = o_0_{ii} + T_0_{ii}(1:3 ,1:3) * o_c_{ii};
end

% Jacobians of link centers of mass
% ---------------------------------

for ii=1: length(a)
    Jv_c_{ii} = sym(0*ones(3,length(a)));
    Jomega_c_{ii} = sym(0*ones(3,length(a)));
    Jv_c_{ii}(:,1) = simplify(cross( z_0_0 , o_0_c_{ii} - o_0_0));
    Jomega_c_{ii}(:,1) = z_0_0;

    for jj=2:ii
        Jv_c_{ii}(:,jj) = simplify(cross(T_0_{jj -1}(1:3,3) , o_0_c_{ii} - o_0_{jj -1}));
        Jomega_c_{ii}(:,jj) = T_0_{jj -1}(1:3,3);
    end
end


% Link inertial parameters

% ------------------------
m=[m1; m2; m3; m4; m5; m6];

I_{1} = [Ixx_1, 0, 0; 0, Iyy_1, 0; 0, 0, Izz_1];
I_{2} = [Ixx_2, 0, 0; 0, Iyy_2, 0; 0, 0, Izz_2];
I_{3} = [Ixx_3, 0, 0; 0, Iyy_3, 0; 0, 0, Izz_3];
I_{4} = [Ixx_4, 0, 0; 0, Iyy_4, 0; 0, 0, Izz_4];
I_{5} = [Ixx_5, 0, 0; 0, Iyy_5, 0; 0, 0, Izz_5];
I_{6} = [Ixx_6, 0, 0; 0, Iyy_6, 0; 0, 0, Izz_6];

% Elements of the inertia matrix
% ------------------------------

D_q =0;
for ii = 1:length(a)
    disp(ii)
    D_q = simplify(D_q + m(ii)*transpose(Jv_c_{ii})*Jv_c_{ii} ...
                   + transpose(Jomega_c_{ii})*T_0_{ii}(1:3,1:3)*I_{ii}...
                   *transpose(T_0_{ii}(1:3,1:3))* Jomega_c_{ii});
end

% Christoffel symbols
% -------------------
for kk=1: length(a)
    disp(kk)
    for ii=1: length(a)
        disp(ii)
        for jj=1: length(a)
            disp(jj)
            c_q{ii,jj,kk} = simplify(0.5*(diff(D_q(kk,jj),q(ii)) ...
                            + diff(D_q(kk,ii),q(jj)) - diff(D_q(ii,jj),q(kk))));
        end
    end
end

% Matrix C_q appears in the term representing Coriolis and centripetal effects

% ----------------------------------------------------------------------------

C_q = 0*D_q;

for kk=1: length(a)
    for jj=1: length(a)
        [kk,jj];
        for ii=1: length(a)
            C_q(kk,jj) = C_q(kk ,jj) + c_q{ii,jj,kk}*q_dot(ii);
        end
    end
end

% gravity relative to the base frame , this is the vector OPPOSITE to the
% gravity vector!
% ----------------------------------------------------------------------

g_vec =[0; 0; g];

% Potential energy

P=0;
for ii = 1:length(a)
    P = simplify(P + m(ii)*transpose(g_vec)*o_0_c_{ii});
end

% Elements of the gravity vector

g_q = [];

for ii=1: length(a)
    g_q = [g_q; simplify(diff(P,q(ii)))];
end

% display
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
