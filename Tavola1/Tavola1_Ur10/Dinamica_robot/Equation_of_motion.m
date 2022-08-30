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
thetar_dot=[ q1des_dot; q2des_dot; q3des_dot; q4des_dot; q5des_dot; q6des_dot];
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

save('EquationOfMotion.mat')