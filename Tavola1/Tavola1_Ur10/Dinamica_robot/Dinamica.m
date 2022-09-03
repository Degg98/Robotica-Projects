%symbolic dynamic
clc
clear
close all
syms q1 q2 q3 q4 q5 q6 l1 l2 l3 l4 l5 l6 dq1 dq2 dq3 dq4 dq5 dq6 m1 m2 m3 m4 m5 m6 r real
g = 9.81;

% Compute links inertia matrix espressi in terna baricentrica
% e principale d'inerzia
I1 = Inertia(m1, l1, r);
I2 = Inertia(m2, l2, r);
I3 = Inertia(m3, l3, r);
I4 = Inertia(m4, l4, r);
I5 = Inertia(m5, l5, r);
I6 = Inertia(m6, l6, r);

% Joints position
q = [q1; q2; q3; q4; q5; q6];
% Joints speed
dq = [dq1; dq2; dq3; dq4; dq5; dq6];

% Defining a DH table for the initial configuration of the robot
DH_table = [0, pi/2, l1, q1;
            l2, 0, 0, q2;
            l3, 0, 0, q3;
            0, pi/2, l4, q4;
            0, -pi/2, l5, q5;
            0, 0, l6, q6];

% from D_H to matrix
% Ai = homogeneous matrix from link i-1 to i
A1 = simplify(Trans(0,pi/2,l1,q1));
A2 = simplify(Trans(l2,0,0,q2));
A3 = simplify(Trans(l3,0,0,q3));
A4 = simplify(Trans(0,pi/2,l4,q4));
A5 = simplify(Trans(0,-pi/2,l5,q5));
A6 = simplify(Trans(0,0,l6,q6));

% Creating all the possible transformation matrices
Tr2= A1*A2;
Tr3= A1*A2*A3;
Tr4= A1*A2*A3*A4;
Tr5= A1*A2*A3*A4*A5;
Tr6= A1*A2*A3*A4*A5*A6;

% Extraction of the rotational matrices
R01 = A1(1:3, 1:3);
R02 = Tr2(1:3, 1:3);
R03 = Tr3(1:3, 1:3);
R04 = Tr4(1:3, 1:3);
R05 = Tr5(1:3, 1:3);
R06 = Tr6(1:3, 1:3);

n = 6;

% Base link transformation
Ts0 = eye(4);
% End Effector transformation
Tee = eye(4);


for k = 1: n

    disp(k)
    % Calcola J_k = Jacobiano geometrico dal frame 0 al frame k
    eval(['J_' num2str(k) '=Geometric_Jacobian(Ts0,Tee,DH_table(1:k,:), k);'])
    % Esegue il calcolo fatto sopra e di fatto associa J = J_k
    % solo che adesso il J ha dimensioni nxk
    J = Geometric_Jacobian(Ts0,Tee,DH_table(1:k,:), k);
    if k<n
        % Riempie J di 0 per avere un matrice quadrata
        J = simplify([J,zeros(6,n-k)]);
        % Associa J_k = J con j di dimensioni nxn
        eval(['J_' num2str(k) '= J;'])
    end

end

% Jacobiano di posizione
Jv_1 = J_1(1:3,:);
Jv_2 = J_2(1:3,:);
Jv_3 = J_3(1:3,:);
Jv_4 = J_4(1:3,:);
Jv_5 = J_5(1:3,:);
Jv_6 = J_6(1:3,:);
% Jacobiano di orientazione
Jo_1 =  J_1(4:6,:);
Jo_2 =  J_2(4:6,:);
Jo_3 =  J_3(4:6,:);
Jo_4 =  J_4(4:6,:);
Jo_5 =  J_5(4:6,:);
Jo_6 =  J_6(4:6,:);

% Jacobiano geometrico della terna EE
JEE = sym(zeros(size(J_6)));
for i = 1:6
    disp(i)
    JEE(i,:) = simplify(J_6(i,:));
end

% Calcolo della matrice d'inerzia
B1 = (m1*(Jv_1')*Jv_1 + Jo_1'*(R01*I1*R01')*Jo_1);
B2 = (m2*(Jv_2')*Jv_2 + Jo_2'*(R02*I2*R02')*Jo_2);
B3 = (m3*(Jv_3')*Jv_3 + Jo_3'*(R03*I3*R03')*Jo_3);
B4 = (m4*(Jv_4')*Jv_4 + Jo_4'*(R04*I4*R04')*Jo_4);
B5 = (m5*(Jv_5')*Jv_5 + Jo_5'*(R05*I5*R05')*Jo_5);
B6 = (m6*(Jv_6')*Jv_6 + Jo_6'*(R06*I6*R06')*Jo_6);

B_ = B1+B2+B3+B4+B5+B6;
B = sym(zeros(size(B_)));
for i = 1:6
    disp(i)
    B(i,:) = simplify(B_(i,:));
end

% Coriolis matrix
Cmat = sym(zeros(size(B)));
for i = 1 : n
    for j = 1 : n
        for k = 1 : n
            % Simboli di Christoffel
            Cmat(i,j) = Cmat(i,j) + (  (1/2) * dq(k) * (jacobian(B(i,j), q(k)) + jacobian(B(i,k), q(j)) - jacobian(B(j,k), q(i))));
        end
    end
end

C = sym(zeros(size(Cmat)));
for i = 1:n
    for j = 1:n
        disp(i)
        
        disp(j)
        C(i,j) = simplify(Cmat(i,j));
    end
end

test_dinamica

%% Gravity matrix
g = [0; 0; g];

G1 = simplify(m1*g'*Jv_1);
G2 = simplify(m2*g'*Jv_2);
G3 = simplify(m3*g'*Jv_3);
G4 = simplify(m4*g'*Jv_4);
G5 = simplify(m5*g'*Jv_5);
G6 = simplify(m6*g'*Jv_6);

G_ = -(G1+G2+G3+G4+G5+G6);

G = simplify(G_)';
