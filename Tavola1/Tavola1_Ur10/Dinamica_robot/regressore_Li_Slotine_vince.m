clear
close all

%% load ur_10
load("Dinamica_sym.mat")

%% DH e T
syms q1 q2 q3 q4 q5 q6 real
theta = [q1 q2 q3 q4 q5 q6];


%Denavith_Hartenberg matrix
DH = [0, pi/2, l1, q1;
    l2, 0, 0, q2;
    l3, 0, 0, q3;
    0, pi/2, l4, q4;
    0, -pi/2, l5, q5;
    0, 0, l6, q6];

A10 = fill_A(DH(1,:));
A21 = fill_A(DH(2,:));
A32 = fill_A(DH(3,:));
A43 = fill_A(DH(4,:));
A54 = fill_A(DH(5,:));
A65 = fill_A(DH(6,:));

T60=A10*A21*A32*A43*A54*A65;
T60 = simplify(T60);


%% J geometrico

ze=[0;0;1];
pe=[0;0;0;1];
% estrazione matrici di rotazione
R60=T60(1:3,1:3);
R10=A10(1:3,1:3);
R21=A21(1:3,1:3);
R32=A32(1:3,1:3);
R43=A43(1:3,1:3);
R54=A54(1:3,1:3);
R65=A65(1:3,1:3);

R20 = simplify(R10*R21);
R30 = simplify(R10*R21*R32);
R40 = simplify(R10*R21*R32*R43);
R50 = simplify(R10*R21*R32*R43*R54);
R60 = simplify(R10*R21*R32*R43*R54*R65);

%estrazione assi z
z0=[0;0;1];
z1=R10*ze;
z2=R20*ze;
z3=R30*ze;
z4=R40*ze;
z5=R50*ze;
%z6=R60*ze;

% giunto prismatico Ji=[zi-1 zeros(3,1)]' (nessuno)
% giunto rotoidale Ji=[zi-1 x (p-pi-1) , zi-1]' (tutti)

P=T60(1:3,4);

P0=[0 0 0]';%P0
p1=simplify(A10);%P1
P1=p1(1:3,4);
p2=simplify(A10*A21);%P2
P2=p2(1:3,4);
p3=simplify(A10*A21*A32);%P3
P3=p3(1:3,4);
p4=simplify(A10*A21*A32*A43);%P4
P4=p4(1:3,4);
p5=simplify(A10*A21*A32*A43*A54);%P5
P5=p5(1:3,4);
p6=simplify(A10*A21*A32*A43*A54*A65);%P6
P6=P;

J1=[cross(z0,P-P0);z0];
J2=[cross(z1,P-P1);z1];
J3=[cross(z2,P-P2);z2];
J4=[cross(z3,P-P3);z3];
J5=[cross(z4,P-P4);z4];
J6=[cross(z5,P-P5);z5];
% Jacobiano finale

J=[J1 J2 J3 J4 J5 J6];
J=simplify(J);

%% Dinamica

n=6; %numero giunti
syms qd1 qd2 qd3 qd4 qd5 qd6 real
q=[q1 q2 q3 q4 q5 q6];  %posizione dei giunti
q_dot =[qd1 qd2 qd3 qd4 qd5 qd6];  %velocità dei giunti
syms m1 m2 m3 m4 m5 m6 real
m_link=[m1 m2 m3 m4 m5 m6];
% go=[-g;0;0];%vettore accelerazione di gravità,g=-9.81 si mette in base a qual'è l'asse verticale(terna base)
g = -9.81;
go = [0; 0; g];

syms real
I_link_1 = Inertia(m1, l1, r);

I_link_2 = Inertia(m2, l2, r);

I_link_3 = Inertia(m3, l3, r);

I_link_4 = Inertia(m4, l4, r);

I_link_5 = Inertia(m5, l5, r);

I_link_6 = Inertia(m6, l6, r);


%% B
Pc = [ L(1).r', L(2).r', L(3).r', L(4).r', L(5).r', L(6).r'];

Pl1=P1 + R10*Pc(:,1);
Pl2=P2 + R20*Pc(:,2);
Pl3=P3 + R30*Pc(:,3);
Pl4=P4 + R40*Pc(:,4);
Pl5=P5 + R50*Pc(:,5);
Pl6=P6 + R60*Pc(:,6);

%tutti rotoidali Jplj^j = [zj-1 x pli] Joi = [zj-1]
Jpl1=[cross(z0,Pl1-P0), zeros(3,5)];                                           
Jol1=[z0,zeros(3,5)];

Jpl2=[cross(z0,Pl2-P0),cross(z1,Pl2-P1),zeros(3,4)];
Jol2=[z0,z1,zeros(3,4)];

Jpl3=[cross(z0,Pl3-P0),cross(z1,Pl3-P1),cross(z2,Pl3-P2),zeros(3,3)];
Jol3=[z0,z1,z2,zeros(3,3)];

Jpl4=[cross(z0,Pl4-P0),cross(z1,Pl4-P1),cross(z2,Pl4-P2),cross(z3,Pl4-P3),zeros(3,2)];
Jol4=[z0,z1,z2,z3,zeros(3,2)];

Jpl5=[cross(z0,Pl5-P0),cross(z1,Pl5-P1),cross(z2,Pl5-P2),cross(z3,Pl5-P3),cross(z4,Pl5-P4),zeros(3,1)];
Jol5=[z0,z1,z2,z3,z4,zeros(3,1)];

Jpl6=[cross(z0,Pl6-P0),cross(z1,Pl6-P1),cross(z2,Pl6-P2),cross(z3,Pl6-P3),cross(z4,Pl6-P4),cross(z5,Pl6-P5)];
Jol6=[z0,z1,z2,z3,z4,z5];



B1=m_link(1)*(Jpl1'*Jpl1) + (Jol1')*R10*I_link_1*(R10')*Jol1;
B2=m_link(2)*(Jpl2'*Jpl2) + (Jol2')*R20*I_link_2*(R20')*Jol2;
B3=m_link(3)*(Jpl3'*Jpl3) + (Jol3')*R30*I_link_3*(R30')*Jol3;
B4=m_link(4)*(Jpl4'*Jpl4) + (Jol4')*R40*I_link_4*(R40')*Jol4;
B5=m_link(5)*(Jpl5'*Jpl5) + (Jol5')*R50*I_link_5*(R50')*Jol5;
B6=m_link(6)*(Jpl6'*Jpl6) + (Jol6')*R60*I_link_6*(R60')*Jol6;

% B=simplify(B1) + simplify(B2) + simplify(B3) + simplify(B4) + simplify(B5) + simplify(B6) + simplify(B7);
% B=simplify(B);
B = B1 + B2 + B3 + B4 + B5 + B6;
%% G

U = - m_link(1)*(go')*Pl1 - m_link(2)*(go')*Pl2 - m_link(3)*(go')*Pl3 - m_link(4)*(go')*Pl4 - m_link(5)*(go')*Pl5 - m_link(6)*(go')*Pl6;
G(1,:)=diff(U,q(1));
G(2,:)=diff(U,q(2));
G(3,:)=diff(U,q(3));
G(4,:)=diff(U,q(4));
G(5,:)=diff(U,q(5));
G(6,:)=diff(U,q(6));
G=simplify(G);

% syms c1 c2 c3 c4 c5 c6 c7 s1 s2 s3 s4 s5 s6 s7 real
% 
% G_sim = subs(G,[sin(t1) sin(t2) sin(t3) sin(t4) sin(t5) sin(t6) sin(t7)],[s1 s2 s3 s4 s5 s6 s7]);
% G_sim = subs(G_sim,[cos(t1) cos(t2) cos(t3) cos(t4) cos(t5) cos(t6) cos(t7)],[c1 c2 c3 c4 c5 c6 c7]);
% 
% % w1 = c3*c4*s2
% % w2 = c3*s2*s4
% % w3 = s2*s3*s5
% % w4 = c5*s2*s3
% % w5 = s3*s4*s6
% 
% syms w1 w2 w3 w4 w5 real
% G_sim = subs(G_sim,[c3*c4*s2 c3*s2*s4 s2*s3*s5 c5*s2*s3 s3*s4*s6],[w1 w2 w3 w4 w5]);
%% C (Christoffel formultion)

C=zeros(6);
C = sym(C);
for i=1:1:n
    for j=1:1:n
        C(i,j)=0;
        for k=1:1:n
            C(i,j)=C(i,j)+0.5*(diff(B(i,j),q(k),1)+diff(B(i,k),q(j),1)-diff(B(j,k),q(i),1))*q_dot(k);
        end
    end
end


%% Lagrangian Dynamics Formulation 
syms qdd1 qdd2 qdd3 qdd4 qdd5 qdd6 real
qd =[qd1 qd2 qd3 qd4 qd5 qd6]';  %velocità dei giunti
qdd = [qdd1 qdd2 qdd3 qdd4 qdd5 qdd6]';
D = B*qdd + C*qd + G;

%% Dynamics for Li-Slotine Regressor
syms qrd1 qrd2 qrd3 qrd4 qrd5 qrd6 real
qrd = [qrd1 qrd2 qrd3 qrd4 qrd5 qrd6]';
syms qrdd1 qrdd2 qrdd3 qrdd4 qrdd5 qrdd6 real
qrdd = [qrdd1 qrdd2 qrdd3 qrdd4 qrdd5 qrdd6]';

Dr = B*qrdd + C*qrd + G;


%% Linear Regressor

Y(:,1) = [diff(D(1,1),m1) diff(D(2,1),m1) diff(D(3,1),m1) diff(D(4,1),m1) diff(D(5,1),m1) diff(D(6,1),m1)]';
Y(:,2) = [diff(D(1,1),m2) diff(D(2,1),m2) diff(D(3,1),m2) diff(D(4,1),m2) diff(D(5,1),m2) diff(D(6,1),m2)]';
Y(:,3) = [diff(D(1,1),m3) diff(D(2,1),m3) diff(D(3,1),m3) diff(D(4,1),m3) diff(D(5,1),m3) diff(D(6,1),m3)]';
Y(:,4) = [diff(D(1,1),m4) diff(D(2,1),m4) diff(D(3,1),m4) diff(D(4,1),m4) diff(D(5,1),m4) diff(D(6,1),m4)]';
Y(:,5) = [diff(D(1,1),m5) diff(D(2,1),m5) diff(D(3,1),m5) diff(D(4,1),m5) diff(D(5,1),m5) diff(D(6,1),m5)]';
Y(:,6) = [diff(D(1,1),m6) diff(D(2,1),m6) diff(D(3,1),m6) diff(D(4,1),m6) diff(D(5,1),m6) diff(D(6,1),m6)]';

%% Linear Regressor Li-Slotine

Yr(:,1) = [diff(Dr(1,1),m1) diff(Dr(2,1),m1) diff(Dr(3,1),m1) diff(Dr(4,1),m1) diff(Dr(5,1),m1) diff(Dr(6,1),m1)]';
Yr(:,2) = [diff(Dr(1,1),m2) diff(Dr(2,1),m2) diff(Dr(3,1),m2) diff(Dr(4,1),m2) diff(Dr(5,1),m2) diff(Dr(6,1),m2)]';
Yr(:,3) = [diff(Dr(1,1),m3) diff(Dr(2,1),m3) diff(Dr(3,1),m3) diff(Dr(4,1),m3) diff(Dr(5,1),m3) diff(Dr(6,1),m3)]';
Yr(:,4) = [diff(Dr(1,1),m4) diff(Dr(2,1),m4) diff(Dr(3,1),m4) diff(Dr(4,1),m4) diff(Dr(5,1),m4) diff(Dr(6,1),m4)]';
Yr(:,5) = [diff(Dr(1,1),m5) diff(Dr(2,1),m5) diff(Dr(3,1),m5) diff(Dr(4,1),m5) diff(Dr(5,1),m5) diff(Dr(6,1),m5)]';
Yr(:,6) = [diff(Dr(1,1),m6) diff(Dr(2,1),m6) diff(Dr(3,1),m6) diff(Dr(4,1),m6) diff(Dr(5,1),m6) diff(Dr(6,1),m6)]';
%%

param = [m1 m2 m3 m4 m5 m6]';
    
% D - Y*param







%% functions

function A = fill_A(input)

a = input(1);
f = input(2);
d = input(3);
t = input(4);

A = [cos(t)     -sin(t)*cos(f)     sin(t)*sin(f)   a*cos(t); ...
        sin(t)       cos(t)*cos(f)   -cos(t)*sin(f)  a*sin(t); ...
        0               sin(f)              cos(f)              d; ...
        0                   0                   0                   1];
end
