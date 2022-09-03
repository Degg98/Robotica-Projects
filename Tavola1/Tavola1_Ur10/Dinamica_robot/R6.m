
%parameters
%link length [m]
l1 = 0.1273;
l2 = -0.6120;
l3 = -0.5723;
l4 = 0.1639;
l5 = 0.1157;
l6 = 0.0922;

%link mass [Kg]
m1 = 7.10;
m2 = 12.70;
m3 = 4.27;
m4 = 2.00;
m5 = 2.00;
m6 = 0.365;

%link radius [m]
r = 0.02;

g = -9.81; %[m/s^2]

% Calcolo i tensori d'inerzia in terna baricentrica e principale d'inerzia
I1 = Inertia(m1, l1, r);
I2 = Inertia(m2, l2, r);
I3 = Inertia(m3, l3, r);
I4 = Inertia(m4, l4, r);
I5 = Inertia(m5, l5, r);
I6 = Inertia(m6, l6, r);

% Costruisco i giunti in base a DH
L(1) = Revolute( 'd', l1, 'a', 0, 'alpha', pi/2, 'offset', 0, 'standard');
L(2) = Revolute( 'd', 0, 'a', l2, 'alpha', 0, 'offset', 0, 'standard');
L(3) = Revolute( 'd', 0, 'a', l3, 'alpha', 0, 'offset', 0, 'standard');
L(4) = Revolute( 'd', l4, 'a', 0, 'alpha', pi/2, 'offset', 0, 'standard');
L(5) = Revolute( 'd', l5, 'a', 0, 'alpha', -pi/2, 'offset',0, 'standard');
L(6) = Revolute( 'd', l6, 'a', 0, 'alpha', 0, 'offset', 0, 'standard');

% Masse dei link
L(1).m = m1;
L(2).m = m2;
L(3).m = m3;
L(4).m = m4;
L(5).m = m5;
L(6).m = m6;

% CdM dei link
L(1).r = [ 0 0 l1/2];
L(2).r = [ 0 0 l2/2];
L(3).r = [ 0 0 l3/2];
L(4).r = [ 0 0 l4/2];
L(5).r = [ 0 0 l5/2];
L(6).r = [ 0 0 l6/2];

% Tensori d'inerzia dei link
L(1).I = I1;
L(2).I = I2;
L(3).I = I3;
L(4).I = I4;
L(5).I = I5;
L(6).I = I6;

% %viscous friction
% L(1).B = 0.0000017;
% L(2).B = 0.0000017;
% L(3).B = 0.0000017;
% L(4).B = 0.0000017;
% L(5).B = 0.0000017;
% L(6).B = 0.0000017;
% 
% %coulomb friction
% L(1).Tc = [0.0008 -0.0008];
% L(2).Tc = [0.0008 -0.0008];
% L(3).Tc = [0.0008 -0.0008];
% L(4).Tc = [0.0008 -0.0008];
% L(5).Tc = [0.0008 -0.0008];
% L(6).Tc = [0.0008 -0.0008];


% Costruisco il robot
ur10 = SerialLink(L, 'name', 'Ur10');
q0 = [0 0 0 0 0 0];
plot3(0,0,0);
ur10.plot(q0)
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');

% %xlim([-0.7,0.7])
% %ylim([-0.7,0.7])
% %zlim([0,1])
