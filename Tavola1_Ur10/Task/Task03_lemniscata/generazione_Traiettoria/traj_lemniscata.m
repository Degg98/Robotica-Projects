
figure(1)
t = linspace(0, 25, 200);
% Semi-distanza tra i fuochi
a = 0.3 + 0*t;
% Lemniscata di Bernoulli
x = 0.5 * a .* sin(t) .* sqrt(cos(2*t)) + 0.2;
y = 0.5 * a .* cos(t) .* sqrt(cos(2*t));
z = 0*t + 0.2;
plot3(x,y,z, 'LineWidth', 2.5);
xlabel('x')
ylabel('y')
zlabel('z')
count = length(t);
grid on

%% Definizione a punti della traiettoria
%traiettoria
points2 = [ x' y' z'];

% TRAIETTORIA FINALE POSIZIONE E ORIENTAZIONE
TrajectoryPos = zeros([count 4]);
TrajectoryPos(:,1) = t';
TrajectoryPos(:,2:4) = points2;

TrajectoryOr = zeros([count 4]);
TrajectoryOr(:,1) = t';
TrajectoryOr(:,2) = pi;
TrajectoryOr(:,3) = pi;
TrajectoryOr(:,4) = pi;

hold on, grid on,
plot3(TrajectoryPos(:,2),TrajectoryPos(:,3),TrajectoryPos(:,4),'b.')
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');

%posizione e rotazione EE lungo la traiettoria
PosEE = TrajectoryPos(:,2:4);
RotEE = eul2rotm(TrajectoryOr(:,2:4));

%conversione dei punti della traiettoria in cordinate omogenee
T = transl(PosEE);

for i = 1:length(PosEE)
    T(1:3,1:3,i) = RotEE(:,:,i);  
end

%ricavo le variabili di giunto desiderate attraverso la funzione ikine del
%robotic toolbox
Q_des = ur10.ikine(T, 'search', true);