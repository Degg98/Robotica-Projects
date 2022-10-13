
figure(1)
t = linspace(0, 15, 500);
% Semi-distanza tra i fuochi
a = 0.3;
% Lemniscata di Bernoulli
% x = 0.8 * a * sin(1*t) .* real(sqrt(cos(15*t))) + 0.5;
% y = 0.8 * a * cos(1*t) .* real(sqrt(cos(15*t))) + 0.5;
y = a*cos(t) ./ (1+(sin(t).^2)) + 0.5;
x = a*sin(t).*cos(t) ./ (1+(sin(t).^2)) + 0.5;
z = 0.00001*t + 0.5;
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
Q_des = ur10.ikine(T);