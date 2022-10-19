%% PLOT DATI
close all
clear x_plot y_plot z_plot xd_plot yd_plot zd_plot
clc

n = length(out.tout);
% Estraggo le componenti di posizione
for i=1:n
x_plot(i) = out.x.signals.values(i,1);
y_plot(i) = out.y.signals.values(i,1);
z_plot(i) = out.z.signals.values(i,1);
xd_plot(i) = out.xd.signals.values(i,1);
yd_plot(i) = out.yd.signals.values(i,1);
zd_plot(i) = out.zd.signals.values(i,1);
end

% Plot inseguimento grandezze desiderate
t = linspace(0,10,n);
figure(1)
subplot(3,1,1)
title('x position')
plot(t, x_plot, t, xd_plot, 'LineWidth', 2)
legend('real', 'desired', 'Location','best')
xlabel('time [s]')
ylabel('x [m]')
grid on

subplot(3,1,2)
title('y position')
plot(t, y_plot, t, yd_plot, 'LineWidth', 2)
legend('real', 'desired', 'Location','best')
xlabel('time [s]')
ylabel('y [m]')
grid on

subplot(3,1,3)
title('z position')
plot(t, z_plot, t, zd_plot, 'LineWidth', 2)
legend('real', 'desired', 'Location','best')
xlabel('time [s]')
ylabel('z [m]')
grid on

% Plot delle posizioni del drone
figure(2)
plot3(x_plot,y_plot,z_plot, xd_plot, yd_plot, zd_plot, 'LineWidth', 2)
legend('real', 'desired', 'Location','best')
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
grid on