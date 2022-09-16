%% PLOT DATI
n = length(out.tout);
% Estraggo le componenti di posizione
for i=1:100:n
x_plot(i) = out.x.signals.values(1,1,i);
y_plot(i) = out.y.signals.values(1,1,i);
z_plot(i) = out.z.signals.values(1,1,i);
end

% Plot delle posizioni del drone
plot3(x_plot,y_plot,z_plot)