%generazione variabili di giunto per task di trajectory tracking

cpts = [1 4 4 3 -2 0; 0 1 2 4 3 1]; %way-points
tpts = [0 5]; %time-points

%calcolo della traiettoria B-Spline
tvec = 0:0.01:5;

[q_des_fin, qd_des_fin, qdd_des_fin, pp] = bsplinepolytraj(cpts,tpts,tvec);


%plot
figure
plot(cpts(1,:),cpts(2,:),'xb-')
hold all
plot(q_des_fin(1,:), q_des_fin(2,:))
xlabel('X')
ylabel('Y')
hold off

figure
plot(tvec,q_des_fin)
hold all
plot([0:length(cpts)-1],cpts,'x')
xlabel('t')
ylabel('Position Value')
legend('X-positions','Y-positions')
hold off

 
 