%% Plot dei dati della simulazione
clc
close all

%% Confronto variabili di giunto con i loro riferimenti
n = length(out.tout);
for i=1:n
    % q desiderate
    q_des_1(i) = out.qdes.signals.values(i,1);
    q_des_2(i) = out.qdes.signals.values(i,2);
    q_des_3(i) = out.qdes.signals.values(i,3);
    q_des_4(i) = out.qdes.signals.values(i,4);
    q_des_5(i) = out.qdes.signals.values(i,5);
    q_des_6(i) = out.qdes.signals.values(i,6);
    % q effettive
    q_plot_1(i) = out.q.signals.values(1,1,i);
    q_plot_2(i) = out.q.signals.values(2,1,i);
    q_plot_3(i) = out.q.signals.values(3,1,i);
    q_plot_4(i) = out.q.signals.values(4,1,i);
    q_plot_5(i) = out.q.signals.values(5,1,i);
    q_plot_6(i) = out.q.signals.values(6,1,i);
end
t = linspace(0,15,15001);
figure(1)
% Giunto 1
subplot(2,3,1)
hold on
plot(t, q_des_1)
plot(t, q_plot_1)
legend('desired value', 'real value', 'Location','best')
title('Joint 1')
xlabel('[sec]')
ylabel('[rad]')
% xlim([0 2e4])
grid on

% Giunto 2
subplot(2,3,2)
hold on
plot(t, q_des_2)
plot(t, q_plot_2)
legend('desired value', 'real value', 'Location','best')
title('Joint 2')
xlabel('[sec]')
ylabel('[rad]')
% xlim([0 2e4])
grid on

% Giunto 3
subplot(2,3,3)
hold on
plot(t, q_des_3)
plot(t, q_plot_3)
legend('desired value', 'real value', 'Location','best')
title('Joint 3')
xlabel('[sec]')
ylabel('[rad]')
% xlim([0 2e4])
grid on

% Giunto 4
subplot(2,3,4)
hold on
plot(t, q_des_4)
plot(t, q_plot_4)
legend('desired value', 'real value', 'Location','best')
title('Joint 4')
xlabel('[sec]')
ylabel('[rad]')
% xlim([0 2e4])
grid on

% Giunto 5
subplot(2,3,5)
hold on
plot(t, q_des_5)
plot(t, q_plot_5)
legend('desired value', 'real value', 'Location','best')
title('Joint 5')
xlabel('[sec]')
ylabel('[rad]')
% xlim([0 2e4])
grid on

% Giunto 6
subplot(2,3,6)
hold on
plot(t, q_des_6)
plot(t, q_plot_6)
legend('desired value', 'real value', 'Location','best')
title('Joint 6')
xlabel('[sec]')
ylabel('[rad]')
% xlim([0 2e4])
grid on

%% Confronto della posizione assunta dall'EE rispetto al riferimento
n = length(out.tout);

for i=1:n
    posEE_x(i) = out.pos_EE.signals.values(1,1,i);
    posEE_y(i) = out.pos_EE.signals.values(2,1,i);
    posEE_z(i) = out.pos_EE.signals.values(3,1,i);
end

figure(2)
t = linspace(0,15,15001);

% X End Effector
subplot(1,3,1)
hold on
plot(t, pfin(1)+0*t)
plot(t, posEE_x)
legend('desired value', 'real value', 'Location','best')
title('X End Effector')
xlabel('[sec]')
ylabel('[m]')
grid on

% Y End Effector
subplot(1,3,2)
hold on
plot(t, pfin(2)+0*t)
plot(t, posEE_y)
legend('desired value', 'real value', 'Location','best')
title('Y End Effector')
xlabel('[sec]')
ylabel('[m]')
grid on

% Z End Effector
subplot(1,3,3)
hold on
plot(t, pfin(3)+0*t)
plot(t, posEE_z)
legend('desired value', 'real value', 'Location','best')
title('Z End Effector')
xlabel('[sec]')
ylabel('[m]')
grid on

%% Andamento degli errori
n = length(out.tout);
for i=1:n
    e_1(i) = out.error.signals.values(1,1,i);
    e_2(i) = out.error.signals.values(2,1,i);
    e_3(i) = out.error.signals.values(3,1,i);
    e_4(i) = out.error.signals.values(4,1,i);
    e_5(i) = out.error.signals.values(5,1,i);
    e_6(i) = out.error.signals.values(6,1,i);
end

figure(3)
hold on
plot(t, e_1)
plot(t, e_2)
plot(t, e_3)
plot(t, e_4)
plot(t, e_5)
plot(t, e_6)
legend('e1', 'e2', 'e3', 'e4', 'e5', 'e6', 'Location','best')
title('Joint variable errors')
xlabel('[sec]')
ylabel('[rad]')
ylim([-0.2 0.2])
grid on

%% Andamento delle coppie ai giunti
% NB: ciascun giunto ha una coppia massima che può applicare e tale vincolo
% è stato rispettato mettendo delle saturazioni
% tau_1 max = 330 Nm
% tau_2 max = 330 Nm
% tau_3 max = 150 Nm
% tau:4 max = 56 Nm
% tau:5 max = 56 Nm
% tau:6 max = 56 Nm

n = length(out.tout);

for i=1:n
    tau_1(i) = out.controls.signals.values(i,1);
    tau_2(i) = out.controls.signals.values(i,2);
    tau_3(i) = out.controls.signals.values(i,3);
    tau_4(i) = out.controls.signals.values(i,4);
    tau_5(i) = out.controls.signals.values(i,5);
    tau_6(i) = out.controls.signals.values(i,6);
end

figure(4)
hold on
plot(t, tau_1)
plot(t, tau_2)
plot(t, tau_3)
plot(t, tau_4)
plot(t, tau_5)
plot(t, tau_6)
legend('\tau_1', '\tau_2', '\tau_3', '\tau_4', '\tau_5', '\tau_6', 'Location','best')
title('Joint torque')
xlabel('[sec]')
ylabel('[Nm]')
grid on

%% Confronto parametri stimati
n = length(out.tout);
for i=1:n
    % masse stimate
    m1_est(i) = out.estimation_params.signals.values(1,1,i);
    m2_est(i) = out.estimation_params.signals.values(2,1,i);
    m3_est(i) = out.estimation_params.signals.values(3,1,i);
    m4_est(i) = out.estimation_params.signals.values(4,1,i);
    m5_est(i) = out.estimation_params.signals.values(5,1,i);
    m6_est(i) = out.estimation_params.signals.values(6,1,i);
end
t = linspace(0,15,15001);
figure(5)
% Massa 1
subplot(2,3,1)
hold on
plot(t, m1+0*t)
plot(t, m1_est)
legend('desired value', 'real value', 'Location','best')
title('Mass 1')
xlabel('[sec]')
ylabel('[kg]')
% xlim([0 2e4])
grid on

% Giunto 2
subplot(2,3,2)
hold on
plot(t, m2+0*t)
plot(t, m2_est)
legend('desired value', 'real value', 'Location','best')
title('Mass 2')
xlabel('[sec]')
ylabel('[kg]')
% xlim([0 2e4])
grid on

% Giunto 3
subplot(2,3,3)
hold on
plot(t, m3+0*t)
plot(t, m3_est)
legend('desired value', 'real value', 'Location','best')
title('Mass 3')
xlabel('[sec]')
ylabel('[kg]')
% xlim([0 2e4])
grid on

% Giunto 4
subplot(2,3,4)
hold on
plot(t, m4+0*t)
plot(t, m4_est)
legend('desired value', 'real value', 'Location','best')
title('Mass 4')
xlabel('[sec]')
ylabel('[kg]')
% xlim([0 2e4])
grid on

% Giunto 5
subplot(2,3,5)
hold on
plot(t, m5+0*t)
plot(t, m5_est)
legend('desired value', 'real value', 'Location','best')
title('Mass 5')
xlabel('[sec]')
ylabel('[kg]')
% xlim([0 2e4])
grid on

% Giunto 6
subplot(2,3,6)
hold on
plot(t, m6+0*t)
plot(t, m6_est)
legend('desired value', 'real value', 'Location','best')
title('Mass 6')
xlabel('[sec]')
ylabel('[kg]')
% xlim([0 2e4])
grid on