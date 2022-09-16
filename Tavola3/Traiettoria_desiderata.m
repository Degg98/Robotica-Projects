%% GENERAZIONE TRAIETTORIA DESIDERATA
% Voglio che il drone esegua una circonferenza a quota costante sul piano
% x-y con heading variabile
th_d = linspace(0,2*pi, 200001);
x_d = 2*cos(th_d); % m
y_d = 2*sin(th_d); % m
z_d = 2 + 0*th_d; % m
% Derivate prima delle uscite desiderate
dth_d = 2*pi/1000;
dx_d = -2*sin(th_d);
dy_d = 2*cos(th_d);
dz_d = 0;
% Derivate seconde delle uscite desiderate
ddth_d = 0;
ddx_d = -2*cos(th_d);
ddy_d = -2*sin(th_d);
ddz_d = 0;
% Derivate terze delle uscite desiderate
dddx_d = 2*sin(th_d);
dddy_d = -2*cos(th_d);
dddz_d = 0;
% Derivate quarte delle uscite desiderate
ddddx_d = 2*cos(th_d); % m
ddddy_d = 2*sin(th_d); % m
ddddz_d = 0;