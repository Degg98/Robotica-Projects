%% STUDIO DELL'IDENTIFICABILITA' DI ALCUNI PARAMETRI DEL SISTEMA NON LINEARE

close all
clear
clc

syms x1 x2 x3 x4 x5 x6 x7 x8 g m Ixx u1 u2 real

% In questo caso si vuole identificare la massa (m) e il momento d'inerzia
% lungo l'asse x (Ixx) del drone.
% Per fare tutto questo è necessario riscrivere la dinamica includendo nel
% vettore di stato anche i parametri da identificare
% Tali parametri si suppongono costanti
% x7 = 1/m
% x8 = 1/Ixx

% Definisco il nuovo vettore di stato
x = [x1 x2 x3 x4 x5 x6 x7 x8]';
u = [u1 u2]';

% Il nuovo equilibrio
x_eq = [0 0 0 0 0 0 1/m 1/Ixx]';    % oppure x_eq = [0 0 pi 0 0 0 1/m 1/Ixx]'

% Descrivo il sistema in forma affine di controllo
% dx = f(x) + g1(x)*u1 + g2(x)*u2
f = [x4 x5 x6 0 -g 0 0 0]';
g1 = [0 0 0 -x7*sin(x3) x7*cos(x3) 0 0 0]';
g2 = [0 0 0 0 0 x8 0 0]';

% Descrivo le uscite provenienti dai sensori di posizione
h1 = x1;
h2 = x2;
dh1 = jacobian(h1, x);
dh2 = jacobian(h2, x);

% Definisco codistribuzione e distribuzione per la filtrazione di
% osservabilità
Delta = simplify([f g1 g2]);
Omega0 = simplify([dh1; dh2]);

fprintf('IDENTIFICABILITA MASSA E MOMENTO DI INERZIA\n')

% Calcolo la più piccola codistribuzione Delta-invariante che contiene
% Omega0, detta codistribuzione di osservabilità
% Filtrazione
fprintf('\nInizio filtrazione...\n')
% Passo 1
for i=1:size(Omega0,1) % Conto le righe di Omega_i
    Om11(i,:) = Ddir(f, Omega0(i,:), x);
    Om12(i,:) = Ddir(g1, Omega0(i,:), x);
    Om13(i,:) = Ddir(g2, Omega0(i,:), x);
end
Omega1 = [Omega0; Om11; Om12; Om13];
% Calcolo il rango di Omega1 valutato nel punto di equilibrio
fprintf('Dimensione Omega1')
R1 = rank(subs(Omega1, x, x_eq))

% Passo2
for i=1:size(Omega1,1)
    Om21(i,:) = Ddir(f, Omega1(i,:), x);
    Om22(i,:) = Ddir(g1, Omega1(i,:), x);
    Om23(i,:) = Ddir(g2, Omega1(i,:), x);
end
Omega2 = [Omega1; Om21; Om22; Om23];
% Calcolo il rango di Omega2 valutato nel punto di equilibrio
fprintf('Dimensione Omega2')
R2 = rank(subs(Omega2, x, x_eq))

% Passo3
for i=1:size(Omega2,1)
    Om31(i,:) = Ddir(f, Omega2(i,:), x);
    Om32(i,:) = Ddir(g1, Omega2(i,:), x);
    Om33(i,:) = Ddir(g2, Omega2(i,:), x);
end
Omega3 = [Omega2; Om31; Om32; Om33];
% Calcolo il rango di Omega3 valutato nel punto di equilibrio
fprintf('Dimensione Omega3')
R3 = rank(subs(Omega3, x, x_eq)) 

% Passo4
for i=1:size(Omega3,1)
    Om41(i,:) = Ddir(f, Omega3(i,:), x);
    Om42(i,:) = Ddir(g1, Omega3(i,:), x);
    Om43(i,:) = Ddir(g2, Omega3(i,:), x);
end
Omega4 = [Omega3; Om41; Om42; Om43];
% Calcolo il rango di Omega4 valutato nel punto di equilibrio
fprintf('Dimensione Omega4')
R4 = rank(subs(Omega4, x, x_eq))

% Passo5
for i=1:size(Omega4,1)
    Om51(i,:) = Ddir(f, Omega4(i,:), x);
    Om52(i,:) = Ddir(g1, Omega4(i,:), x);
    Om53(i,:) = Ddir(g2, Omega4(i,:), x);
end
Omega5 = [Omega4; Om51; Om52; Om53];
% Calcolo il rango di Omega5 valutato nel punto di equilibrio
fprintf('Dimensione Omega5')
R5 = rank(subs(Omega5, x, x_eq))

% Termine della filtrazione
fprintf('... termine filtrazione\n')
% Dalla teoria si sa che la codistribuzione di osservabilità è uguale a dO
% e se dim(dO) = 8 allora il sistema è localmente osservabile
fprintf(['\nIl sistema è localmente osservabile purché lo stato' ...
    ' x7 = 1/m non sia nullo\n'])

% Con x7 = 0, Omega4 ha rango 5 quindi il sistema non è osservabile,
% tuttavia se x7 = 0 => m = inf e questo non è possibile.
% Da notare che Ixx non altera l'osservabilità del sistema.
