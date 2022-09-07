%% STUDIO DELL'OSSERVABILITA' DEL SISTEMA NON LINEARE

close all
clear
clc

Descrizione_Drone_2D
%%% ---------------------------------------------------------------------
% Verifico l'osservabilità del sistema sfruttando l'approccio geometrico
% differenziale
% Saranno analizzati quattro casi:
% 1) Sensori di posizione
% 2) Sensori di velocità lineare
% 3) Entrambe le tipologie di sensori
% 4) Sensori di velocità sia lineare che angolare
%%% ---------------------------------------------------------------------

Delta = simplify([f g1 g2]);

%% 1) Sensori di posizione
fprintf('ANALISI CON SENSORI DI POSIZIONE\n')
h1 = x1;
h2 = x2;
dh1 = jacobian(h1, x);
dh2 = jacobian(h2, x);
Omega0 = simplify([dh1; dh2]);

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

% Termine della filtrazione
fprintf('... termine filtrazione\n')
% Dalla teoria si sa che la codistribuzione di osservabilità è uguale a dO
% e se dim(dO) = 6 allora il sistema è localmente osservabile
fprintf(['\nIl sistema è localmente osservabile se si prendono come uscite le posizioni ' ...
    'velivolo \n'])

txt = '\nPremere invio per continuare\n';
invio = input(txt);
fprintf('----------------------------------------------------------------------------\n')

%% 2) Sensori di velocità lineare
fprintf('ANALISI CON SENSORI DI VELOCITA LINEARE\n')
h1 = x4;
h2 = x5;
dh1 = jacobian(h1, x);
dh2 = jacobian(h2, x);
Omega0 = simplify([dh1; dh2]);

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

% Termine della filtrazione
fprintf('... termine filtrazione\n')
% Dalla teoria si sa che la codistribuzione di osservabilità è uguale a dO
% e se dim(dO) = 6 allora il sistema è localmente osservabile
fprintf(['\nIl sistema non è osservabile se si prendono come uscite ' ...
    'le velocità lineari del velivolo \n'])

txt = '\nPremere invio per continuare\n';
invio = input(txt);
fprintf('----------------------------------------------------------------------------\n')


%% 3) Sensori di posizione e di velocità lineare
fprintf('ANALISI CON SENSORI DI POSIZIONE E DI VELOCITA LINEARE\n')
h1 = x1;
h2 = x2;
h3 = x4;
h4 = x5;
dh1 = jacobian(h1, x);
dh2 = jacobian(h2, x);
dh3 = jacobian(h3, x);
dh4 = jacobian(h4, x);
Omega0 = simplify([dh1; dh2; dh3; dh4]);

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

% Passo 2
for i=1:size(Omega1,1)
    Om21(i,:) = Ddir(f, Omega1(i,:), x);
    Om22(i,:) = Ddir(g1, Omega1(i,:), x);
    Om23(i,:) = Ddir(g2, Omega1(i,:), x);
end
Omega2 = [Omega1; Om21; Om22; Om23];
% Calcolo il rango di Omega2 valutato nel punto di equilibrio
fprintf('Dimensione Omega2')
R2 = rank(subs(Omega2, x, x_eq))

% Passo 3
for i=1:size(Omega2,1)
    Om31(i,:) = Ddir(f, Omega2(i,:), x);
    Om32(i,:) = Ddir(g1, Omega2(i,:), x);
    Om33(i,:) = Ddir(g2, Omega2(i,:), x);
end
Omega3 = [Omega2; Om31; Om32; Om33];
% Calcolo il rango di Omega3 valutato nel punto di equilibrio
fprintf('Dimensione Omega3')
R3 = rank(subs(Omega3, x, x_eq)) 

% Termine della filtrazione
fprintf('... termine filtrazione\n')
% Dalla teoria si sa che la codistribuzione di osservabilità è uguale a dO
% e se dim(dO) = 6 allora il sistema è localmente osservabile
fprintf(['\nIl sistema è localmente osservabile se si prendono come uscite sia le posizioni ' ...
    ' che le velocità lineari del velivolo \n'])


txt = '\nPremere invio per continuare\n';
invio = input(txt);
fprintf('----------------------------------------------------------------------------\n')

%% 4) Sensori di velocità lineare e angolare
fprintf('ANALISI CON SENSORI DI VELOCITA LINEARE E ANGOLARE\n')
h1 = x4;
h2 = x5;
h3 = x6;
dh1 = jacobian(h1, x);
dh2 = jacobian(h2, x);
dh3 = jacobian(h3, x);

Omega0 = simplify([dh1; dh2; dh3]);

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

% Passo 2
for i=1:size(Omega1,1)
    Om21(i,:) = Ddir(f, Omega1(i,:), x);
    Om22(i,:) = Ddir(g1, Omega1(i,:), x);
    Om23(i,:) = Ddir(g2, Omega1(i,:), x);
end
Omega2 = [Omega1; Om21; Om22; Om23];
% Calcolo il rango di Omega2 valutato nel punto di equilibrio
fprintf('Dimensione Omega2')
R2 = rank(subs(Omega2, x, x_eq))

% Passo 3
for i=1:size(Omega2,1)
    Om31(i,:) = Ddir(f, Omega2(i,:), x);
    Om32(i,:) = Ddir(g1, Omega2(i,:), x);
    Om33(i,:) = Ddir(g2, Omega2(i,:), x);
end
Omega3 = [Omega2; Om31; Om32; Om33];
% Calcolo il rango di Omega3 valutato nel punto di equilibrio
fprintf('Dimensione Omega3')
R3 = rank(subs(Omega3, x, x_eq)) 

% Termine della filtrazione
fprintf('... termine filtrazione\n')
% Dalla teoria si sa che la codistribuzione di osservabilità è uguale a dO
% e se dim(dO) = 6 allora il sistema è localmente osservabile
fprintf(['\nIl sistema non è osservabile se si prendono come uscite sia ' ...
    ' le velocità lineari che quella angolare del velivolo \n'])