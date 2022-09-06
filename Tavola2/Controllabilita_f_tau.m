%% STUDIO DELLA CONTROLLABILITA DEL SISTEMA NON LINEARE

clear
close all
clc

Descrizione_Drone_2D

%% ACCESSIBILITA
% Verifico l'accessibilità del sistema calcolando la distribuzione di
% accessibilità
Delta0 = simplify([g1 g2]);
Delta = simplify([f g1 g2]);

% Filtrazione
fprintf('Inizio filtrazione...\n')
% Passo 1
for i=1:size(Delta0,2)
    D11(:,i) = LieBrackets(Delta0(:,i), f, x);
    D12(:,i) = LieBrackets(Delta0(:,i), g1, x);
    D13(:,i) = LieBrackets(Delta0(:,i), g2, x);
end
Delta1 = [Delta0 D11 D12 D13];
% Calcolo il rango di Delta1 valutato nel punto di equilibrio
fprintf('Dimensione Delta1')
R1 = rank(subs(Delta1, x, x_eq))

% Passo2
for i=1:size(Delta1,2)
    D21(:,i) = LieBrackets(Delta1(:,i), f, x);
    D22(:,i) = LieBrackets(Delta1(:,i), g1, x);
    D23(:,i) = LieBrackets(Delta1(:,i), g2, x);
end
Delta2 = [Delta1 D21 D22 D23];
% Calcolo il rango di Delta2 valutato nel punto di equilibrio
fprintf('Dimensione Delta2')
R2 = rank(subs(Delta2, x, x_eq))

% Passo3
for i=1:size(Delta2,2)
    D31(:,i) = LieBrackets(Delta2(:,i), f, x);
    D32(:,i) = LieBrackets(Delta2(:,i), g1, x);
    D33(:,i) = LieBrackets(Delta2(:,i), g2, x);
end
Delta3 = [Delta2 D31 D32 D33];
% Calcolo il rango di Delta3 valutato nel punto di equilibrio
fprintf('Dimensione Delta3')
R3 = rank(subs(Delta3, x, x_eq)) 

% Passo4
for i=1:size(Delta3,2)
    D41(:,i) = LieBrackets(Delta3(:,i), f, x);
    D42(:,i) = LieBrackets(Delta3(:,i), g1, x);
    D43(:,i) = LieBrackets(Delta3(:,i), g2, x);
end
Delta4 = [Delta3 D41 D42 D43];
% Calcolo il rango di Delta4 valutato nel punto di equilibrio
fprintf('Dimensione Delta4')
R4 = rank(subs(Delta4, x, x_eq))

% Termine della filtrazione
fprintf('... termine filtrazione\n')

fprintf('Il sistema è accessibile da entrambi gli ingressi\n')

%% DA ACCESSIBILITA A CONTROLLABILITA
% Per valutare la controllabilità sfrutto il punto iii) della teoria
f0 = subs(f, x, x_eq);

% Poichè è nullo procedo con il calcolo di DeltaL
% Dalla teoria si trova che DeltaL coincide con la matrice di
% raggiungibilità del sistema linearizzato

% g = 9.81;
% % m = ?;
% % Ixx = ?;
% 
% An = subs(A);
% Bn = subs(B);
% % Matrice di raggiungibilità
% R = ctrb(An, Bn);
% Rr = rank(R);
% fprintf('Il rango di DeltaL vale %d', Rr)
% fprintf('Il sistema è localmente controllabile in small-time')

