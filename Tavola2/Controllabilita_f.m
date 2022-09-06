%% STUDIO DELLA CONTROLLABILITA DEL SISTEMA NON LINEARE

clear
close all
clc

Descrizione_Drone_2D

%% ACCESSIBILITA
% Verifico l'accessibilità del sistema calcolando la distribuzione di
% accessibilità considerando solo il primo ingresso
Delta0 = simplify([g1]);
Delta = simplify([f g1]);

% Filtrazione
fprintf('Inizio filtrazione...\n')
% Passo 1
for i=1:size(Delta0,2)
    D11(:,i) = LieBrackets(Delta0(:,i), f, x);
    D12(:,i) = LieBrackets(Delta0(:,i), g1, x);
end
Delta1 = [Delta0 D11 D12];
% Calcolo il rango di Delta1 valutato nel punto di equilibrio
fprintf('Dimensione Delta1')
R1 = rank(subs(Delta1, x, x_eq))

% Passo2
for i=1:size(Delta1,2)
    D21(:,i) = LieBrackets(Delta1(:,i), f, x);
    D22(:,i) = LieBrackets(Delta1(:,i), g1, x);
end
Delta2 = [Delta1 D21 D22];
% Calcolo il rango di Delta2 valutato nel punto di equilibrio
fprintf('Dimensione Delta2')
R2 = rank(subs(Delta2, x, x_eq))

% Termine della filtrazione
fprintf('... termine filtrazione\n')

fprintf('Il sistema non è accessibile dal primo ingresso (f)\n')