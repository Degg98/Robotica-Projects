function test(Delta, Gamma0, x, x_eq)
% Eseguo la filtrazione per n-1 passi (11 passi)
fprintf('Inizio filtrazione...\n')
% Passo 1
for i=1:size(Gamma0,2)
    G1(:,i) = LieBrackets(Delta, Gamma0(:,i), x);
end
Gamma1 = [Gamma0 G1];
% Calcolo le condizioni del teorema su Gamma1
fprintf('Condizioni su Gamma1')
involuzione(Gamma1, x)
T12 = rank(Gamma1) - rank(subs(Gamma1, x, x_eq))

% Passo2
for i=1:size(Gamma1,2)
    G2(:,i) = LieBrackets(Delta, Gamma1(:,i), x);
end
Gamma2 = [Gamma1 G2];
% Calcolo le condizioni del teorema su Gamma2
fprintf('Condizioni su Gamma2')
involuzione(Gamma2, x)
T22 = rank(Gamma2) - rank(subs(Gamma2, x, x_eq))

% Passo3
for i=1:size(Gamma2,2)
    G3(:,i) = LieBrackets(Delta, Gamma2(:,i), x);
end
Gamma3 = [Gamma2 G3];
% Calcolo le condizioni del teorema su Gamma3
fprintf('Condizioni su Gamma3')
involuzione(Gamma3, x)
T23 = rank(Gamma3) - rank(subs(Gamma3, x, x_eq)) 

% Passo 4
for i=1:size(Gamma3,2)
    G4(:,i) = LieBrackets(Delta, Gamma3(:,i), x);
end
Gamma4 = [Gamma3 G4];
% Calcolo le condizioni del teorema su Gamma4
fprintf('Condizioni su Gamma4')
involuzione(Gamma4, x)
T24 = rank(Gamma4) - rank(subs(Gamma4, x, x_eq))

% Passo 5
for i=1:size(Gamma4,2)
    G5(:,i) = LieBrackets(Delta, Gamma4(:,i), x);
end
Gamma5 = [Gamma4 G5];
% Calcolo le condizioni del teorema su Gamma5
fprintf('Condizioni su Gamma5')
involuzione(Gamma5, x)
T25 = rank(Gamma5) - rank(subs(Gamma5, x, x_eq))

% Passo 6
for i=1:size(Gamma5,2)
    G6(:,i) = LieBrackets(Delta, Gamma5(:,i), x);
end
Gamma6 = [Gamma5 G6];
% Calcolo le condizioni del teorema su Gamma6
fprintf('Condizioni su Gamma6')
involuzione(Gamma6, x)
T26 = rank(Gamma6) - rank(subs(Gamma6, x, x_eq))

% Passo 7
for i=1:size(Gamma6,2)
    G7(:,i) = LieBrackets(Delta, Gamma6(:,i), x);
end
Gamma7 = [Gamma6 G7];
% Calcolo le condizioni del teorema su Gamma7
fprintf('Condizioni su Gamma7')
involuzione(Gamma7, x)
T27 = rank(Gamma7) - rank(subs(Gamma7, x, x_eq))

% Passo 8
for i=1:size(Gamma7,2)
    G8(:,i) = LieBrackets(Delta, Gamma7(:,i), x);
end
Gamma8 = [Gamma7 G8];
% Calcolo le condizioni del teorema su Gamma8
fprintf('Condizioni su Gamma8')
involuzione(Gamma8, x)
T28 = rank(Gamma8) - rank(subs(Gamma8, x, x_eq))

% Passo 9
for i=1:size(Gamma8,2)
    G9(:,i) = LieBrackets(Delta, Gamma8(:,i), x);
end
Gamma9 = [Gamma8 G9];
% Calcolo le condizioni del teorema su Gamma9
fprintf('Condizioni su Gamma9')
involuzione(Gamma9, x)
T29 = rank(Gamma9) - rank(subs(Gamma9, x, x_eq))

% Passo 10
for i=1:size(Gamma9,2)
    G10(:,i) = LieBrackets(Delta, Gamma9(:,i), x);
end
Gamma10 = [Gamma9 G10];
% Calcolo le condizioni del teorema su Gamma10
fprintf('Condizioni su Gamma10')
involuzione(Gamma10, x)
T210 = rank(Gamma10) - rank(subs(Gamma10, x, x_eq))

% Passo 11
for i=1:size(Gamma10,2)
    G11(:,i) = LieBrackets(Delta, Gamma10(:,i), x);
end
Gamma11 = [Gamma10 G11];
% Calcolo le condizioni del teorema su Gamma11
fprintf('Condizioni su Gamma11')
involuzione(Gamma11, x)
T211 = rank(Gamma11) - rank(subs(Gamma11, x, x_eq))

% Verifico la seconda condizione
fprintf('Dimensione Gamma11')
T2 = rank(Gamma11)
























% Termine della filtrazione
fprintf('... termine filtrazione\n')

fprintf('Il sistema è accessibile da entrambi gli ingressi\n')