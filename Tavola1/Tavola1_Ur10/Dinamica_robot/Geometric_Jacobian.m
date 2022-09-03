function J = Geometric_Jacobian(Ts0, Tee, DH_table, k)
% Calcolo il Jacobiano geometrico associato al manipolatore
% Tr = matrice di trasformazione dal frame 0 al frame EE
Tr = DH_Kynematics(Ts0, Tee, DH_table);
zi=[]; pi=[];
for i = 1 : k
    % Tr_i = matrice di trasformazione dal 0 al frame i
    Tr_i = DH_Kynematics(Ts0, Tee, DH_table(1:i-1,:));
    % Aggiungo le orientazioni dei vari frame rispetto al frame 0
    zi =[zi, Tr_i(1:3, 3)];
    % Aggiungo le posizioni relative del frame EE rispetto al frame i
    pi =[pi, Tr(1:3,4) - Tr_i(1:3,4)];

    % Jacobiano di posizione (orientazione x posiz. relativa origine)
    jv = cross(zi, pi);
    % Jacobiano di orientazione
    jo = zi;

    J = [jv; jo];
end

end