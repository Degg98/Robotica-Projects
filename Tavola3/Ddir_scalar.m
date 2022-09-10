function L = Ddir_scalar(f, lambda, state)
% Calcola la derivata direzionale di una funzione scalare lambda 
% lungo il vettore f
    L = jacobian(lambda, state) * f;
end