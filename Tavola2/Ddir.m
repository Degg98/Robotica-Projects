function L = Ddir(f, w, state)
% Calcolo la derivata direzionale di un covettore w lungo il vettoref
    diff_f = jacobian(f, state);
    diff_w = jacobian(w', state);
    L = (f')*diff_w' + w*diff_f;
end