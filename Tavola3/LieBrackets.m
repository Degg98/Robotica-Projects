function [L] = LieBrackets(f, g, state)
% Funzione che calcola la Lie Bracket tra due oggetti simbolici
    d_g = jacobian(g, state);
    d_f = jacobian(f, state);
    L = d_g*f - d_f*g;
end