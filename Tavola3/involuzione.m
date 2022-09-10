function ris = involuzione(Gamma, x)
% n = numero di colonne di Gamma
    n = size(Gamma,2);
    h = 1;
    % LieBracket(Gamma, Gamma)
    for i=1:n-1
        for j=i+1:n
            I(:,h) = LieBrackets(Gamma(:,i), Gamma(:,j), x);
            h = h + 1;
        end
    end
    Gamma_ = [Gamma I];
    ris = rank(Gamma) - rank(Gamma_);
end