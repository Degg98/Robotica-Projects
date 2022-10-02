% Calcolo N = B_dot - 2C
n = size(q');
Nmat = sym(zeros(size(B)));
for i = 1 : n
    for j = 1 : n
        for k = 1 : n
            Nmat(i,j) = Nmat(i,j) + ( (jacobian(B(j,k), q(i)) - jacobian(B(i,k), q(j))) * dq(k));
        end
    end
end

N = sym(zeros(size(Nmat)));
for i = 1:n
    for j = 1:n
        disp(i)
        
        disp(j)
        N(i,j) = simplify(Nmat(i,j));
    end
end

test = dq' * N * dq
subs(test)