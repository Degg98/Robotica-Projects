function [ Tf ] = DH_Kynematics( T_base, T_ee, DH_table)

n_link = size(DH_table,1);
T = T_base;

for i=1:n_link
    b1 = DH_matrix(DH_table(i,:));
    T =  T*b1;
end

Tf =  T*T_ee;

end

