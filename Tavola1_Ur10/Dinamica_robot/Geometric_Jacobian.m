function J = Geometric_Jacobian(Ts0, Tee, DH_table, k)
Tr = DH_Kynematics(Ts0, Tee, DH_table);
zi=[]; pi=[];
for i = 1 : k
    Tr_i = DH_Kynematics(Ts0, Tee, DH_table(1:i-1,:));
    zi =[zi, Tr_i(1:3, 3)];
    pi =[pi, Tr(1:3,4) - Tr_i(1:3,4)];


    jv = cross(zi, pi);
    jo = zi;

    J = [jv; jo];
end

end