function J = J_3r(al2,al3,d2,d3,q1,q2,q3)
%J_3r
%    J = J_3r(AL2,AL3,D2,D3,Q1,Q2,Q3)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    02-Sep-2022 11:48:22

t2 = cos(q1);
t3 = cos(q2);
t4 = sin(q1);
t5 = sin(q2);
t6 = d2+d3;
t7 = q2+q3;
t8 = al2.*t3;
t9 = cos(t7);
t10 = al2.*t5;
t11 = sin(t7);
t12 = -t4;
t13 = al3.*t9;
t14 = al3.*t11;
t15 = -t13;
t16 = t8+t13;
t17 = t10+t14;
J = reshape([-t2.*t6+t12.*t16,t2.*t16+t6.*t12,0.0,0.0,0.0,1.0,-t2.*t17,t12.*t17,-t8+t15,t12,t2,0.0,-t2.*t14,t12.*t14,t15,t12,t2,0.0],[6,3]);
