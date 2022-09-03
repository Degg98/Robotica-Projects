function Y_t = Y_fun(a2,a3,al2,al3,d2,d3,ddq1,ddq2,ddq3,dq1,dq2,dq3,g,m2,q2,q3)
%Y_fun
%    Y_t = Y_fun(A2,A3,AL2,AL3,D2,D3,DDQ1,DDQ2,DDQ3,DQ1,DQ2,DQ3,G,M2,Q2,Q3)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    02-Sep-2022 11:48:54

t2 = cos(q2);
t3 = cos(q3);
t4 = sin(q2);
t5 = sin(q3);
t6 = a2+al2;
t7 = a3+al3;
t8 = d2+d3;
t9 = q2+q3;
t10 = a2.^2;
t11 = a3.^2;
t12 = al2.^2;
t13 = al3.^2;
t14 = d2.^2;
t15 = dq1.^2;
t16 = m2+1.0;
t17 = q2.*2.0;
t18 = q3.*2.0;
t19 = a2.*al2.*2.0;
t20 = a3.*al3.*2.0;
t21 = a3.*al3.*4.0;
t22 = t2.^2;
t23 = sin(t17);
t24 = cos(t9);
t25 = sin(t9);
t26 = t11.*2.0;
t27 = t13.*2.0;
t28 = q2+t9;
t31 = t9.*2.0;
t32 = al2.*t3.*t7;
t33 = g.*t2.*t6;
t34 = d2.*ddq1.*t4.*t6;
t38 = t10+t12+t19+1.2625425e-1;
t39 = t10+t12+t19+1.2344175e-1;
t29 = cos(t28);
t30 = sin(t28);
t35 = cos(t31);
t36 = sin(t31);
t37 = -t33;
t40 = ddq2.*t38;
t41 = t21+t26+t27+2.155390266666667e-1;
t42 = t11+t13+t20+t32+1.105820133333333e-1;
t43 = t36.*t41;
mt1 = [ddq1.*2.8125e-3,0.0,0.0,(ddq1.*(t14.*5.62949953421312e+16+t22.*6.949152741274523e+15+t10.*t22.*5.62949953421312e+16+t12.*t22.*5.62949953421312e+16+a2.*al2.*t22.*1.125899906842624e+17+1.58329674399744e+14))./5.62949953421312e+16,t34+t37+t40+t2.*t4.*t15.*t39,0.0];
mt2 = [ddq1.*5.669725666666667e-2+(ddq1.*t11)./2.0+(ddq1.*t12)./2.0+(ddq1.*t13)./2.0+ddq1.*t14+ddq1.*t35.*5.388475666666667e-2+d3.^2.*ddq1+(ddq1.*t12.*cos(t17))./2.0+a3.*al3.*ddq1+d2.*d3.*ddq1.*2.0+(ddq1.*t11.*t35)./2.0+(ddq1.*t13.*t35)./2.0+a3.*al2.*ddq1.*t3+a3.*al2.*ddq1.*t29+a3.*al3.*ddq1.*t35+al2.*al3.*ddq1.*t3+al2.*al3.*ddq1.*t29];
mt3 = [-g.*(t7.*t24+t2.*(a2.*m2+al2.*t16))+ddq3.*t42-m2.*(t34+t37+t40+(t15.*t23.*t39)./2.0)+ddq1.*(t4.*(al2.*d3+a2.*d2.*m2+al2.*d2.*t16)+t7.*t8.*t25)+(t15.*(t43+t23.*(t12.*2.0+m2.*t39.*2.0)+al2.*t7.*t30.*4.0))./4.0+ddq2.*(t12+t32+t42+m2.*t38)-al2.*dq3.*t5.*t7.*(dq2+dq3)-al2.*dq2.*dq3.*t5.*t7,ddq3.*(t11+t13+t20+1.105820133333333e-1)+ddq2.*t42+(t15.*(t43+al2.*t5.*t7.*2.0+al2.*t7.*t30.*2.0))./4.0-g.*t7.*t24+ddq1.*t7.*t8.*t25+al2.*dq2.^2.*t5.*t7];
Y_t = reshape([mt1,mt2,mt3],3,3);