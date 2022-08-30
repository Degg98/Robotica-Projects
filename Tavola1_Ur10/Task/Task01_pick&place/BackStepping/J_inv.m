function J_inv = Jacobian_matrix(param, q)
%#codegen

l1 = param(1);
l2 = param(2);
l3 = param(3);
l4 = param(4);
l5 = param(5);
l6 = param(6);
m1 = param(7);
m2 = param(8);
m3 = param(9);
m4 = param(10);
m5 = param(11);
m6 = param(12);
r = param(13);

q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
q5 = q(5);
q6 = q(6);

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    07-Jul-2022 13:47:55

t2 = cos(q1);
t3 = cos(q2);
t4 = cos(q3);
t5 = cos(q4);
t6 = cos(q5);
t7 = sin(q1);
t8 = sin(q2);
t9 = sin(q3);
t10 = sin(q4);
t11 = sin(q5);
t12 = l4.*t2;
t13 = l2.*t8;
t14 = l4.*t7;
t15 = t3.*t4;
t16 = t2.*t6;
t17 = t3.*t9;
t18 = t4.*t8;
t19 = t6.*t7;
t20 = t8.*t9;
t21 = -t2;
t22 = l2.*t2.*t3;
t23 = l2.*t3.*t7;
t24 = l3.*t17;
t25 = l3.*t18;
t26 = t7.*t20;
t27 = -t20;
t28 = t2.*t15;
t29 = t2.*t17;
t30 = t2.*t18;
t31 = t7.*t15;
t32 = t2.*t20;
t33 = t7.*t17;
t34 = t7.*t18;
t42 = t20.*t21;
t45 = t17+t18;
t35 = l3.*t28;
t36 = l3.*t31;
t37 = l3.*t32;
t38 = l3.*t26;
t39 = -t24;
t40 = -t25;
t41 = -t31;
t44 = l3.*t42;
t46 = t15+t27;
t47 = t5.*t45;
t48 = t10.*t45;
t49 = t29+t30;
t50 = t33+t34;
t54 = t28+t42;
t43 = -t36;
t51 = t5.*t46;
t52 = t10.*t46;
t53 = -t48;
t55 = t26+t41;
t56 = t5.*t49;
t57 = t10.*t49;
t58 = t5.*t50;
t59 = t10.*t50;
t60 = t5.*t54;
t61 = t10.*t54;
t62 = -t57;
t63 = t5.*t55;
t64 = t10.*t55;
t66 = t47+t52;
t67 = t51+t53;
t69 = -l5.*(t48-t51);
t71 = l5.*(t48-t51);
t72 = t56+t61;
t79 = -t11.*(t57-t60);
t82 = t11.*(t57-t60);
t65 = -t64;
t68 = l6.*t11.*t66;
t73 = t59+t63;
t74 = l5.*t72;
t75 = t60+t62;
t84 = t19+t82;
t70 = -t68;
t76 = t58+t65;
t77 = t11.*t73;
t86 = l6.*t84;
t88 = t68+t69;
t78 = l5.*t76;
t80 = -t77;
t89 = t39+t40+t88;
t90 = t13+t24+t25+t70+t71;
t91 = t74+t86;
t81 = -t78;
t83 = t16+t80;
t93 = t14+t22+t35+t44+t91;
t85 = l6.*t83;
t87 = -t85;
t92 = t78+t87;
J = reshape([t12-t23+t38+t43+t81+t85,t93,0.0,0.0,0.0,1.0,t21.*t90,-t7.*t90,t2.*t93-t7.*(t12-t23+t38+t43+t81+t85),t7,t21,0.0,t21.*(t24+t25+t70+t71),-t7.*(t24+t25+t70+t71),t2.*(t14+t35+t44+t91)-t7.*(t12+t38+t43+t81+t85),t7,t21,0.0,-t21.*t88,t7.*t88,t2.*(t14+t91)-t7.*(t12+t81+t85),t7,t21,0.0,-t76.*t88-t92.*(t48-t51),t72.*t88+t91.*(t48-t51),t72.*t92-t76.*t91,t72,t76,t48-t51,0.0,0.0,0.0,t84,-t16+t77,-t11.*t66],[6,6]);
J_inv = inv(J)