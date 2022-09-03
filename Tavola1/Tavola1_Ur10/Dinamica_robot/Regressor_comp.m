syms q1 q2 q3 q4 q5 q6 real

%link length [m]
l1 = 0.1273;
l2 = -0.6120;
l3 = -0.5723;
l4 = 0.1639;
l5 = 0.1157;
l6 = 0.0922;

%link mass [Kg]
m1 = 7.10;
m2 = 12.70;
m3 = 4.27;
m4 = 2.00;
m5 = 2.00;
m6 = 0.365;

%link radius [m]
r = 0.02;

m = [m1 m2 m3 m4 m5 m6]';

% Links inertia matrix 
I = [L(1).I L(2).I L(3).I L(4).I L(5).I L(6).I]';

% Gravity matrix
g = [0; 0; 9.81];

% Links center of mass
R = [L(1).r' L(2).r' L(3).r' L(4).r' L(5).r' L(6).r'];
%--------------------------------------------------------------------
%Denavith_Hartenberg matrix
d_h = [0, pi/2, l1, q1;
       l2, 0, 0, q2;
       l3, 0, 0, q3;
       0, pi/2, l4, q4;
       0, -pi/2, l5, q5;
       0, 0, l6, q6];

%--------------------------------------------------------------------
%Regressor computation
Ts0 = eye(4);
Tee = eye(4);

c = R;

n = size(d_h,1);

q = sym('q',[n,1]);
qd = sym('qd',[n,1]);
qdd = sym('qdd',[n,1]);

[Y_, pi_param] = Regressor_fcn(d_h,Ts0,Tee,I,m,c,g,q,qd,qdd);

Y = sym(zeros(size(Y_)));
for i = 1:21
    disp(i)
    Y(:,i) = simplify(Y_(:,i));
end


% Test di correttezza
tau_reg = simplify(Y * pi_param);
tau = subs(B*qdd + C*qd + G);
res = subs(tau - tau_reg)

% fid = fopen('Mymatrix.txt','wt');
% for ii = 1:size(Y,1)
%     fprintf(fid,'%s\n', char(Y(ii,:)));
%     fprintf(fid,'\n');
% end
% fclose(fid);
