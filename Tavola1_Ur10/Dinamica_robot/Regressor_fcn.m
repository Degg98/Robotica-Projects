function [Wreg,pi_param] = Regressor_fcn(DH_table,Ts0,Tee,I,m,c,g,q,qd,qdd)
% This script calculates the robot 's equations of motion and rewrites it in
% regressor form. The script can handle any serial configuration with
% revolute joints. Entries to be changed :
% - DH parameters ( line 48 -53)
% - mass - and inertia parameters if known ( line 143 -155)
% - gravitational vector ; depends on base coordinate frame ( line 193)
% - symbolic variable declarations ( line 26 -39)
% - locations of centers of mass ( line 109 -111)
% note that extra entries can be added if a robot of more links is
% considered.

n = size(DH_table,1);
% Generate matrices of homogenous transformations representing positions
% x^0 _i and orientations R^0 _i of the link coordinate frames oixiyizi (i=1 ,2 ,3)
% relative to the coordinate frame o0x0y0z0 of the base.
% -----------------------------------------------------------------------------
for i =1: n
    T_0_{i} =  DH_Kynematics( Ts0, eye(4), DH_table(1:i,:));
end

% Create linear velocity Jacobian Jv and angular velocity Jacobian Jomega
% -----------------------------------------------------------------------
for k = 1: n
    J_v{ k }=[]; J_omega{ k }=[];
    J= Geometric_Jacobian(Ts0,Tee,DH_table(1:k,:), k);
    if k<n
         J = simplify([J,zeros(6,n-k)]);
    end
    J_v{ k } = J(1:3,:);
    J_omega{ k } = J(4:6,:);
    
end

% Coordinates of the i-th link center of mass expressed relative to the
% i-th coordinate frame
% ---------------------------------------------------------------------
for k =1:n
    o_c_{k}=c(:,k);
end
% Origins o_0_ {i} (i=1 ,... ,n) of the link coordinate frames expressed
% relative to the coordinate frame of the base and coordinates o_0_c_ {i}
% of the i-th link center of mass expressed relative to the coordinate
% frame of the base
% ----------------------------------------------------------------------
for i =1: n
    [R0k,J_G] =  CGJacobBaseDyn( DH_table,Ts0,eye(4),o_c_{i},i );
    J_v_c_{i} = J_G(1:3,:);
    J_omega_c_{i} = J_G(4:6,:);
end
% Link inertial parameters
% ------------------------
j=1;
for i=1:n
    I_{i} = I(j:j+2,:);
    j = j+3;
end
% =========================================================================
% REGRESSION
% =========================================================================


% Display start regression and start timer
disp('Start regression ');
tic ;
% The regressor Yreg and the parameter vector pi_param are initially empty
Wreg = [];
pi_param = [];

% Express inertia tensor i in a 3 x3x6 tensor E and a vector J. These are
% the components of the tensor E
E_{1} = [1 0 0; 0 0 0; 0 0 0];
E_{2} = [0 1 0; 1 0 0; 0 0 0];
E_{3} = [0 0 1; 0 0 0; 1 0 0];
E_{4} = [0 0 0; 0 1 0; 0 0 0];
E_{5} = [0 0 0; 0 0 1; 0 1 0];
E_{6} = [0 0 0; 0 0 0; 0 0 1];
E = [ E_{1} E_{2} E_{3} E_{4} E_{5} E_{6}];

% start loop over each link
for i = 1: n
    % Rewrite the first part of the lagrange equation in X and pi
    % -----------------------------------------------------------
    
    % calculate X
    X_0_{ i } = (( J_v{ i }.'* J_v{ i })* qd );
    
    s1 = J_omega{ i }* qd ; S1 = [0 -s1(3) s1(2); s1(3) 0 -s1(1); -s1(2) s1(1) 0];
    s2 = J_v{ i }* qd ; S2 = [0 -s2(3) s2(2); s2(3) 0 -s2(1); -s2(2) s2(1) 0];
    X_1_{ i } = (( J_v{ i }.'* S1 - J_omega{ i }.'* S2 )* T_0_{ i }(1:3 ,1:3));
    
    for jj = 1:6
        X_2_{ i }(: , jj ) = J_omega{ i }.'* T_0_{ i }(1:3 ,1:3)* E_{ jj }* T_0_{ i }(1:3 ,1:3).' ...
            * J_omega{ i }* qd ;
    end
    
    % pi
    
    pi_0_{ i } = m( i );
    f =  o_c_{ i };
    pi_1_{ i } = [m( i )* f(1) m( i )* f(2) m( i )* f(3)].';
    
    iI_{ i } = +I_{ i } + m( i )*[0 -f(3) f(2); f(3) 0 -f(1); -f(2) f(1) 0].'*[0 -f(3) f(2); ...
        f(3) 0 -f(1); -f(2) f(1) 0];
    Ji = [ iI_{ i }(1 ,1) iI_{ i }(1 ,2) iI_{ i }(1 ,3) iI_{ i }(2 ,2) iI_{ i }(2 ,3) iI_{ i }(3 ,3)];
    pi_2_{ i } = Ji.';
    
    
    
    % Rewrite the second part of the lagrange equation in Y and pi
    % (pi already determined above )% ------------------------------------------------------------
    % calculate Y
    Y_0_{ i } = [];
    for jj = 1: n
        if q( jj ) ~= pi /2
            line_{ jj } = 0.5 * qd.'*( diff(( J_v{ i }.'* J_v{ i }) , q( jj )))* qd ;
        else
            line_{jj} = 0;
        end
        
        Y_0_{ i } = [ Y_0_{ i }; line_{ jj }];
    end
    clear line_
    
    Y_1_{ i } = [];
    for jj = 1: n
        if q( jj ) ~= pi /2
            
            line_{ jj } = 0.5 *( diff(( T_0_{ i }(1:3 ,1:3).'* S1.'*J_v{ i }* qd ...
                - T_0_{ i }(1:3 ,1:3).'* S2.'* J_omega{ i }* qd ),q( jj ))).';
        else
            line_{jj} = 0;
        end
        
        Y_1_{ i } = [ Y_1_{ i }; line_{ jj }];
    end
    clear line_
    
    Y_2_ { i } = [];
    for jj = 1: n
        for k = 1:6
            if q( jj ) ~= pi /2
                line_{ jj }(: , k ) = 0.5 * qd.'*( diff(( J_omega{ i }.'* T_0_{ i }(1:3 ,1:3) ...
                    * E_{ k }* T_0_{ i }(1:3 ,1:3).'* J_omega{ i }) , q( jj )))* qd ;
            else
                line_{jj} = 0;
            end
            
        end
        Y_2_{ i } = [ Y_2_{ i }; line_{ jj }];
    end
    clear line_
    
    
    % Rewrite the third part of the lagrange equation in Z
    % ----------------------------------------------------
    
    % calculate Z
    Z_0_{ i } = -J_v{ i }.'*g ; % note that the minus sign is dropped , since
    % here the vector g is opposite to the true
    % gravitational vector
    
    Z_1_{ i } = [];
    for jj = 1: n
        line_{ jj } = diff(( T_0_{ i }(1:3 ,1:3).'* g ), q( jj ));
        Z_1_{ i } = [ Z_1_{ i }; line_{ jj }.'];
        
    end
    clear line_
    
    
    % Calculate Xdot from X
    % ---------------------
    
    X_0_d_{ i } = 0;
    for jj = 1: n
        if q( jj ) ~= pi /2 && qd( jj ) ~= 0
            X_0_d_{ i } = X_0_d_{ i } + diff( X_0_{ i },q( jj ))* qd( jj ) ...
                + diff( X_0_{ i }, qd( jj ))* qdd( jj );
        else
            X_0_d_{ i } = zeros(6,1);
        end
    end
    X_1_d_{ i } = 0;
    for jj = 1: n
        if q( jj ) ~= pi /2 && qd( jj ) ~= 0
            
            X_1_d_{ i } = X_1_d_{ i } + diff( X_1_{ i },q( jj ))* qd( jj ) ...
                + diff( X_1_{ i }, qd( jj ))* qdd( jj );
        else
            X_1_d_{ i } = zeros(n,3);
        end
    end
    X_2_d_{ i } = 0;
    for jj = 1: n
        if q( jj ) ~= pi /2 && qd( jj ) ~= 0
            
            X_2_d_{ i } = X_2_d_{ i } + diff( X_2_{ i },q( jj ))* qd( jj ) ...
                + diff( X_2_{ i }, qd( jj ))* qdd( jj );
        else
            X_2_d_{ i } = zeros(n,6);
        end
    end
    
    % Determine the regressor blocks
    % ------------------------------
    
    W_0_{ i } = X_0_d_{ i } - Y_0_{ i } - Z_0_{ i };
    W_1_{ i } = X_1_d_{ i } - Y_1_{ i } + Z_1_{ i };
    W_2_{ i } = X_2_d_{ i } - Y_2_{ i };
    
    % Determine the regressor and parameter vector for link i
    % -------------------------------------------------------
    
    W_{ i } = [ W_0_{ i } W_1_{ i } W_2_{ i }];
    pi_v_{ i } = [ pi_0_{ i }; pi_1_{ i }; pi_2_{ i }];
    % Fill total regressor and parameter vector
    % -----------------------------------------
    
    Wreg = [ Wreg W_{ i }];
    pi_param = [ pi_param ; pi_v_{ i }];
    
    % Display that one loop is done
    % -----------------------------
    fprintf('loop %i of %i is done \n',i , n);
    
    % end loop
end


% Remove columns that result in zero entries
disp ('removing columns from regressor that will result in zero and clean up parameter vector ');
index = 0;
for i = 1:size(Wreg,2)
    a = symvar(Wreg(:,i));
    if isempty(a)
        index = [index,i];
    end
end

index = index(2:end);
n = size(index,2);
for i=1:n
    
    Wreg(:,index(end)) = [];
    pi_param(index(end)) = [];
index(end) = [];
end
index = 0;
for i = 1:size(pi_param,1)
    a = isAlways(pi_param(i)<10^-6,'Unknown','false');
    if a
        index = [index,i];
    end
end

index = index(2:end);
n = size(index,2);
for i=1:n
    
    Wreg(:,index(end)) = [];
    pi_param(index(end)) = [];
index(end) = [];
end

% Display complete and stop timer
disp('Regression complete , type " Y " and " pi_param " to displays regressor and parameter vector ');
toc ;
end