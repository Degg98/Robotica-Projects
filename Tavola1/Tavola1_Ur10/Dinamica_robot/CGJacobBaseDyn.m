function [ RDH, CGJacob ] = CGJacobBaseDyn( DHtable,Tbo,Tee,c,k )

        
        n = size(DHtable,1);

        if k<n
            Tee = eye(4);
        end
        
        % Estraggo le prime k righe dalla DH table
        DHtable = DHtable(1:k,:);
        % Costruisco la trasformazione dal frame base al k-esimo frame
        Tf = (DH_Kynematics(Tbo, eye(4) ,DHtable));
        % Estraggo la matrice di rotazione
        RDH = (Tf(1:3,1:3));
        % Centro di gravità del k-esimo link in terna k
		pkck = c;
        % Centro di gravità del k-esimo link in terna 0
        x = (RDH*pkck);
        % Matrice antisimmetrica di x
        X_hat = [0 -x(3) x(2);x(3) 0 -x(1);-x(2) x(1) 0];
		M = [ eye(3),     -X_hat; zeros(3,3),  eye(3)];
        % Jacobiano geometrico di dimensioni nxk
		DHJacob = Geometric_Jacobian(Tbo, Tee, DHtable, k);
        if k<n
            % Riempie J di 0 per avere un matrice quadrata
            DHJacob = [DHJacob,zeros(6,n-k)];
        end
        % Jacobiano geometrico rispetto ai centri di massa dei link
		CGJacob = M*DHJacob;
			           

end

