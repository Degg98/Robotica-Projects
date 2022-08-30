function [ RDH, CGJacob ] = CGJacobBaseDyn( DHtable,Tbo,Tee,c,k )

        
        n = size(DHtable,1);

        if k<n
            Tee = eye(4);
        end
        
        DHtable = DHtable(1:k,:);
        Tf = (DH_Kynematics(Tbo, eye(4) ,DHtable));
        RDH = (Tf(1:3,1:3));
		pkck = c;
        x = (RDH*pkck);
        X_hat = [0 -x(3) x(2);x(3) 0 -x(1);-x(2) x(1) 0];
		M = [ eye(3),     -X_hat; zeros(3,3),  eye(3)];
		DHJacob = Geometric_Jacobian(Tbo, Tee, DHtable, k);
        if k<n
            DHJacob = [DHJacob,zeros(6,n-k)];
        end
		CGJacob = M*DHJacob;
			           

end

