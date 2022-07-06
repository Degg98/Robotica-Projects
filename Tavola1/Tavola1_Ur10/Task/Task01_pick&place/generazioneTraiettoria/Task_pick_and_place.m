%generazione variabili di giunto per task di pick and place

%cordinate x, y e z che il robot voglio che raggiunga
pfin = [0.5, 0.8, 0.1];

%conversione punto in cordinate omogenee
Tfin = transl(pfin);
%modifico la parte di rotazione 
Tfin(1:3,1:3) = [0 0 1;
                 0 -1 0;
                 1 0 0];
             
 %cinematica inversa per ricavare le variabili di giunto desiderate
 %attraverso la funzione ikine.m del Robotic Toolbox
 q_des_fin = ur10.ikine(Tfin);
 
 
 t=0:.1:15;
 traj_des = jtraj(q0, q_des_fin, t);
 pos_des = ur10.fkine(traj_des);
 pos = zeros(3, length(pos_des));
 for i = 1:length(pos_des)
     pos(:,i) = pos_des(i).t;
 end
 pos = pos';
 figure
 plot3(pos(:,1), pos(:,2), pos(:,3))
 
ur10.plot(traj_des)
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
ur10.plot(q0);


 
 