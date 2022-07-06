function [ Tsc ] = DH_matrix( DH_param )
    %Returns the transformation matrix between 2 joints 
    % DH_param = (a, alfa, d, theta) (it's a row)
    a = DH_param(1);
    alfa = DH_param(2);
    d = DH_param(3);
    theta = DH_param(4);
    Tsc(1,1) = cos(theta);
    Tsc(1,2) = -sin(theta) * cos(alfa);
    Tsc(1,3) = sin(theta) * sin(alfa);
    Tsc(1,4) = a * cos(theta);
    Tsc(2,1) = sin(theta);
    Tsc(2,2) = cos(theta) * cos(alfa);
    Tsc(2,3) = -cos(theta) * sin(alfa);
    Tsc(2,4) = a * sin(theta);
    Tsc(3,1) = 0;
    Tsc(3,2) = sin(alfa);
    Tsc(3,3) = cos(alfa);
    Tsc(3,4) = d;
    Tsc(4,1) = 0;
    Tsc(4,2) = 0;
    Tsc(4,3) = 0;
    Tsc(4,4) = 1;

end

