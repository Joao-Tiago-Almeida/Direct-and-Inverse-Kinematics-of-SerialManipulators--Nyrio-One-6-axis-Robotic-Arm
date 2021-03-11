%pi/2 = 1.57079627
%pi =   3.14159265

% input = [3.14159265, -1.570796327, -1.570796327, 0, 0, 0];
input = [0, pi/4, 0, 0, 0, 0];

% initialPoint = [0, -0.2215, 0.423, 1]';
initialPoint = [0 0 103]';

%apply_transforms(input, initialPoint)




%function end_effector = apply_transforms(input, initialPoint)

    T01 = [ [eye(3); zeros(1,3)], [initialPoint; 1] ];
    
    angle = input(1);
    R1 = [cos(angle)	-sin(angle)	0
           sin(angle)	cos(angle)	0
           0            0           1];
    P1 = [0 0 80]';
       
    T12 = [ [R1; zeros(1,3)], [R1*P1; 1] ];
    
    angle = input(2);
    R2 = [cos(angle)	0           sin(angle)
           0            1           0         
           -sin(angle)	0           cos(angle)];
    P2 = [0 0 210]';
    
    T23 = [ [R2; zeros(1,3)], [R2*P2; 1] ];
       
    angle = input(3);
    R3 = [cos(angle)	0           sin(angle)
           0            1           0         
           -sin(angle)	0           cos(angle)];
       
    angle = input(4);
    R4 = [1    0           0           
           0    cos(angle)  -sin(angle)
           0    sin(angle)  cos(angle)];
       
     angle = input(5);
    R5 = [cos(angle)	0           sin(angle)
           0            1           0         
           -sin(angle)	0           cos(angle)];

    angle = input(6);
    R6 = [1    0            0          
           0    cos(angle)  -sin(angle)
           0    sin(angle)  cos(angle)];
           

    %end_effector = T67*T56*T45*T34*T23*T12*initialPoint
%end

