function angles = inverse_transform(O)

    if nargin < 1
        kinematics
        return
    end
    
    [P0_5, T0_6] = get_joint_position_five(O);
    joint123 = get_planar_geometry(P0_5(1), P0_5(2), P0_5(3)-10.3-8); % TODO tirar as colunas com 5
     
    T0_3 = compute_T0_3(joint123(:,1)');
    T3_6 = T0_3\T0_6;
    joint456 = get_joint456(T3_6);
    
    angles = zeros(6, size(joint123,2)*size(joint456,2));   % each column is a different option
    angles(1:3,:) = repmat(joint123, 1, size(joint456,2));  % combinations of the joints 123
    angles(4:6,:) = repmat(joint456, 1, size(joint123,2));  % combinations of the joints 456
   
end

%% compute fifth joint position
function [P0_5, T0_6] = get_joint_position_five(O)
    
    x = O(1); y = O(2); z = O(3);
  
    P0_6 = [ x y z ]';
    
    ca = cos(O(4)); sa = sin(O(4));
    cb = cos(O(5)); sb = sin(O(5));
    cg = cos(O(6)); sg = sin(O(6));

    R0_6 = [    ca*cb*cg-sa*sg,   -ca*cb*sg-sa*cg,    ca*sb
                sa*cb*cg+ca*sg,   -sa*cb*sg+ca*cg,    sa*sb
                -sb*cg,           sg*sb,              cb    ];
    
    P5_6 = [2.37 0 0]';
    
    joint6_init = 0;
    R5_6 = [1,  0,                  0
            0,  cos(joint6_init),   -sin(joint6_init)  
            0,  sin(joint6_init),   cos(joint6_init)];
        
    P0_5 = P0_6 - (R0_6/R5_6)*P5_6;
    
    T0_6 = [[R0_6; zeros(1,3)], [P0_6; 1] ];
end

%% draw T0_3 matrix
function T0_3 = compute_T0_3(joint123)
    
    A1 = joint123(1);   A2 = joint123(2);   A3 = joint123(3);
    
    F0_to_F1 = [cos(A1),    -sin(A1),  0,   0;
                sin(A1),    cos(A1),   0,   0;
                0,          0,         1,   10.3;
                0,          0,         0,   1
               ];
           
	F1_to_F2 = [cos(A2),   0,  sin(A2),    0;
                0,         1,  0,          0;
                -sin(A2),  0,	cos(A2),    8;
                0,         0,	0,          1
               ];
            
    F2_to_F3 = [cos(A3),   0   sin(A3),	0;
                0,         1,  0,          0;
                -sin(A3),  0,  cos(A3),    21;
                0,         0,  0,          1
               ];
           
    T0_3 = F0_to_F1 * F1_to_F2 * F2_to_F3;

end

%% compute 3DOF orientation joints
function angles = get_joint456(T3_6)

    joint5 = acos(T3_6(1,1));   joint5 = [-joint5, joint5];
    
    % in case there is only a rotation in x
    if T3_6(1,1) == 1
        ang = atan2(T3_6(3,2),T3_6(2,2));   % Rotx --->>> T3_6(3,2) == cos()  T3_6(2,2) == sin()
        if abs(ang) > 175*pi/180
            joint4 = sign*(175*pi/180);
            joint6 = sign(ang)*(abs(ang) - 175*pi/180);
            angles = [joint4; 0; joint6]
            
        else
            angles = [ang; 0; 0];
        end
        return
    else
    
        joint4_aux = asin( T3_6(2,1)./sin(joint5) );  joint4_aux = [joint4_aux; sign(joint4_aux)*pi-joint4_aux];
        joint4_flag = round(T3_6(3,1), 4) == round(-cos(joint4_aux).*sin(joint5), 4);
        joint4 = sum(joint4_aux.*joint4_flag,1);    % only one option per column

        joint6_aux = asin( T3_6(1,2)./sin(joint5) );  joint6_aux = [joint6_aux; sign(joint6_aux)*pi-joint6_aux];
        joint6_flag = round(T3_6(1,3), 4) == round(cos(joint6_aux).*sin(joint5), 4);
        joint6 = sum(joint6_aux.*joint6_flag,1);
    end
    
    if( sum(sum(joint4_flag)) ~= 2 || sum(sum(joint6_flag)) ~= 2 )
        disp("O Rosa tinha razão e o Almeida é um burro")
    end
    
    % bound between -pi and pi
    joint4 = round( bound_angle(joint4, -pi, pi), 4);
    joint5 = round( bound_angle(joint5, -pi, pi), 4);
    joint6 = round( bound_angle(joint6, -pi, pi), 4);
    
    % check if it is a possible position
    f = @(x,m,M) (x>=m) .* (x<=M);
    
    min4 = round(-175*pi/180, 4);       max4 = round(175*pi/180, 4);
    min5 = round(-110*pi/180, 4);       max5 = round(100*pi/180, 4);
    min6 = round(-147.5*pi/180, 4);     max6 = round(147.5*pi/180, 4);
    
    % individuals angles for each movement
    joint4_flag = f(joint4, min4, max4);
    joint5_flag = f(joint5, min5, max5);
    joint6_flag = f(joint6, min6, max6);
    
    % flag if movement is possible, both angles
    joint_flag = (joint4_flag+joint5_flag+joint6_flag) == 3;
   
    angles = [joint4; joint5; joint6];
    angles = angles(:,joint_flag);
 
end

%% compute the linear geometry
function angles = get_planar_geometry(x, y, z)

    % better notation
    xy = sqrt(x^2+y^2); xyz = sqrt(xy^2 + z^2);
    l2 = 21;
    l3 = sqrt(3^2 + (4.15+18)^2);
    
    beta = atan2(z, [-xy; xy]);    % angle between end effort and ground Q1
    
    phi = acos( (l2^2 + xyz^2 - l3^2 )/(2*l2*xyz) ); % angle between beta and length 1
    
    % coordinates in 2D
    theta3 = acos( (xy.^2 + z^2 - l2^2 -l3^2)/(2*l2*l3) ) ; theta3 = [-theta3 theta3];   
    theta2 = beta + phi*(2*(theta3 < 0)-1); theta2 = [theta2(1,:), theta2(2,:)];
    theta3 = [theta3 theta3];
   
%      round(xy,3) == round(l2*cos(theta2) + l3*cos(theta2+theta3), 3)
%      round(z,3) == round(l2*sin(theta2) + l3*sin(theta2+theta3), 3)

    % Note that the rotation angles are symetric from 2D to 3D
    joint2 = pi/2 - theta2;
    joint3 = -atan(22.15/3) - theta3;
    
    % bound between -pi and pi
    joint1 = [ atan2(-y,-x) atan2(-y,-x) atan2(y,x) atan2(y,x) ]; 
    joint2 = round( bound_angle(joint2, -pi, pi), 4);
    joint3 = round( bound_angle(joint3, -pi, pi), 4);
    
    % check if it is a possible position
    f = @(x,m,M) (x>=m) .* (x<=M);
    
    min1 = round(-175*pi/180, 4);   max1 = round(175*pi/180, 4);
    min2 = round(-36.7*pi/180, 4);  max2 = round(pi/2, 4);
    min3 = round(-80*pi/180, 4);    max3 = round(pi/2, 4);
    
    % individuals angles for each movement
    joint1_flag = f(joint1, min1, max1);
    joint2_flag = f(joint2, min2, max2);
    joint3_flag = f(joint3, min3, max3);
    
    % flag if movement is possible, both angles
    joint_flag = (joint1_flag+joint2_flag+joint3_flag) == 3;
   
    angles = [joint1; joint2; joint3];
    angles = angles(:,joint_flag);
end