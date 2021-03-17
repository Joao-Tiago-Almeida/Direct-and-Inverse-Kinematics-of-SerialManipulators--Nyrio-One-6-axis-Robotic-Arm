function joint123 = inverse_transform(O)

    if nargin < 1
        kinematics
        return
    end
    
    ca = cos(O(4)); sa = sin(O(4));
    cb = cos(O(5)); sb = sin(O(5));
    cg = cos(O(6)); sg = sin(O(6));
    x = O(1); y = O(2); z = O(3);
    
    R0_6 = [    ca*cb*cg-sa*sg,   -ca*cb*sg-sa*cg,    ca*sb
                sa*cb*cg+ca*sg,   -sa*cb*sg+ca*cg,    sa*sb
                -sb*cg,           sg*sb,              cb    ];
            
    P0_6 = [ x y z ]';
    
    T0_6 = [[R0_6; zeros(1,3)], [P0_6; 1] ];
    
    P5_6 = [2.37 0 0]';
    
    joint6_init = 0;
    R5_6 = [1,  0,                  0
            0,  cos(joint6_init),   -sin(joint6_init)  
            0,  sin(joint6_init),   cos(joint6_init)];
        
    P0_5 = P0_6 - (R0_6/R5_6)*P5_6
    
    % under construction
    [joint23, backforward] = get_planar_geometry(P0_5(1), P0_5(2), P0_5(3)-10.3-8); % TODO tirar as colunas com 5
    joint1 = atan2(y,x) - pi*backforward;
    joint123 = [joint1   joint23(:,1)' 0 0 0]
    
end

function [angles, backforward] = get_planar_geometry(x, y, z)

    % better notation
    xy = sqrt(x^2+y^2); xyz = sqrt(xy^2 + z^2);
    l2 = 21;
    l3 = sqrt(3^2 + (4.15+18)^2);
    
    sig = sign(x);%sign(x-0.89188); % TODO
    backforward = ~logical((sig+1)/2); % False is pointing to front and True to back
        
    beta = atan2(z, sig*xy);    % angle between end effort and ground Q1
    
    phi = acos( (l2^2 + xyz^2 - l3^2 )/(2*l2*xyz) ); % angle between beta and length 1
    
    % coordinates in 2D
    theta3 = acos( (xy.^2 + z^2 - l2^2 -l3^2)/(2*l2*l3) ) ; theta3 = [-theta3 theta3];   
    theta2 = beta + phi*(2*(theta3 < 0)-1);


%     round(xy,3) == round(l2*cos(theta2) + l3*cos(theta2+theta3), 3)
%     round(xy,3) == round(l2*cos(theta2) + l3*cos(theta2+theta3), 3)

    % Note that the rotation angles are symetric from 2D to 3D
    joint2 = pi/2 - theta2;
    joint3 = -atan(22.15/3) - theta3;
    
    % bound between -pi and pi
    joint2 = round( bound_angle(joint2, -pi, pi), 4);
    joint3 = round( bound_angle(joint3, -pi, pi), 4);
    
    % check if it is a possible position
    f = @(x,m,M) (x>=m) .* (x<=M);
    
    min2 = round(-36.7*pi/180, 4); max2 = round(pi/2, 4);
    min3 = round(-80*pi/180, 4);   max3 = round(pi/2, 4);
    
    % individuals angles for each movement
    joint2_flag = f(joint2, min2, max2);
    joint3_flag = f(joint3, min3, max3);
    
    % flag if movement is possible, both angles
    joint_flag = (joint2_flag+joint3_flag) == 2;
   
    angles = [joint2; joint3]*diag(joint_flag) + ~joint_flag*5; % it is bounded to ±pi
   
    
end

