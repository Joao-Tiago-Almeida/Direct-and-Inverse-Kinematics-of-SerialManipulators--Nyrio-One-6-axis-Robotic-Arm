function output = direct_kinematics(A, constraints)

    if nargin < 1
        kinematics
        return
    elseif nargin < 2
        constraints = false;
    end
  
    if(constraints) % physical limits
        range_rotation = [-175, 175
                          -36.7, 90
                          -80, 90
                          -175, 175
                          -110, 100
                          -147.5, 147.5]*pi/180;    

        %   bound to max angle rotation
        A1 = bound_angle(A(1), range_rotation(1,1), range_rotation(1,2));
        A2 = bound_angle(A(2), range_rotation(2,1), range_rotation(2,2));
        A3 = bound_angle(A(3), range_rotation(3,1), range_rotation(3,2));
        A4 = bound_angle(A(4), range_rotation(4,1), range_rotation(4,2));
        A5 = bound_angle(A(5), range_rotation(5,1), range_rotation(5,2));
        A6 = bound_angle(A(6), range_rotation(6,1), range_rotation(6,2));
        
    else    % without physical limitations
        A1 = A(1);  A2 = A(2);  A3 = A(3);  A4 = A(4);  A5 = A(5);  A6 = A(6);
    end
    
    F1_to_F0 = [cos(A1),    -sin(A1),  0,   0;
                sin(A1),    cos(A1),   0,   0;
                0,          0,         1,   103;
                0,          0,         0,   1
               ];
           
     F2_to_F1 = [cos(A2),   0,  sin(A2),    0;
                 0,         1,  0,          0;
                 -sin(A2),  0,	cos(A2),    80;
                 0,         0,	0,          1
                ];
            
     F3_to_F2 = [cos(A3),   0   sin(A3),	0;
                 0,         1,  0,          0;
                 -sin(A3),  0,  cos(A3),    210;
                 0,         0,  0,          1
                ];
     
     F4_to_F3 = [1,	0,          0,          41.5;
                 0, cos(A4),    -sin(A4),   0;
                 0, sin(A4),    cos(A4),    30;
                 0, 0,          0,          1
                ];
            
     F5_to_F4 = [cos(A5),   0,  sin(A5),    180;
                 0,         1,  0,          0;
                 -sin(A5),  0,  cos(A5),    0;
                 0,         0,  0,          1
                ];
            
      F6_to_F5 = [1,    0,          0       ,   23.7;
                  0,    cos(A6),    -sin(A6),   0;
                  0,    sin(A6),    cos(A6),    -5.5;
                  0,    0,          0,          1      
                ];
      
      Total_T_matrix = F1_to_F0 * F2_to_F1 * F3_to_F2 * F4_to_F3 * F5_to_F4 * F6_to_F5;
            
      coords = Total_T_matrix*[0, 0, 0, 1]';
      
      orientation = get_orientation(Total_T_matrix);
      
      output = [coords(1:3);orientation'];
end

% Computes de Euler Angles in Z-Y-Z convenction
function out = get_orientation(matrix)
    
    alpha = 0;                                                          % Z
    beta = atan2(sqrt(matrix(1,3)^2 + matrix(2,3)^2), matrix(3,3));     % Y
    gamma = 0;                                                           % Z
    

    if round(beta,3) == round(pi/2,3)
        gamma = atan2(matrix(2,1), matrix(1,1));

    elseif round(beta,3) == round(-pi/2,3)
        gamma = -atan2(matrix(2,1), matrix(1,1));

    elseif round(beta,3) == 0
        gamma = 0;
        alpha = cos(beta)*atan2(matrix(2,1), matrix(1,1)); % cos(beta) -> ±1
    else
        alpha = atan2(matrix(2,3)/sin(beta), matrix(1,3)/sin(beta));
        gamma = atan2(matrix(3,2)/sin(beta), -matrix(3,1)/sin(beta));
    end

    matrix_test = rotz(alpha*180/pi)*roty(beta*180/pi)*rotz(gamma*180/pi);

      
    out = [alpha beta gamma];
end

% Limits the joint angles to the constraints (maximum values)
function value = bound_angle(x, m, M)

    x = round(x,4);
    m = round(m,4);
    M = round(M,4);

    value = x .* (x>=m) + m .* (x<m);
    value = value .* (value<=M) + M .* (value>M);
    
end
