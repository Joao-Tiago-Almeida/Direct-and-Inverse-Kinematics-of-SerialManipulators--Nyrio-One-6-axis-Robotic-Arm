direct_transform(0, 0, -pi/2, 0, 0, 0)


function output = direct_transform(A1, A2, A3, A4, A5, A6)

    %% physical limits
    
    range_rotation = [-175, 175
                      -36.7, 90
                      -80, 90
                      -175, 175
                      -110, 100
                      -147.5, 147.5]*pi/180;
                    
    config = homeConfiguration(robot)
    
        % bound to max angle rotation
        
    A1 = bound_angle(A1, range_rotation(1,1), range_rotation(1,2));
    A2 = bound_angle(A2, range_rotation(2,1), range_rotation(2,2));
    A3 = bound_angle(A3, range_rotation(3,1), range_rotation(3,2));
    A4 = bound_angle(A4, range_rotation(4,1), range_rotation(4,2));
    A5 = bound_angle(A5, range_rotation(5,1), range_rotation(5,2));
    A6 = bound_angle(A6, range_rotation(6,1), range_rotation(6,2));
    
    %%

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
     
            %Next one we might have to make it symmetric because the y(rotation) axis is towards the right 
     F3_to_F4 = [1,	0,          0,          4.15;
                 0, cos(A4),    -sin(A4),   0;
                 0, sin(A4),    cos(A4),    3;
                 0, 0,          0,          1
                ];
            
     F4_to_F5 = [cos(A5),   0,  sin(A5),    18;
                 0,         1,  0,          0;
                 -sin(A5),  0,  cos(A5),    0;
                 0,         0,  0,          1
                ];
      F5_to_F6 = [1,    0,          0       ,   2.37;
                  0,    cos(A6),    -sin(A6),   0;
                  0,    sin(A6),    cos(A6),    -0.55;
                  0,    0,          0,          1      
                ];
      
      Total_T_matrix = F0_to_F1 * F1_to_F2 * F2_to_F3 * F3_to_F4 * F4_to_F5 * F5_to_F6;
            
      coords = Total_T_matrix*[0, 0, 0, 1]';
      
      orientation = get_orientation(Total_T_matrix);
      
      output = [coords(1:3);orientation']
end

function out = get_orientation(matrix)

    beta = atan2(sqrt(matrix(1,3)^2 + matrix(2, 3)^2), matrix(3,3));
    alpha = 0;
    gama = 0;
    
    if beta == pi/2
        gama = atan2(matrix(2,1), matrix(1,1));
        
    elseif beta == -pi/2
        gama = atan2(matrix(2,1), matrix(1,1));
        
    else
        alpha = atan2(matrix(2,3)/sin(beta), matrix(1,3)/sin(beta));
        gama = atan2(matrix(3,2)/sin(beta), -matrix(1,3)/sin(beta));
    end
        
    out = [alpha beta gama];
end

function value = bound_angle(x, m, M)
    
    if x>M
        value = M;
    
    elseif x<m
        value = m;
    end
end