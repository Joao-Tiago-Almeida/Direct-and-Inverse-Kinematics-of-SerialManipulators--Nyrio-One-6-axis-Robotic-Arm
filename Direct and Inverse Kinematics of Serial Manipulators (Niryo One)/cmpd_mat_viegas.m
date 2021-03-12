direct_transform(0, 0, -pi/2, 0, 0, 0)


function output = direct_transform(A1, A2, A3, A4, A5, A6)
    F0_to_F1 = [cos(A1), -sin(A1), 0, 0;
                sin(A1), cos(A1), 0, 0;
                0, 0, 1, 0.103;
                0, 0, 0, 1
               ];
           
     F1_to_F2 = [1, 0, 0, 0;
                 0, cos(A2), -sin(A2), 0;
                 0, sin(A2), cos(A2), 0.08;
                 0, 0, 0, 1
                ];
            
     F2_to_F3 = [1, 0, 0, 0;
                 0, cos(A3), -sin(A3), 0;
                 0, sin(A3), cos(A3), 0.210;
                 0, 0, 0, 1
                ];
     
            %Next one we might have to make it symmetric because the y(rotation) axis is towards the right 
     F3_to_F4 = [cos(A4), 0, sin(A4), 0;
                 0, 1, 0, -0.0415;
                 -sin(A4), 0, cos(A4), 0.030;
                 0, 0, 0, 1
                ];
            
     F4_to_F5 = [1, 0, 0, 0;
                 0, cos(A5), -sin(A5), -0.180;
                 0, sin(A5), cos(A5), 0;
                 0, 0, 0, 1
                ];
      F5_to_F6 = [cos(A6), 0, sin(A6), 0;
                 0, 1, 0, -0.0237;
                 -sin(A6), 0, cos(A6), -0.0055;
                 0, 0, 0, 1
                ];
      
      Total_T_matrix = F0_to_F1 * F1_to_F2 * F2_to_F3 * F3_to_F4 * F4_to_F5 * F5_to_F6;
            
      coords = Total_T_matrix*[0, 0, 0, 1]';
      
      orientation = get_orientation(Total_T_matrix);
      
      output = cat(1, coords, orientation')
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
        
    out = [alpha; beta; gama]';
end
