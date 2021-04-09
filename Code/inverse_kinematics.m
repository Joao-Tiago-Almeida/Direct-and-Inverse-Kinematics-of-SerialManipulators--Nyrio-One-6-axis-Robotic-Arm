function best_solution_sorted = inverse_kinematics(O)

    if nargin < 1
        kinematics
        return
    end
    
    % reshape to column vector
    O = reshape(O, 6, 1);
    
    threshold = pi/36; % rad  
    trials = -pi:threshold:pi-threshold;    % range angles

    error_vec = zeros(1,0);
    best_solution = zeros(6,0);
    weight_solution = zeros(1,0);

    count = 0;
    f = figure;hold on
    
    % The beginning of the brute force method, each iteration is for a
    % different value of theta6
    for i = trials
        try
            
            [P0_5, T0_6] = get_joint_position_five(O, i);   % new guess for joint6 angle

            % compute the first 3 joints angles(each column is a solution with
            % the 1st line for theta1, 2nd for theta 2 and the 3rd for theta3 )
            % [3x4] matrix
            joint123 = get_planar_geometry(P0_5(1), P0_5(2), P0_5(3)-103-80);

            % compute T3_6 matrix [4x4x4] matrix
            T0_3 = compute_T0_3(joint123);
            T3_6 = compute_T3_6(T0_3, T0_6);

            % get all the possible combinations of angles(8 solutions) [6x8]
            % matrix
            joints6dof = compute_joints456(T3_6, joint123);  % get available solutions ( 8 preferable )

            % discard the imaginary solutions
            joints6dof = get_real_movements(joints6dof, false);

            plot([count count], [0 pi], 'r');
            % calculate the error of the computed solutions
        catch
            disp("Error was found. Possible a lost imaginary solution. :(")
            close all
            best_solution_sorted = zeros(6,0);
            return
        end
        for j = 1:size(joints6dof,2)
            
            error = abs(angdiff(i,joints6dof(6,j)));
            
            dir_kin = direct_kinematics(joints6dof(:,j));
            dir_kin_diff = vecnorm(O(1:3)-dir_kin(1:3));
            
            % store the best values, error in theta6 under threshold,
            % position error under 2mm
            % the round function is needed since 'pi' is not finite and may
            % presents flutuation errors
            if round(error-threshold, 3) <= 0 && dir_kin_diff < 1
                error_vec = [error_vec error];
                best_solution = [best_solution joints6dof(:,j)];
                
                % compute the best solution
                weight_solution = [weight_solution dir_kin_diff];
            end
            
            % bonus point research
            error_vec = [error_vec error];
            plot(count, error, '*b');
            count = count+1;
        end
    end
    
    plot([0 count-1], [threshold threshold], 'k', "LineWidth", 2);
    xlim([0 count-1]);
    ylim([0 pi]);
    xticks([0 count/4 count/2 3*count/4 count-1]);
    xticklabels({'-\pi','-\pi/2', '0','\pi/2','\pi'})
    yticks([0 threshold pi]);
    yticklabels({'-\pi','\pi/18','\pi'})
    xlabel("\Theta_6 ", "FontSize", 20)
    ylabel("error [rad]", "FontSize", 20)
    legend(["\Theta_6", "angdiff(\Theta_6, \theta_6)"], "FontSize", 16, "Location", "Best")
    f.CurrentAxes.FontSize = 16;
    t = title("Diference between computed \theta_6 and tested value (\Theta_6)", "FontSize", 16);
    
    [error_vec_sorted, order] = sort(weight_solution);  % sort by closest xyz
    best_solution_sorted = best_solution(:,order)';
    error_vec_sorted = error_vec_sorted'; % column vector
    has_solutions = logical(size(best_solution_sorted,1));
    
    if ~has_solutions
    disp("There are no available solutions!");
    best_solution_sorted = zeros(0,6);
    return
    end

    disp("Found at least " + num2str(size(best_solution_sorted,1)) + " solutions.");
    disp("Best inverse kinematic solution with " + num2str(error_vec_sorted(1)) + " norm error.");

    disp("Best solution :");
    disp(best_solution_sorted(1,:));
    
    % bonus point
    n = my_Kmeans(best_solution_sorted);
    end

%% computes fifth joint position matrix and the end-effector coordinates in the reference 
function [P0_5, T0_6] = get_joint_position_five(O, joint6_init)
    
    x = O(1); y = O(2); z = O(3);
  
    P0_6 = [ x y z ]'; 
    
    ca = cos(O(4)); sa = sin(O(4));
    cb = cos(O(5)); sb = sin(O(5));
    cg = cos(O(6)); sg = sin(O(6));

    R0_6 = [    ca*cb*cg-sa*sg,   -ca*cb*sg-sa*cg,    ca*sb
                sa*cb*cg+ca*sg,   -sa*cb*sg+ca*cg,    sa*sb
                -sb*cg,           sg*sb,              cb    ];
    
    P5_6 = [23.7 0 -5.5]';

    R5_6 = [1,  0,                  0
            0,  cos(joint6_init),   -sin(joint6_init)  
            0,  sin(joint6_init),   cos(joint6_init)];
        
    P0_5 = P0_6 - (R0_6*R5_6')*P5_6;
    
    T0_6 = [[R0_6; zeros(1,3)], [P0_6; 1] ];
end

%% computes T0_3 matrix for each solution (4)
function T0_3 = compute_T0_3(joint123)
    
    T0_3 = zeros(4,4,size(joint123,2));
    
    for i = 1:size(T0_3,3)
        A1 = joint123(1,i);   A2 = joint123(2,i);   A3 = joint123(3,i);

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

        T0_3(:,:,i) = F1_to_F0 * F2_to_F1 * F3_to_F2;
    end
end

%% computes T3_6 matrix for each solution (4)
function T3_6 = compute_T3_6(T0_3, T0_6)
    
    T3_6 = zeros(4,4,size(T0_3,3));
    
    for i=1:size(T0_3,3)
        T3_6(:,:,i) = T0_3(:,:,i)\T0_6;    
    end
end

%% computes 6DOF orientation and translation joints
function joints = compute_joints456(T3_6, joint123)
    
    joints = zeros(6,0);    % init declaration
    
    for i=1:size(T3_6,3)
        T = T3_6(:,:,i);
        joint5 = round(acos(T(1,1)), 4);   joint5 = [-joint5, joint5];
        
        % if else - Compute the available positions to joint456
        % in case there is only a rotation in x
       if round(T(1,1),4) == 1
            ang = atan2(T(3,2),T(2,2));   % Rotx --->>> T3_6(3,2) == sin()  T3_6(2,2) == cos()

            joint456_4 = [ang; 0; 0];
            joint456_6 = [0; 0; ang];
            joint456 = [joint456_4, joint456_6];
       else
            joint4_aux = asin( round(T(2,1)./sin(joint5),4) );  joint4_aux = [joint4_aux; sign(T(2,1)./sin(joint5))*pi-joint4_aux];
            % when joint4 is +-pi/2, there are repeatable solutions
            if(round(cos(joint4_aux(1,1)), 2)==0)
                joint4_aux = joint4_aux.*eye(2);
            end
            joint4_flag = round(T(3,1) - (-cos(joint4_aux).*sin(joint5)), 2) == 0;
            joint4 = sum(joint4_aux.*joint4_flag,1);    % only one option per column

            joint6_aux = asin( round(T(1,2)./sin(joint5),4) );  joint6_aux = [joint6_aux; sign(T(1,2)./sin(joint5))*pi-joint6_aux];
            % when joint6 is +-pi/2, there are repeatable solutions
            if(round(cos(joint6_aux(1,1)), 2)==0)
                joint6_aux = joint6_aux.*eye(2);
            end
            joint6_flag = round(T(1,3) - cos(joint6_aux).*sin(joint5), 2) == 0;
            joint6 = sum(joint6_aux.*joint6_flag,1);

            % bound between -pi and pi
            joint4 = round( WrapToPi(joint4), 4);
            joint5 = round( WrapToPi(joint5), 4);
            joint6 = round( WrapToPi(joint6), 4); 
            
            joint456 = [joint4; joint5; joint6];
        end         
        
        % compute all 6DOF package
        angles = zeros(6, size(joint456,2));                    % each column is a different option    
        angles(1:3,:) = repmat(joint123(:,i), 1, size(joint456,2));  % combinations of the joints 123
        angles(4:6,:) = joint456 ;  % combinations of the joints 456
        
        joints = [joints angles];
    end
end

%% computes the linear geometry
function angles = get_planar_geometry(x, y, z)
        
    % ignore imprecisions of 1e-6mm.
    R = @(x) round(x,6);
    x = R(x);   y = R(y);   z = R(z);
    
    % better notation
    l2 = 210;
    l34 = sqrt(30^2 + (41.5+180)^2);%    z = l2+l34; x = 0, y=0;
    xy = sqrt(x^2+y^2); xyz = sqrt(xy^2 + z^2);
    
    beta = atan2(z, [-xy; xy]);    % angle between end effort and ground Q1
    
    phi = acos( round((l2^2 + xyz^2 - l34^2)/(2*l2*xyz),3) ); % angle between beta and length 1

    % coordinates in 2D
    theta3 = acos( round((xy.^2 + z^2 - l2^2 -l34^2)/(2*l2*l34), 3) ) ; theta3 = [-theta3 theta3];   
    theta2 = beta + phi*(2*(theta3 < 0)-1); theta2 = [theta2(1,:), theta2(2,:)];
    theta3 = [theta3 theta3];

    % Note that the rotation angles are symetric from 2D to 3D
    joint2 = pi/2 - theta2;
    joint3 = -atan(221.5/30) - theta3;
    
    % bound between -pi and pi
    if(atan2(-y,-x) == atan2(y,x))
        disp("Singularity at sight! multiple solutions for $\theta_1$");
    end
    joint1 = [ atan2(-y,-x) atan2(-y,-x) atan2(y,x) atan2(y,x) ];
    joint2 = round( WrapToPi(joint2), 4);
    joint3 = round( WrapToPi(joint3), 4);
    
    angles = [joint1; joint2; joint3];  % each solution per column
end

%% singularities
function joints_real = get_real_movements(joints_all, constrains)
    
    if nargin < 2
        constrains = true;
    end

    for i =1:size(joints_all,2)
        real_solutions(i) = isreal(joints_all(:,i));
    end

    joints_real = joints_all(:,logical(real_solutions));
    
    if ~constrains
        return
    end

    % check if it is a possible position
    f = @(x,m,M) (x>=m) .* (x<=M);
    
    min1 = round(-175*pi/180, 4);       max1 = round(175*pi/180, 4);
    min2 = round(-36.7*pi/180, 4);      max2 = round(pi/2, 4);
    min3 = round(-80*pi/180, 4);        max3 = round(pi/2, 4);
    min4 = round(-175*pi/180, 4);       max4 = round(175*pi/180, 4);
    min5 = round(-110*pi/180, 4);       max5 = round(100*pi/180, 4);
    min6 = round(-147.5*pi/180, 4);     max6 = round(147.5*pi/180, 4);

    % individuals angles for each movement
    joint1_flag = f(joints_real(1,:), min1, max1);
    joint2_flag = f(joints_real(2,:), min2, max2);
    joint3_flag = f(joints_real(3,:), min3, max3);
    joint4_flag = f(joints_real(4,:), min4, max4);
    joint5_flag = f(joints_real(5,:), min5, max5);
    joint6_flag = f(joints_real(6,:), min6, max6);
    
    % flag if movement is possible, both angles
    joint_flag = (joint1_flag+joint2_flag+joint3_flag+joint4_flag+joint5_flag+joint6_flag) == 6;
   
    joints_real = joints_real(:,joint_flag);

end

function y = WrapToPi(angle)
    if(isreal(angle))
        y = wrapToPi(angle);
        return
    end
    y = angle;
end

% each pair of angles has to be displayed in rows
function n = my_Kmeans(solution)
    
    % consedering a threshold of 1
    threshold = 1;
    len = size(solution, 1);
    K_solution = ones(len,1);
    for i = 1:len-1
        for j = (i+1):len
            dist = vecnorm(angdiff(solution(i,:),solution(j,:))); % compare two lines
            if(dist < threshold)    % same solution
                K_solution(i) = 0;
                break;  % return to the main for
            end
        end
    end
    n = sum(K_solution);
    disp("The number of groups of solution of the inverse kinematics for this specific Robot is " + num2str(n));
    solution_group = solution(logical(K_solution),:);
end

