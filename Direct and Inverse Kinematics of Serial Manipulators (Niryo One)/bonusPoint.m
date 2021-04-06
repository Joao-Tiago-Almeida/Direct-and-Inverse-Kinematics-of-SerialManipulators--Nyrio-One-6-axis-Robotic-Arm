      %% ****************************************

% O = [x, y, z, A, B, G];
x0 = -1.9450;
[x, error] = fmincon(@inverse_transform, x0, [], [], [], [], -2.15, -1.74) %(function, startingPos, -,-,-,-, lowerBoundToSearch, UpperBoundToSearch)

%For the point O = [295.6429, -262.7273, 199.6873, -1.8925, 2.7352,
%0.8861];, it gives 4 regions:

% region1: [-2.15, -1.74]
% region2: [-1.64, -0.84]
% region3: [0.558, 1.558]
% region4: [1.558, 2.558]

%Angles for region 1:
%    2.3704
%    -0.9923
%    -2.4351
%     1.4388
%    -1.0499
%    -2.0813
%error = 7.2374e-09

%***
%    -0.7742
%     2.0352
%    -2.4357
%    -2.0083
%    -1.2555
%    -1.3597

%***
%    2.3565
%    -1.0472
%    -2.3410
%    -1.7278
%     1.0688
%     1.1026

%***
% 2.3608
%    -1.9736
%    -0.5262
%    -2.0617
%     1.3690
%     2.0022

function [bestError, solution] = inverse_transform(theta6)
    O = [295.6429, -262.7273, 199.6873, -1.8925, 2.7352, 0.8861];
    if nargin < 1
        kinematics
        return
    end
    
    error_vec = zeros(1,0);
    best_solution = zeros(6,0);

        [P0_5, T0_6] = get_joint_position_five(O, theta6);   % new guess for joint6 angle
        
        joint123 = get_planar_geometry(P0_5(1), P0_5(2), P0_5(3)-10.3-8);

        T0_3 = compute_T0_3(joint123);
        T3_6 = compute_T3_6(T0_3, T0_6);
        joints6dof = compute_combinations(T3_6, joint123);  % get available solutions ( 8 preferable )
        
        joints6dof = get_real_movements(joints6dof, false);
        
        for j = 1:size(joints6dof,2)
            
            error = abs(angdiff(theta6,joints6dof(6,j)));
            error_vec = [error_vec error];
            best_solution = [best_solution joints6dof(:,j)];
        end

    
    [error_vec_sorted, order] = sort(error_vec);  % sort by the theta 6
    best_solution_sorted = best_solution(:,order);
    error_vec_sorted = error_vec_sorted'; % column vector
    has_solutions = logical(size(best_solution_sorted,2));
    bestError = error_vec_sorted(1);
    solution = best_solution_sorted(:,1)
end

%% compute fifth joint position
function [P0_5, T0_6] = get_joint_position_five(O, joint6_init)
    
    x = O(1); y = O(2); z = O(3);
  
    P0_6 = 0.1*[ x y z ]';  % to cm
    
    ca = cos(O(4)); sa = sin(O(4));
    cb = cos(O(5)); sb = sin(O(5));
    cg = cos(O(6)); sg = sin(O(6));

    R0_6 = [    ca*cb*cg-sa*sg,   -ca*cb*sg-sa*cg,    ca*sb
                sa*cb*cg+ca*sg,   -sa*cb*sg+ca*cg,    sa*sb
                -sb*cg,           sg*sb,              cb    ];
    
    P5_6 = [2.37 0 -0.55]';

    R5_6 = [1,  0,                  0
            0,  cos(joint6_init),   -sin(joint6_init)  
            0,  sin(joint6_init),   cos(joint6_init)];
        
    P0_5 = P0_6 - (R0_6*(R5_6'))*P5_6;
    
    T0_6 = [[R0_6; zeros(1,3)], [P0_6; 1] ];
end

%% draw T0_3 matrix
function T0_3 = compute_T0_3(joint123)
    
    T0_3 = zeros(4,4,size(joint123,2));
    
    for i = 1:size(T0_3,3)
        A1 = joint123(1,i);   A2 = joint123(2,i);   A3 = joint123(3,i);

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

        T0_3(:,:,i) = F0_to_F1 * F1_to_F2 * F2_to_F3;
    end
end

%% draw T3_6 matrix
function T3_6 = compute_T3_6(T0_3, T0_6)
    
    T3_6 = zeros(4,4,size(T0_3,3));
    
    for i=1:size(T0_3,3)
        T3_6(:,:,i) = T0_3(:,:,i)\T0_6;    
    end
end

%% compute 6DOF orientation and translation joints
function joints = compute_combinations(T3_6, joint123)
    
    joints = zeros(6,0);    % init declaration
    
    for i=1:size(T3_6,3)
        
        
        T = T3_6(:,:,i);
        joint5 = round(acos(T(1,1)), 4);   joint5 = [-joint5, joint5];
        
        % if else - Compute the available positions to joint456
        % in case there is only a rotation in x
        if round(T(1,1),4) == 1
            ang = atan2(T(3,2),T(2,2));   % Rotx --->>> T3_6(3,2) == sin()  T3_6(2,2) == cos()
            
            % force maximum rotation in joint4
            if abs(ang) > 175*pi/180
                joint4 = sign(ang)*(175*pi/180);
                joint6 = sign(ang)*(abs(ang) - 175*pi/180);
                joint456_4 = [joint4; 0; joint6]

            else
                joint456_4 = [ang; 0; 0];
            end
            
            % force maximum rotation in joint6
            if abs(ang) > 147.5*pi/180
                joint4 = sign(ang)*(147.5*pi/180);
                joint6 = sign(ang)*(abs(ang) - 147.5*pi/180);
                joint456_6 = [joint4; 0; joint6]

            else
                joint456_6 = [ang; 0; 0];
            end
        
            joint456 = [joint456_4, joint456_6];
            
        else

            joint4_aux = asin( round(T(2,1)./sin(joint5),4) );  joint4_aux = [joint4_aux; sign(joint4_aux)*pi-joint4_aux];
            % when joint4 is ±pi/2, there are repeatable solutions
            if(round(cos(joint4_aux(1,1)), 2)==0)
                joint4_aux = joint4_aux.*eye(2);
            end
            joint4_flag = round(T(3,1), 2) == round(-cos(joint4_aux).*sin(joint5), 2);
            joint4 = sum(joint4_aux.*joint4_flag,1);    % only one option per column

            joint6_aux = asin( round(T(1,2)./sin(joint5),4) );  joint6_aux = [joint6_aux; sign(joint6_aux)*pi-joint6_aux];
            % when joint6 is ±pi/2, there are repeatable solutions
            if(round(cos(joint6_aux(1,1)), 2)==0)
                joint6_aux = joint6_aux.*eye(2);
            end
            joint6_flag = round(T(1,3), 2) == round(cos(joint6_aux).*sin(joint5), 2);
            joint6 = sum(joint6_aux.*joint6_flag,1);

            % bound between -pi and pi
            joint4 = round( wrapToPi(joint4), 4);
            joint5 = round( wrapToPi(joint5), 4);
            joint6 = round( wrapToPi(joint6), 4); 
            
            joint456 = [joint4; joint5; joint6];
        end         
        
        % compute all 6DOF package
        angles = zeros(6, size(joint456,2));                    % each column is a different option    
        angles(1:3,:) = repmat(joint123(:,i), 1, size(joint456,2));  % combinations of the joints 123
        angles(4:6,:) = joint456 ;  % combinations of the joints 456
        
        joints = [joints angles];
    end
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
    joint2 = round( wrapToPi(joint2), 4);
    joint3 = round( wrapToPi(joint3), 4);
    
    angles = [joint1; joint2; joint3];  % each solution per column
end

%% singularities
function joints_real = get_real_movements(joints_all, constrains)
    
    if nargin < 2
        constrains = true;
    end
    
    
    real_solutions = zeros(size(joints_all,2), 1);  % has to be 8x1!

    for i =1:size(real_solutions,1)
        real_solutoins(i) = isreal(joints_all(:,i));
    end
    
    if(imag(joints_all) | sum(real_solutoins)~=8)
        disp("Line 216! By the order of fucking Peaky Blinders!! Tommy says: " + sum(real_solutoins));
    end

    joints_real = joints_all(:,real_solutoins);
    
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
      
      
function d = angdiff(th1, th2)

    switch nargin
        case 1
            if length(th1) == 2
                d = th1(1) - th1(2);
            else
                d = th1;
            end
        case 2
            if length(th1) > 1 && length(th2) > 1
                % if both arguments are vectors, they must be the same
                assert(all(size(th1) == size(th2)), 'SMTB:angdiff:badarg', 'vectors must be same shape');
            end
            % th1 or th2 could be scalar
            d = th1 - th2;
    end

    % wrap the result into the interval [-pi pi)
    d = mod(d+pi, 2*pi) - pi;
end