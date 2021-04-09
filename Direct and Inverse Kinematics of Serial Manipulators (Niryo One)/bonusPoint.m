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
    joints6dof = compute_joints456(T3_6, joint123);  % get available solutions ( 8 preferable )

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
