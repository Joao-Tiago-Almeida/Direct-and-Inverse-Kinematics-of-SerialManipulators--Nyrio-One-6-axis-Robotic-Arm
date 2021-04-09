      %% ****************************************

x0 = -1.9450;   % initial position
ub = -1.74;     % upper bound
lb = -2.15;     % lower bound

%(function, startingPos, -,-,-,-, lowerBoundToSearch, UpperBoundToSearch)
[x, error] = fmincon(@inverse_transform, x0, [], [], [], [], lb, ub);

%% This function, computes one of the best solutions (local minimum).
% @param theta6 is a MATLAB symbol
% @return deviation from the computed solution and the vector of 6 joint
% angle solution
function [bestError, solution] = inverse_transform(theta6)

    O = [295.6429, -262.7273, 199.6873, -1.8925, 2.7352, 0.8861];   % O = [x, y, z, alpha, beta, gamma];
    
    if nargin < 1
        kinematics
        return
    end
    
    % initialization
    error_vec = zeros(1,0);
    best_solution = zeros(6,0);
    
    %% start of inverse kinematics
    [P0_5, T0_6] = get_joint_position_five(O, theta6);   % new guess for joint6 angle
    joint123 = get_planar_geometry(P0_5(1), P0_5(2), P0_5(3)-10.3-8);
    T0_3 = compute_T0_3(joint123);
    T3_6 = compute_T3_6(T0_3, T0_6);
    joints6dof = compute_joints456(T3_6, joint123);  % get available solutions ( 8 preferable )
    joints6dof = get_real_movements(joints6dof, false);
    %% end of inverse kinematics
    
    % for each solution in the 8 temporary solutions
    for j = 1:size(joints6dof,2)
        error = abs(angdiff(theta6,joints6dof(6,j)));
        error_vec = [error_vec error];
        best_solution = [best_solution joints6dof(:,j)];
    end

    [error_vec_sorted, order] = sort(error_vec);        % sort by the error
    best_solution_sorted = best_solution(:,order);
    error_vec_sorted = error_vec_sorted';               % column vector
    has_solutions = logical(size(best_solution_sorted,2));
    bestError = error_vec_sorted(1);
    solution = best_solution_sorted(:,1)';
end
