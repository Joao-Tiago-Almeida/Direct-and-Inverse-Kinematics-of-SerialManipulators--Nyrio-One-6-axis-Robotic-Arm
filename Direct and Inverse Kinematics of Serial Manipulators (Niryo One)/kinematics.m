%% direct Kinematics
% 
a_dk = [-pi/4 pi/3 -pi/6 pi/2 pi/3 pi/4];
% 
o_dk = direct_transform(a_dk)
% 
pos = niryo_one(a_dk, false, false);

%% inverse Kinematics

o_ik = [400 0 150 0 0 0];

[a_ik, offset, has_solutions] = inverse_transform(o_dk');

%%
if ~has_solutions
    disp("There are no available solutions!");
    return
end

disp("Found at least " + num2str(size(a_ik,2)) + " option.");
disp("Best inverse kinematic solution with " + num2str(offset(1)) + " norm error.");

disp("Best solution :");
disp(a_ik(:,1));


%%
niryo_one(a_ik(:,1), false, true)
niryo_one(a_ik(:,3), false, true)
niryo_one(a_ik(:,5), false, true)
niryo_one(a_ik(:,7), false, true)


%%
% for i=1:size(a_ik,2)
%     niryo_one(a_ik(:,i), false, false);
% end

% %% testing baby rosa
% 
% f = waitbar(0, "Acalma as Hormonas, tou a fazer");
% for i = 1:500
%     waitbar(i/500);
%     
%     pos = [200 200 300]'.*(2*rand(3,1)-1); 
%     ori = pi*(2*rand(3,1)-1);
%     test_code([pos; ori]);
% end
% close(f)
% 
% 
% function test_code(O)
% 
% [A_inv, ~, ~] = inverse_transform(O)
% flag = true;
% 
% for i = 1:length(A_inv)
%     
%     O_dir = direct_transform(A_inv(:,1))
%     
%     if(vecnorm(O(1:3,:)-O_dir(1:3,i)) > 0.1)
%         disp("merda na posicao");
%         flag = false;
%     end
%     
%     if(vecnorm(O(4:6,:)-O_dir(4:6,i)) > 0.1)
%         disp("merda na orientacao");
%         flag = false;
%     end
%     
% end
% 
% if(flag)
%     disp("All good");
% end
% 
% 
% end