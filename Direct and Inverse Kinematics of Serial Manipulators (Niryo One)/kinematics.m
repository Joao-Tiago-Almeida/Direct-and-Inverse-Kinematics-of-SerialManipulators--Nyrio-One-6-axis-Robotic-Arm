%% direct Kinematics
% 
a_dk = [0 pi/10 -pi/9 0 0 0];
% 
o_dk = direct_transform(a_dk)
% 
pos = niryo_one(a_dk, false, true);

%% inverse Kinematics

o_ik = [400 0 150 0 0 0];

[a_ik, offset, has_solutions] = inverse_transform(o_ik');

%%
if ~has_solutions
    disp("There are no available solutions!");
    return
end

disp("Found at least " + num2str(size(a_ik,2)) + " option.");
disp("Best solution with " + num2str(error(1)) + " norm error.");

%%
for i=1:size(a_ik,2)
    niryo_one(a_ik(:,i), false, false);
end

