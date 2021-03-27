%% direct Kinematics
% 
a_dk = [-10 33 -21 80 45 -38]*pi/180;
% 
o_dk = direct_transform(a_dk);
% 
pos = niryo_one(a_dk, false)

%% inverse Kinematics

%o_ik = [o_dk(1:3)' 0 0 0];

[a_ik, offset] = inverse_transform(o_dk');
disp("Found at least " + num2str(size(a_ik,2)) + " option.");
disp("Best solution with " + num2str(offset(1)) + " norm error.");

%%
for i=1:size(a_ik,2)
    niryo_one(a_ik(:,i), false);
end

