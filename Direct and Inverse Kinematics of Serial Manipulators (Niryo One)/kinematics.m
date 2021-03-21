%% direct Kinematics
% 
a_dk = [pi/4 -pi/3 -50*pi/180 0 0 0];
% 
o_dk = direct_transform(a_dk);
% 
pos = niryo_one(a_dk, false);

%% inverse Kinematics

%o_ik = [o_dk(1:3)' 0 0 0];

a_ik = inverse_transform(o_dk');
disp("Found at least " +num2str(size(a_ik,2)) + " option");

%%
for i=1:size(a_ik,2)
    niryo_one(a_ik(:,i), true);
end