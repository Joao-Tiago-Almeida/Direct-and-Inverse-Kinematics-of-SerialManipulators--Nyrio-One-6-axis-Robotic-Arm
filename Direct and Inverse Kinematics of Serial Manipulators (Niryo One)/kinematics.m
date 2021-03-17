%% direct Kinematics
% 
a_dk = [0 0 -pi*80/180 0 0 0];
% 
o_dk = direct_transform(a_dk);
% 
pos = niryo_one(a_dk)

%% inverse Kinematics

o_ik = [0 0 62 3.1416 1.3963 0];

inverse_transform(o_ik);

%niryo_one(a_ik);