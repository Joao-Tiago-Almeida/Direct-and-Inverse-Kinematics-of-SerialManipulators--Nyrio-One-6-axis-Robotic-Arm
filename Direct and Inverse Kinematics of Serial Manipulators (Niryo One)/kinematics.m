%% direct Kinematics
% 
a_dk = [0 0 -pi/2 0 0 0];
% 
o_dk = direct_transform(a_dk);
% 
pos = niryo_one(a_dk);

%% inverse Kinematics

o_ik = [-10 0 20 3.1416 1.3963 0];

inverse_transform(o_ik);

%niryo_one(a_ik);