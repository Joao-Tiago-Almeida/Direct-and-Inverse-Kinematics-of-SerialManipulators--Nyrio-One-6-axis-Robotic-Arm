%% direct Kinematics
% 
a_dk = [pi/2 -pi/2 pi/4 -pi/4 pi/5 pi];
% 
o_dk = direct_transform(a_dk)
% 
pos = niryo_one(a_dk)

%% inverse Kinematics

%o_ik = [o_dk(1:3)' 0 0 0];

a_ik = inverse_transform(o_dk');

niryo_one([a_ik 0 0 0]);