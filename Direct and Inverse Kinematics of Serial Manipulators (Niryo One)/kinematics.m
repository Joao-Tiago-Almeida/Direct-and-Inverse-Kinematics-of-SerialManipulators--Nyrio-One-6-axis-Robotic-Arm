%% direct Kinematics

a_dk =  [0 0 -atan(22.15/3) 0 0 0];
%a_dk =  [-pi/4 pi/3 -pi/6 pi/2 pi/3 pi/4];
%a_dk =  [0 0 0 0 0 0];

o_dk = direct_kinematics(a_dk)


%% inverse Kinematics

a_ik = inverse_kinematics(o_dk);

try
    pos = niryo_one(a_ik(:,1), false, true);
catch
    disp("Install the Robotic ToolBox properly");
end