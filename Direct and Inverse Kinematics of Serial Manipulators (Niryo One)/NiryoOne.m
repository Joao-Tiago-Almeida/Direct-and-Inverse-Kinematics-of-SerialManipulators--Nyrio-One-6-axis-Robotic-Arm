close all
clear
clc
%%

robot = rigidBodyTree; % rigid body tree

%% Robot construction

addVisual(robot.Base,"Mesh","STL/Base.stl")

body1 = rigidBody('Shoulder');
jnt1 = rigidBodyJoint('jnt1','revolute');
addVisual(body1,"Mesh","STL/Shoulder.stl");

body2 = rigidBody('Arm');
jnt2 = rigidBodyJoint('jnt2','revolute');
addVisual(body2,"Mesh","STL/ARM.stl");

body3 = rigidBody('Elbow');
jnt3 = rigidBodyJoint('jnt3','revolute');
addVisual(body3,"Mesh","STL/Elbow.stl");

body4 = rigidBody('Forearm');
jnt4 = rigidBodyJoint('jnt4','revolute');
addVisual(body4,"Mesh","STL/Forearm.stl");

body5 = rigidBody('Wrist');
jnt5 = rigidBodyJoint('jnt5','revolute');
addVisual(body5,"Mesh","STL/Wrist.stl");

body6 = rigidBody('Mesh');
jnt6 = rigidBodyJoint('jnt6','revolute');
addVisual(body6,"Mesh","STL/Hand.stl");

% setFixedTransform(jnt2,dhparams(2,:),'dh');
% setFixedTransform(jnt3,dhparams(3,:),'dh');
% setFixedTransform(jnt4,dhparams(4,:),'dh');
% setFixedTransform(jnt5,dhparams(5,:),'dh');
% setFixedTransform(jnt6,dhparams(6,:),'dh');

body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;

addBody(robot,body1,'base')
addBody(robot,body2,'Shoulder')
addBody(robot,body3,'Arm')
addBody(robot,body4,'Elbow')
addBody(robot,body5,'Forearm')
addBody(robot,body6,'Wrist')


% body.Joint = rigidBodyJoint('Forearm','revolute');
% addBody(robot,body,'Forearm');
% addVisual(robot.Base,"Mesh","STL/Forearm.stl");
% 
% body.Joint = rigidBodyJoint('Wrist','revolute');
% addBody(robot,body,'Wrist');
% addVisual(robot.Base,"Mesh","STL/Wrist.stl");
% 
% body.Joint = rigidBodyJoint('Hand','revolute');
% addBody(robot,body,'Hand');
% addVisual(robot.Base,"Mesh","STL/Hand.stl");

robot_images = show(robot);
robot_images.Units = "centimeters";
robot_images.XLimMode = "auto";
robot_images.YLimMode = "auto";
robot_images.ZLimMode = "auto";
showdetails(robot);