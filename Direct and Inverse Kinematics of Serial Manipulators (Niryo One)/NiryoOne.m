close all
clear
clc
%%

robot = rigidBodyTree; % rigid body tree


%% Robot construction

body0 = rigidBody('Support');
addVisual(body0,"Mesh","STL/Base.stl")

jnt1 = rigidBodyJoint('Support-Shoulder','revolute');
jnt1.JointAxis = [0 0 1];

body1 = rigidBody('Shoulder');
addVisual(body1,"Mesh","STL/Shoulder.stl");

jnt2 = rigidBodyJoint('Shoulder-Arm','revolute');
jnt2.JointAxis = [0 1 0];

body2 = rigidBody('Arm');
addVisual(body2,"Mesh","STL/ARM.stl");

jnt3 = rigidBodyJoint('Arm-Elbow','revolute');
jnt3.JointAxis = [0 1 0];

body3 = rigidBody('Elbow');
addVisual(body3,"Mesh","STL/Elbow.stl") ;

jnt4 = rigidBodyJoint('Elbow-Forearm','revolute');
jnt4.JointAxis = [1 0 0];

body4 = rigidBody('Forearm');
addVisual(body4,"Mesh","STL/Forearm.stl");

jnt5 = rigidBodyJoint('Forearm-Wrist','revolute');
jnt5.JointAxis = [0 1 0];

body5 = rigidBody('Wrist');
addVisual(body5,"Mesh","STL/Wrist.stl");

jnt6 = rigidBodyJoint('Wirst-Hand','revolute');
jnt6.JointAxis = [1 0 0];

body6 = rigidBody('Hand');
addVisual(body6,"Mesh","STL/Hand.stl");

dhparams = [0       0   	10.3    0;
            0       0   	8       0;
            0       0   	21      0;
            4.15    0   	3       0;
            18      0   	0       0;
            2.37    0   	-0.55   0];

setFixedTransform(jnt1,dhparams(1,:),'mdh');
setFixedTransform(jnt2,dhparams(2,:),'mdh');
setFixedTransform(jnt3,dhparams(3,:),'mdh');
setFixedTransform(jnt4,dhparams(4,:),'mdh');
setFixedTransform(jnt5,dhparams(5,:),'mdh');
setFixedTransform(jnt6,dhparams(6,:),'mdh');

body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;

addBody(robot,body0,'base')
addBody(robot,body1,'Support')
addBody(robot,body2,'Shoulder')
addBody(robot,body3,'Arm')
addBody(robot,body4,'Elbow')
addBody(robot,body5,'Forearm')
addBody(robot,body6,'Wrist')

showdetails(robot);

%%

robot_images = show(robot);
robot_images.Units = "centimeters";
robot_images.XLimMode = "auto";
robot_images.YLimMode = "auto";
robot_images.ZLimMode = "auto";

%%
figure
config = homeConfiguration(robot)
config(1).JointPosition = pi/2;
config(2).JointPosition = pi/2;

robot_images = show(robot, config);
robot_images.Units = "centimeters";
robot_images.XLimMode = "auto";
robot_images.YLimMode = "auto";
robot_images.ZLimMode = "auto";

