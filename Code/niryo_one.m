function pos = niryo_one(mov, constrains, draw)

    if nargin < 3
        draw = true;
    end

    robot = rigidBodyTree; % rigid body tree
 
    %% Robot construction

    body0 = rigidBody('Support');
    
    jnt1 = rigidBodyJoint('Support-Shoulder','revolute');
    jnt1.JointAxis = [0 0 1];
    
    body1 = rigidBody('Shoulder');
    
    jnt2 = rigidBodyJoint('Shoulder-Arm','revolute');
    jnt2.JointAxis = [0 1 0];
    
    body2 = rigidBody('Arm');
    
    jnt3 = rigidBodyJoint('Arm-Elbow','revolute');
    jnt3.JointAxis = [0 1 0];

    body3 = rigidBody('Elbow');
    
    jnt4 = rigidBodyJoint('Elbow-Forearm','revolute');
    jnt4.JointAxis = [1 0 0];
    
    body4 = rigidBody('Forearm');
    
    jnt5 = rigidBodyJoint('Forearm-Wrist','revolute');
    jnt5.JointAxis = [0 1 0];
    
    body5 = rigidBody('Wrist');
    
    jnt6 = rigidBodyJoint('Wirst-Hand','revolute');
    jnt6.JointAxis = [1 0 0];
    
    body6 = rigidBody('Hand');
    
    % visual effects
    addVisual(body0,"Mesh","STL/Base.stl")
    addVisual(body1,"Mesh","STL/Shoulder.stl");
    addVisual(body2,"Mesh","STL/ARM.stl");
    addVisual(body3,"Mesh","STL/Elbow.stl") ;
    addVisual(body4,"Mesh","STL/Forearm.stl");
    addVisual(body5,"Mesh","STL/Wrist.stl");
    addVisual(body6,"Mesh","STL/Hand.stl");
    
    % robot architecture
    dhparams = [0       0   	10.3    0;
                0       0   	8       0;
                0       0   	21      0;
                4.15    0   	3       0;
                18      0   	0       0;
                2.37    0   	-0.55   0];
    
    % place spatial the joints
    setFixedTransform(jnt1,dhparams(1,:),'mdh');
    setFixedTransform(jnt2,dhparams(2,:),'mdh');
    setFixedTransform(jnt3,dhparams(3,:),'mdh');
    setFixedTransform(jnt4,dhparams(4,:),'mdh');
    setFixedTransform(jnt5,dhparams(5,:),'mdh');
    setFixedTransform(jnt6,dhparams(6,:),'mdh');
    
    % attach the join to the children so that the axis is in the base of
    % each piece
    body1.Joint = jnt1;
    body2.Joint = jnt2;
    body3.Joint = jnt3;
    body4.Joint = jnt4;
    body5.Joint = jnt5;
    body6.Joint = jnt6;
    
    % lego the robot
    addBody(robot,body0,'base')
    addBody(robot,body1,'Support')
    addBody(robot,body2,'Shoulder')
    addBody(robot,body3,'Arm')
    addBody(robot,body4,'Elbow')
    addBody(robot,body5,'Forearm')
    addBody(robot,body6,'Wrist')
   

    %showdetails(robot);

    %% Movement
              
    if constrains
        range_rotation = [-175, 175
                          -36.7, 90
                          -80, 90
                          -175, 175
                          -110, 100
                          -147.5, 147.5]*pi/180;
    else
        range_rotation = repmat([-pi, pi], 6, 1);
    end
                    
    config = homeConfiguration(robot);
    
    % bound to max angle rotation      
    for i = 1:6
        config(i).JointPosition = max( range_rotation(i,1), mov(i) );
        config(i).JointPosition = min( range_rotation(i,2), config(i).JointPosition );
    end
    
    pos = getTransform(robot,config,'Hand'); pos = pos(1:3,4);
% 
    %% plot figure
    
    if(draw)
        f = figure;
        tit1 = "Niryo One positions, after a movement.";
        tit2 = "$\theta_1$ = " + num2str(mov(1)) + " rad;   $\theta_2$ = " + num2str(mov(2)) + " rad;   $\theta_3$ = " + num2str(mov(3)) + " rad;   $\theta_4$ = " + num2str(mov(4)) + " rad;   $\theta_5$ = " + num2str(mov(5)) + " rad;   $\theta_6$ = " + num2str(mov(6)) + " rad;";

        f.Position(3) = 2*f.Position(3);
        f.Position(4) = 2*f.Position(4);

        robot_action = show(robot, config);
        robot_action.Units = "centimeters";
        robot_action.XLimMode = "auto";
        robot_action.YLimMode = "auto";
        robot_action.ZLimMode = "auto";
        
        hold on;
        title([tit1 tit2], "FontSize", 20, 'FontName', 'Arial', 'interpreter','latex')
        
        % colors
        f.CurrentAxes.Children(1).FaceColor = [1.00 0.00 0.00]; % Hand
        f.CurrentAxes.Children(2).FaceColor = [1.00 0.07 0.65]; % Wrist
        f.CurrentAxes.Children(3).FaceColor = [0.00 0.00 1.00]; % Forearm
        f.CurrentAxes.Children(4).FaceColor = [0.00 1.00 1.00]; % Elbow
        f.CurrentAxes.Children(5).FaceColor = [0.42 1.00 0.00]; % Arm
        f.CurrentAxes.Children(6).FaceColor = [0.81 0.81 0.05]; % Shoulder
        f.CurrentAxes.Children(7).FaceColor = [0.94 0.69 0.09]; % Support
        
        % camera position
        f.CurrentAxes.CameraPosition = f.CurrentAxes.CameraPosition/3;
    
    %% draw final position
    
         hold on
         %plot3(pos(1), pos(2), pos(3), 'o','Color','w','MarkerSize',30,'MarkerFaceColor','#000000');
    
    end
    pos = 10*pos; % change to mm
end