% main file 

function main()
    clc;
    clf; 

    % shuts down the existing global ros node
    rosshutdown; 

    % starts the global ros node 
    rosinit; 

    % creating all ros publishers and subscribers
    jointStateSubscriber = rossubscriber('/dobot_magician/joint_states');
    endEffectorPoseSubscriber = rossubscriber('/dobot_magician/end_effector_poses');
    toolStateSubscriber = rossubscriber('/dobot_magician/tool_state');
    visualServoingSubscriber = rossubscriber('/tf');
    [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
    [safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
    [targetJointTrajPub,targetEEMsg] = rospublisher('/dobot_magician/target_joint_states');
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    [targetEEPub,targetEEMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
    % pause to allow time for matlab to start publishers and subscribers
    pause(4);

    % % homing EE position ~ 0.2604, 0, -0.0086 
    % % homing joint position ~ 0, 0.7863, 0.7865, 0
    % % z = -0.06 to reach ar marker on ground 
    % % z = 0.05 to clear tray w/ ar marker attached 
    % % drop off location ~ 0.2, 0.18, -0.03

% % initialises dobot vmia homing procedure
%     safetyStateMsg.Data = 2;
%     send(safetyStatePublisher,safetyStateMsg);
%     pause(0.5);

    % INPUTS % 
    % INPUT dobot's base pose 43fff
    dobotBasePose = transl(0,0,0);
    % INPUT dobot's end effector rotation (set as 0)
    targetEERotation = [0,0,0];
    qua = eul2quat(targetEERotation);
    % INPUT measured offset value from dobot's centre to camera's centre 
    offsetPosition = [0.003,0.036,0];
    % INPUT dobot's safety pose
    safetyPosition = [0.2,0,0.15];
    % INPUT drop off position
    dropOffLocationPose = transl(-0.034,-0.3,0.05);
    % INPUT no. of markers ( = no. of rows)
    markerMatrix = zeros(1,3);

    % dobot's data processing class initialisation
    dobotData = Data(); 
    
    % dobot's movement class initialisation
    dobotMovement = Movement();

    % extract the joint state of robot from the received message
    currentJS = (jointStateSubscriber.LatestMessage.Position)'

    % extract the pose of the end effector from the received message
    currentEEPose = dobotData.findEEPose(endEffectorPoseSubscriber.LatestMessage)
    
    % dobot's robot class initialisation
    dobot = Dobot(dobotBasePose, currentJS, currentEEPose); 
    pause(10)
   
%     % extract the joint state of robot from model 
%     currentModelJS = dobot.model.getpos
% 
%     % extract the pose of the end effector from model
%     currentModelEEPose = dobot.model.fkine(currentModelJS)

    while 1
        % obtain ar markers locations
        while 1  
            % taking data from kinect camera (visual servoing) 
            scanVSData = receive(visualServoingSubscriber); 
            tf_0 = strcmp('ar_marker_0', scanVSData.Transforms.ChildFrameId);
            tf_1 = strcmp('ar_marker_1', scanVSData.Transforms.ChildFrameId);
            tf_10 = strcmp('ar_marker_10', scanVSData.Transforms.ChildFrameId);
            tf_avoidance = strcmp('ar_marker_2', scanVSData.Transforms.ChildFrameId);
            tf_retreat = strcmp('ar_marker_5', scanVSData.Transforms.ChildFrameId); 
            % if any non-safety markers are detected, push first detected
            % marker to 'markerMatrix'
            if tf_0 == 1 || tf_1 == 1 || tf_10 == 1 
                vs_X = scanVSData.Transforms.Transform.Translation.X; 
                vs_Y = scanVSData.Transforms.Transform.Translation.Y; 
                vs_Z = scanVSData.Transforms.Transform.Translation.Z;
                markerMatrix(1,:) = [-vs_Y, -vs_X, -0.06] - offsetPosition; 
                break;
            end 
            if safety marker is detected, actuate dobot to safety
            position 
            if tf_retreat == 1 
                while 1
                    targetEEMsg.Position.X = safetyPosition(1,1);
                    targetEEMsg.Position.Y = safetyPosition(1,2);
                    targetEEMsg.Position.Z = safetyPosition(1,3);   
                    targetEEMsg.Orientation.W = qua(1);
                    targetEEMsg.Orientation.X = qua(2);
                    targetEEMsg.Orientation.Y = qua(3);
                    targetEEMsg.Orientation.Z = qua(4);
                    send(targetEEPub, targetEEMsg);
%                     [qSimMatrix, qSimDot] = dobotMovement.calculateSimArmTrajectoryRMRC(dobot, transl(safetyPosition));
%                     for i = size(qSimMatrix, 1)
%                         animate(dobot.model,qSimMatrix(i,:));
%                         drawnow();
%                     end   
                end 
            end 
        end 
        
        
        % setting first detected ar marker location as target pose for the
        % end effector
        targetEEPose = transl(markerMatrix);
%         [qSimMatrix, qSimDot] = dobotMovement.calculateSimArmTrajectoryRMRC(dobot, targetEEPose);
%         for i = size(qSimMatrix, 1)
%             animate(dobot.model,qSimMatrix(i,:));
%             drawnow();
%         end  
        % calculate trajectory matrix to target pose, rotation set to 0
        targetPMatrix = dobotData.calculateTrajMatrix(currentEEPose, targetEEPose)
        targetPMatrix = targetPMatrix'
        pause(2)
        for i = 1:size(targetPMatrix, 1)
            targetEEMsg.Position.X = (targetPMatrix(i,1));
            targetEEMsg.Position.Y = (targetPMatrix(i,2));
            targetEEMsg.Position.Z = (targetPMatrix(i,3));      
            targetEEMsg.Orientation.W = qua(1);
            targetEEMsg.Orientation.X = qua(2);
            targetEEMsg.Orientation.Y = qua(3);
            targetEEMsg.Orientation.Z = qua(4);
            send(targetEEPub, targetEEMsg);
            pause(0.05)
            scanVSData = receive(visualServoingSubscriber);
            tf_retreat = strcmp('ar_marker_5', scanVSData.Transforms.ChildFrameId);
            if tf_retreat == 1 
%                 [qSimMatrix, qSimDot] = dobotMovement.calculateSimArmTrajectoryRMRC(dobot, transl(SafetyPosition));
%                 for i = size(qSimMatrix, 1)
%                     animate(dobot.model,qSimMatrix(i,:));
%                     drawnow();
%                 end 
                while 1
                    targetEEMsg.Position.X = safetyPosition(1,1);
                    targetEEMsg.Position.Y = safetyPosition(1,2);
                    targetEEMsg.Position.Z = safetyPosition(1,3);   
                    targetEEMsg.Orientation.W = qua(1);
                    targetEEMsg.Orientation.X = qua(2);
                    targetEEMsg.Orientation.Y = qua(3);
                    targetEEMsg.Orientation.Z = qua(4);
                    send(targetEEPub, targetEEMsg);
                end 
            end  
        end
        pause(18)
        currentEEPose = dobotData.findEEPose(endEffectorPoseSubscriber.LatestMessage);
        checkFlag = dobotData.checkDistance(currentEEPose, targetEEPose);
        if checkFlag == 0
            pause; 
        end 

        toolStateMsg.Data = [1]; 
        send(toolStatePub,toolStateMsg);
        pause(2)
        
        targetEEPose = transl(0.25, -0.17, 0.1);
%         [qSimMatrix, qSimDot] = dobotMovement.calculateSimArmTrajectoryRMRC(dobot, targetEEPose);
%         for i = size(qSimMatrix, 1)
%             animate(dobot.model,qSimMatrix(i,:));
%             drawnow();
%         end  
        % calculate trajectory matrix to target pose, rotation set to 0
        targetPMatrix = dobotData.calculateTrajMatrix(currentEEPose, targetEEPose)
        targetPMatrix = targetPMatrix'
        pause(2)
        for i = 1:size(targetPMatrix, 1)
            targetEEMsg.Position.X = (targetPMatrix(i,1));
            targetEEMsg.Position.Y = (targetPMatrix(i,2));
            targetEEMsg.Position.Z = (targetPMatrix(i,3));      
            targetEEMsg.Orientation.W = qua(1);
            targetEEMsg.Orientation.X = qua(2);
            targetEEMsg.Orientation.Y = qua(3);
            targetEEMsg.Orientation.Z = qua(4);
            send(targetEEPub, targetEEMsg);
            pause(0.05)
        end 
 
        
        % setting ar marker location as target pose
        targetEEPose = dropOffLocationPose;
%         [qSimMatrix, qSimDot] = dobotMovement.calculateSimArmTrajectoryRMRC(dobot, targetEEPose);
%         for i = size(qSimMatrix, 1)
%             animate(dobot.model,qSimMatrix(i,:));
%             drawnow();
%         end       
        % calculate trajectory matrix to target pose, rotation set to 0
        targetPMatrix = dobotData.calculateTrajMatrix(currentEEPose, targetEEPose);
        targetPMatrix = targetPMatrix'
        pause(2)
        toolStateMsg.Data = [1]; 
        send(toolStatePub,toolStateMsg);
        pause(2)
        for i = 1:size(targetPMatrix, 1)
            targetEEMsg.Position.X = (targetPMatrix(i,1));
            targetEEMsg.Position.Y = (targetPMatrix(i,2));
            targetEEMsg.Position.Z = (targetPMatrix(i,3));      
            targetEEMsg.Orientation.W = qua(1);
            targetEEMsg.Orientation.X = qua(2);
            targetEEMsg.Orientation.Y = qua(3);
            targetEEMsg.Orientation.Z = qua(4);
            send(targetEEPub, targetEEMsg);
            pause(0.05)
            scanVSData = receive(visualServoingSubscriber);
            tf_retreat = strcmp('ar_marker_5', scanVSData.Transforms.ChildFrameId);
            if tf_retreat == 1 
                while 1
                    targetEEMsg.Position.X = safetyPosition(1,1);
                    targetEEMsg.Position.Y = safetyPosition(1,2);
                    targetEEMsg.Position.Z = safetyPosition(1,3);   
                    targetEEMsg.Orientation.W = qua(1);
                    targetEEMsg.Orientation.X = qua(2);
                    targetEEMsg.Orientation.Y = qua(3);
                    targetEEMsg.Orientation.Z = qua(4);
                    send(targetEEPub, targetEEMsg);
%                     [qSimMatrix, qSimDot] = dobotMovement.calculateSimArmTrajectoryRMRC(dobot, transl(SafetyPosition));
%                     for i = size(qSimMatrix, 1)
%                         animate(dobot.model,qSimMatrix(i,:));
%                         drawnow();
%                     end 
                end 
            end  
        end
        pause(18)
        currentEEPose = dobotData.findEEPose(endEffectorPoseSubscriber.LatestMessage);
        checkFlag = dobotData.checkDistance(currentEEPose, targetEEPose);
        if checkFlag == 1
            toolStateMsg.Data = [0]; 
            send(toolStatePub,toolStateMsg); 
        else 
            pause; 
        end
    end 
end 




%             if tf_avoidance == 1 
%                 a = 0; 
%                 while 1
%                     scanVSData = receive(visualServoingSubscriber);
%                     tf_avoidance = strcmp('ar_marker_2', scanVSData.Transforms.ChildFrameId);
%                     currentEEPose = dobotData.findEEPose(endEffectorPoseSubscriber.LatestMessage);
%                     vs_X = scanVSData.Transforms.Transform.Translation.X; 
%                     vs_Y = scanVSData.Transforms.Transform.Translation.Y; 
%                     vs_Z = scanVSData.Transforms.Transform.Translation.Z;
%                     markerMatrix(1,:) = [-vs_Y, -vs_X, -0.06] - offsetPosition;
%                     distance = norm(currentEEPose(1:2,4)' - markerMatrix(1:2));
%                     if distance < 0.1
%                         display("Obstacle Near Robot")
%                     end 
%                     if tf_avoidance == 0                       
%                         pause(1)
%                         a = a + 1;
%                         if a > 5 
%                             break; 
%                         end 
%                     end
%                 end 
%             end 
%             display("progress through young padiwan")

