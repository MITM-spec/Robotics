% A = [8,2,1] 
% B = [3,1,5] 
% 
% Dist1 = sqrt(sum((A-B) .^2))
% Dist2 = norm(A - B) 
% Dist3 = sqrt( (A(1)-B(1))^2 + (A(2)-B(2))^2 x+ (A(3)-B(3))^2 )

function main()
    clc;
    clf; 

    % shuts down the existing global ros node
    rosshutdown; 

    % starts the global ros node 
    rosinit; 

    % INPUT dobot's end effector rotation (set as 0)
    targetEERotation = [0,0,0];
    qua = eul2quat(targetEERotation);
    
    % creating all ros publishers and subscribers
    jointStateSubscriber = rossubscriber('/dobot_magician/joint_states');
    endEffectorPoseSubscriber = rossubscriber('/dobot_magician/end_effector_poses');
    toolStateSubscriber = rossubscriber('/dobot_magician/tool_state');
    % visualServoingSubscriber = rossubscriber('/tf');
    [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
    [safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
    [targetJointTrajPub,targetEEMsg] = rospublisher('/dobot_magician/target_joint_states');
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    [targetEEPub,targetEEMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
    % pause to allow time for matlab to start publishers and subscribers
    pause(4);
    
    targetEEMsg.Position.X = 0.2;
    targetEEMsg.Position.Y = 0;
    targetEEMsg.Position.Z = 0.15;   
    targetEEMsg.Orientation.W = qua(1);
    targetEEMsg.Orientation.X = qua(2);
    targetEEMsg.Orientation.Y = qua(3);
    targetEEMsg.Orientation.Z = qua(4);
    send(targetEEPub, targetEEMsg);

        
    targetEEMsg.Position.X = 0;
    targetEEMsg.Position.Y = -0.2;
    targetEEMsg.Position.Z = 0.15;   
    targetEEMsg.Orientation.W = qua(1);
    targetEEMsg.Orientation.X = qua(2);
    targetEEMsg.Orientation.Y = qua(3);
    targetEEMsg.Orientation.Z = qua(4);
    send(targetEEPub, targetEEMsg);
    
        
    targetEEMsg.Position.X = 0;
    targetEEMsg.Position.Y = -0.3;
    targetEEMsg.Position.Z = 0.05;   
    targetEEMsg.Orientation.W = qua(1);
    targetEEMsg.Orientation.X = qua(2);
    targetEEMsg.Orientation.Y = qua(3);
    targetEEMsg.Orientation.Z = qua(4);
    send(targetEEPub, targetEEMsg);
    
    
end 