% Data Processing Class
% For DoBot ROS Data Processing 
% note: for function declaration: output = function_name(input)
% note: '~' is a substitute for 'self' 

classdef Data
    properties 
    end 
    
    methods 
        % class constructor
        function self = Data()     
        end 
        
        function currentEEPose = findEEPose(self, currentEEPoseMsg)
            % extract the pose of the end effector from the received message
            % extract the position of the end effector
            currentEEPosition = [currentEEPoseMsg.Pose.Position.X, ... 
                                          currentEEPoseMsg.Pose.Position.Y, ...
                                          currentEEPoseMsg.Pose.Position.Z];
            % extract the orientation of the end effector
            currentEEQuat = [currentEEPoseMsg.Pose.Orientation.W, ...
                                      currentEEPoseMsg.Pose.Orientation.X, ... 
                                      currentEEPoseMsg.Pose.Orientation.Y, ... 
                                      currentEEPoseMsg.Pose.Orientation.Z];
            % convert from orientation from quaternion to euler
            currentEEEuler = quat2eul(currentEEQuat, 'XYZ');
            % convert pose to 4x4 matrix
            currentEEPose = transl(currentEEPosition)*trotx(currentEEEuler(1,1)) ... 
                *troty(currentEEEuler(1,2))*trotz(currentEEEuler(1,3));
        end 
        
        % ADD functionality for rotation
        function targetEEPose = visualServoingOffset(self, offsetPose, targetVSPose) 
            x = offsetPose(1,4) + targetVSPose(1,4); 
            y = offsetPose(2,4) + targetVSPose(2,4); 
            z = offsetPose(3,4) + targetVSPose(3,4); 
            targetEEPose = transl(x,y,z); 
        end 
        
        function checkFlag = checkDistance(self, currentEEPose, targetEEPose)
            checkFlag = 0;
            distance = norm(currentEEPose(1:3,4)' - targetEEPose(1:3,4)')
            if distance < 0.05
                checkFlag = 1;
            else 
                checkFlag = 0; 
            end 
        end 
            
        function targetPMatrix = calculateTrajMatrix(self, currentEEPose, targetEEPose)
            actionSteps = 10;
            totalSteps = 30;
%             % array for roll-pitch-yaw (x-y-z) angles
%             targetRMatrix = zeros(3, steps);
            % array for x-y-z trajectory
            targetPMatrix_1 = zeros(3, actionSteps); 
            targetPMatrix_2 = zeros(3, actionSteps); 
            targetPMatrix_3 = zeros(3, actionSteps); 
            targetPMatrix = zeros(3, totalSteps);         
%             % calculate roll-pitch-yaw angles from input poses 
%             % given (1 x 3) matrix w/ angles in radians
%             eERPY = tr2rpy(currentEEPose); 
%             boxRPY = tr2rpy(targetEEPose);          
            % interpolate trajectory (position) from end effector pose to
            % box pose via jump mode 
            s = lspb(0, 1, actionSteps);
            for i = 1:actionSteps
                % x points trajectory
                targetPMatrix_1(1,i) = (1-s(i))*currentEEPose(1,4) + s(i)*currentEEPose(1,4);
                % y points trajectory
                targetPMatrix_1(2,i) = (1-s(i))*currentEEPose(2,4) + s(i)*currentEEPose(2,4);
                % z points trajectory (0.06)
                targetPMatrix_1(3,i) = (1-s(i))*currentEEPose(3,4) + s(i)*0.1;
%                 % roll (x) angle trajectory
%                 targetRMatrix(1,i) = (1-s(i))*eERPY(1,1) + s(i)*boxRPY(1,1);  
%                 % pitch (y) angle trajectory 
%                 targetRMatrix(2,i) = (1-s(i))*eERPY(1,2) + s(i)*boxRPY(1,2);    
%                 % yaw (z) angle trajectory
%                 targetRMatrix(3,i) = (1-s(i))*eERPY(1,3) + s(i)*boxRPY(1,3);                     
            end 
            s = lspb(0, 1, actionSteps);
            for i = 1:actionSteps
                % x points trajectory
                targetPMatrix_2(1,i) = (1-s(i))*currentEEPose(1,4) + s(i)*targetEEPose(1,4); 
                % y points trajectory
                targetPMatrix_2(2,i) = (1-s(i))*currentEEPose(2,4) + s(i)*targetEEPose(2,4); 
                % z points trajectory
                targetPMatrix_2(3,i) = (1-s(i))*0.1 + s(i)*0.1; 
%                 % roll (x) angle trajectory
%                 targetRMatrix(1,i) = (1-s(i))*eERPY(1,1) + s(i)*boxRPY(1,1);  
%                 % pitch (y) angle trajectory 
%                 targetRMatrix(2,i) = (1-s(i))*eERPY(1,2) + s(i)*boxRPY(1,2);    
%                 % yaw (z) angle trajectory
%                 targetRMatrix(3,i) = (1-s(i))*eERPY(1,3) + s(i)*boxRPY(1,3);                     
            end         
            s = lspb(0, 1, actionSteps);
            for i = 1:actionSteps
                % x points trajectory
                targetPMatrix_3(1,i) = (1-s(i))*targetEEPose(1,4) + s(i)*targetEEPose(1,4); 
                % y points trajectory
                targetPMatrix_3(2,i) = (1-s(i))*targetEEPose(2,4) + s(i)*targetEEPose(2,4); 
                % z points trajectory
                targetPMatrix_3(3,i) = (1-s(i))*0.1 + s(i)*(targetEEPose(3,4)); 
%                 % roll (x) angle trajectory
%                 targetRMatrix(1,i) = (1-s(i))*eERPY(1,1) + s(i)*boxRPY(1,1);  
%                 % pitch (y) angle trajectory 
%                 targetRMatrix(2,i) = (1-s(i))*eERPY(1,2) + s(i)*boxRPY(1,2);    
%                 % yaw (z) angle trajectory
%                 targetRMatrix(3,i) = (1-s(i))*eERPY(1,3) + s(i)*boxRPY(1,3);                     
            end 
            targetPMatrix = [targetPMatrix_1, targetPMatrix_2, targetPMatrix_3];
        end 
    end 
end 

        