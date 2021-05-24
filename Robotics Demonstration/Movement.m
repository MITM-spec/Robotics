% Movement Class
% For DoBot Robot Movement 
% note: for function declaration: output = function_name(input)
% note: '~' is a substitute for 'self' 

classdef Movement
    properties 
    end 
    
    methods 
        % class constructor
        function self = Movement()     
        end 
        
        function [qSimMatrix] = calculateArmTrajectory(self, dobot, targetEEPose)
            currentQ = dobot.model.getpos; 
            targetQ = dobot.model.ikcon(targetEEPose, currentQ);
            steps = 100; 
            s = lspb(0,1,steps);
            % array for joint angles
            % CHECK: w/o end effector = 4, w/ end effector = 5  
            qSimMatrix = nan(steps, 4); 
            for i = 1:steps
                qSimMatrix(i,:) = (1 - s(i))*currentQ + s(i)*targetQ;  
            end 
        end
                
        % to move dobot's arm via RMRC, taken from Lab 9 Solution
        % time = 10, deltaT = 0.02, epsilon = 0.1 
        function [qSimMatrix, qSimDot] = calculateSimArmTrajectoryRMRC(self, dobot, targetEEPose)
            % PARAMETERS FOR SIMULATION % 
            % time required per movement 
            time = 10;
            % control frequency, must be very small for rotational error to be
            % calculated correctly in RMRC
            deltaT = 0.2;
            % no. of steps for simulation
            steps = ceil(time/deltaT); 
            % threshold value for manipulability (Damped Least Squares)
            epsilon = 0.1;
            % weighting matrix for velocity
            % 1 x 6 to represent 3 translational and 3 angular velocities 
            W = diag([1 1 1 1 1 1]); 
            
            % ALLOCATION OF ARRAY DATA % 
            % pre-allocation done to improve the speed of the code as data 
            % changes with every new iteration of information
            % array for measure of manipulability 
            m = zeros(steps, 1);
            % array for joint angles
            % CHECK: w/o end effector = 4, w/ end effector = 5 
            qSimMatrix = zeros(steps, 4);     
            % array for joint velocities
            % CHECK: w/o end effector = 4, w/ end effector = 5
            qSimDot = zeros(steps, 4); 
            % array for joint angles
            % CHECK: w/o end effector = 4, w/ end effector = 5 
            qRealMatrix = zeros(steps, 4);     
            % array for joint velocities
            % CHECK: w/o end effector = 4, w/ end effector = 5
            qRealDot = zeros(steps, 4); 
            % array for roll-pitch-yaw (x-y-z) angles
            theta = zeros(3, steps);
            % array for x-y-z trajectory
            x = zeros(3, steps);
            
            currentEEPose = dobot.model.fkine(dobot.model.getpos);
            % calculate roll-pitch-yaw angles from input poses 
            % given (1 x 3) matrix w/ angles in radians
            eERPY = tr2rpy(currentEEPose); 
            boxRPY = tr2rpy(targetEEPose); 
            
            % interpolate trajectory (position) from end effector pose to box pose
            s = lspb(0, 1, steps);
            for i = 1:steps
                % x points trajectory
                x(1,i) = (1-s(i))*currentEEPose(1,4) + s(i)*targetEEPose(1,4); 
                % y points trajectory
                x(2,i) = (1-s(i))*currentEEPose(2,4) + s(i)*targetEEPose(2,4); 
                % z points trajectory
                x(3,i) = (1-s(i))*currentEEPose(3,4) + s(i)*targetEEPose(3,4); 
                % roll (x) angle trajectory
                theta(1,i) = (1-s(i))*eERPY(1,1) + s(i)*boxRPY(1,1);  
                % pitch (y) angle trajectory 
                theta(2,i) = (1-s(i))*eERPY(1,2) + s(i)*boxRPY(1,2);    
                % yaw (z) angle trajectory
                theta(3,i) = (1-s(i))*eERPY(1,3) + s(i)*boxRPY(1,3);                     
            end 
            
            % taking simulation current joint configs as first step in qMatrix 
            qSimMatrix(1,:) = dobot.model.getpos;
            
            % track trajectory (velocity) from end effector pose to box pose
            % MN: see handwritten notes on back of 8.5 RMRC lecture & 10.4 DLS lecture notes 
            for i = 1:steps-1
                % calculating T via fkine instead of 'currentEEPose' to
                % account for slight discrepancies
                % get forward transformation at current joint state
                T = dobot.model.fkine(qSimMatrix(i,:));
                % calculate position error from next waypoint aka p(t+1) - p(t)
                deltaX = x(:,i+1) - T(1:3,4);       
                % get next roll-pitch-yaw angles, convert to rotation matrix aka R(t + 1)
                % given (3 x 3) rotational matrix
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));  
                % current end-effector rotation matrix aka R(t)
                Ra = T(1:3,1:3); 
                % calculate rotation matrix error from next waypoint
                Rdot = (1/deltaT)*(Rd - Ra); 
                % calculate skew-symmetric matrix
                S = Rdot*Ra';
                % calculate linear velocity required for pose 1 to pose 2
                linear_velocity = (1/deltaT)*deltaX;
                % calculate angular velocity required for pose 1 to pose 2
                % roll-pitch-yaw angles given by position in skew-symmetric matrix
                angular_velocity = [S(3,2);S(1,3);S(2,1)];  
                % calculate end-effector velocity to reach next waypoint
                % given (6 x 1) matrix 
                xdot = W*[linear_velocity;angular_velocity];  
                % get jacobian w/ respective to the base at current joint state
                J = dobot.model.jacob0(qSimMatrix(i,:)); 
                % calculate yoshikawa's measure of manipulability 
                m(i) = sqrt(det(J*J'));
                % if manipulability is less than given threshold (epsilon)
                % 'lambda' is damping factor
                if m(i) < epsilon
                    % damping factor switched on
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    % damping factor switched off
                    lambda = 0;
                end
                
                % CALCULATE DLS INVERSE - CHANGE AS REQUIRED
                % CHECK: w/o end effector = 4, w/ end effector = 5
                % no dls inverse 
                % invJ = J'*inv(J*J');
                % dls inverse as fully-actuated
                % invJ = inv(J'*J + lambda*eye(5))*J'; 
                % dls inverse as under-actuated
                 invJ = J'*inv(J*J' + lambda*eye(6));   
                
                % solve the RMRC equation, calculate joint velocities
                % transposed from (5 x 1) matrix to (1 x 5) matrix to align 
                % with qdot's structure
                qSimDot(i,:) = (invJ*xdot)'; 
                % CHECK: w/o end effector = 4, w/ end effector = 5
                for j = 1:4        
                    % if next joint angle is lower than joint limit ...
                    if qSimMatrix(i,j) + deltaT*qSimDot(i,j) < dobot.model.qlim(j,1)  
                        % stop the motor
                        qSimDot(i,j) = 0;
                    % if next joint angle is greater than joint limit ...
                    elseif qSimMatrix(i,j) + deltaT*qSimDot(i,j) > dobot.model.qlim(j,2)      
                        % stop the motor
                        qSimDot(i,j) = 0; 
                    end
                end
                % update next joint state based on joint velocities
                qSimMatrix(i+1,:) = qSimMatrix(i,:) + deltaT*qSimDot(i,:);
            end 
            qSimMatrix = real(qSimMatrix); 
            qSimDot = real(qSimDot);
        end
         
        % ikine solver to determine real joint angles (q1,q2,q3,q4)
        % given an end-effector pose and simQ 
        % FIX, ADD Q5
        function [realQ] = calculateRealQs(self, goalPose, qSimMatrixRow) 
            x = goalPose(1,4);
            y = goalPose(2,4);
            z = goalPose(3,4);
            l = sqrt(x^2 + y^2);
            h = z; 
            a2 = 0.135;
            a3 = 0.147;
            q1 = atan(y/x);
            q2 = qSimMatrixRow(1,2);
            q3 = asin((h - a2*cos(q2))/a3);
            q4 = pi/2 - q3;       
            realQ = [q1, q2, q3, q4];        
        end 
    end 
end 
        