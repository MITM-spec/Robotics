% Robot Class
% For DoBot Robot Generation
% note: for function declaration: output = function_name(input) 

classdef Dobot < handle
    properties 
        model;
        dobotBasePose; 
        dobotEEPose; 
        dobotJS; 
        scale = 0.2;  
        workspace = [-1 1 -1 1 -0.1 1]; 
    end
    methods
        % real robot class constructor 
        function self = Dobot(dobotBasePose, dobotJS, dobotEEPose)       
            % setting dobot's base
            self.dobotBasePose = dobotBasePose;
            % setting dobot's end effector 
            self.dobotEEPose = dobotEEPose; 
            % setting dobot's joint state
            self.dobotJS = dobotJS; 
            % getting dobot
            self.GetDobotRobot();
            % plotting and adding ply files to dobot
            self.PlotAndColourRobot();
        end 
%         % simulated robot class constructor 
%         function self = Dobot(dobotBasePose)
%             % setting dobot's base
%             self.dobotBasePose = dobotBasePose;
%             % getting dobot
%             self.GetDobotRobot();
%             % plotting and adding ply files to dobot
%             self.PlotAndColourRobot();
%         end 
        
        % function to plot the dobot
        function GetDobotRobot(self)
            % dh parameters to align with suggested model dh parameters
            % CHECK offset w/ link 2, removed or not removed " ,'offset',deg2rad(-90) " 
            % L1 = 0.138 - 0.056 = 0.082 to offset for the dobot magician base
            L1 = Link('d',0.082,'a',0,'alpha',-pi/2,'qlim',[deg2rad(-135) deg2rad(135)]);
            L2 = Link('d',0,'a',0.135,'alpha',0,'qlim',[deg2rad(-5) deg2rad(85)]);
            L3 = Link('d',0,'a',0.147,'alpha',0,'qlim',[deg2rad(-10) deg2rad(95)]);
            L4 = Link('d',0,'a',0.04,'alpha',pi/2,'qlim',[deg2rad(-90) deg2rad(90)]);
            % setting the dobot dh parameters, making it an object of the serial link class
            self.model = SerialLink([L1 L2 L3 L4],'name','DOBOT_MAGICIAN','base',self.dobotBasePose);
        end
        
        % real robot function   
        function PlotAndColourRobot(self)
            % dobot's end effector home pose
            tOriginal = transl(self.dobotEEPose(1,4) + self.dobotBasePose(1,4), ... 
                self.dobotEEPose(2,4) + self.dobotBasePose(2,4), ...
                self.dobotEEPose(3,4) + self.dobotBasePose(3,4));
            % obtaining joint angles from dobot's end effector 
            q0 = self.model.ikcon(tOriginal, self.dobotJS);
            % plotting the dobot    
            % CHANGE: to 'plot3d' once ply files are in
            self.model.plot(q0,'noarrow','workspace',self.workspace, 'scale', self.scale);
            
            axis equal 
            hold on
            
            % FOR DANIEL %
            % add ply file code, follow UR5 class model
            
        end    
        
%         % simulated robot function
%         function PlotAndColourRobot(self)
%             % initial joint configurations for simulated model
%             q0 = [0, 1.4835, 0.7700, -0.6827];
%             % plotting the dobot    
%             % CHANGE: to 'plot3d' once ply files are in
%             self.model.plot(q0,'noarrow','workspace',self.workspace, 'scale', self.scale);
%             
%             axis equal 
%             hold on
%             
%             % FOR DANIEL %
%             % add ply file code, follow UR5 class model
%             
%         end    
    end 
end 
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Daniel's Original Dobot Code. 
% %% Creating and Plotting Dobot Magician
% function [Dobot] = DobotMagician()
% clf;
% clc
% scale = 0.5;
% workspace = [-2 2 -2 2 -2 2]; % workspace environment boundary for the project 
% 
% %These links were from Sensors and Controls Dobot. There was a section with
% %DH parameters for the Dobot Magician
% Q1 = Link('d',0.08,'a',0,'alpha',-pi/2,'qlim',[deg2rad(-135) deg2rad(135)]);
% Q2 = Link('d',0,'a',0.14,'alpha',0,'qlim',[deg2rad(5) deg2rad(80)],'offset', -pi/2);
% Q3 = Link('d',0,'a',0.16,'alpha',0,'qlim',[deg2rad(15) deg2rad(170)]);
% Q4 = Link('d',0,'a',0.05,'alpha',pi/2,'qlim',[deg2rad(-90) deg2rad(90)]);
% Q5 = Link('d',-0.05,'a',0,'alpha',0,'qlim',[deg2rad(-85) deg2rad(85)]);
% 
% 
% base = transl(0,0,0); %this will put the base of the robot at this coordinates
% Dobot = SerialLink([Q1 Q2 Q3 Q4 Q5],'name','DOBOT MAGICIAN','base',base); %setting the parameters of the robot      
% Dobot.teach(); % This will allow the robot to be controlled with each individual joints.
%  %This will allow us to plot the dobot in its home position
% T = transl(0.2,0.1,0.21);
% q = Dobot.ikcon(T);
% % 
% Dobot.plot(q,'workspace',workspace,'scale',scale); %this will plot the robot with the translation coordinates
% hold on;
% 
% end
