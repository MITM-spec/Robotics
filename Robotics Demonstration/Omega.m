% Robot Class
% For DoBot Robot Generation
% note: for function declaration: output = function_name(input) 

classdef Omega < handle
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

        % simulated robot class constructor 
        function self = Omega(dobotBasePose)
            % setting dobot's base
            self.dobotBasePose = dobotBasePose;
            % getting dobot
            self.GetDobotRobot();
            % plotting and adding ply files to dobot
            self.PlotAndColourRobot();
        end 
        
        % function to plot the dobot
        function GetDobotRobot(self)
            % dh parameters to align with suggested model dh parameters
            % CHECK offset w/ link 2, removed or not removed " ,'offset',deg2rad(-90) " 
            % L1 = 0.138 - 0.056 = 0.082 to offset for the dobot magician base
            L1 = Link('d',0.1,'a',0,'alpha',-pi/2,'qlim',[deg2rad(-135) deg2rad(135)]);
            L2 = Link('d',0,'a',0.135,'alpha',0,'qlim',[deg2rad(-5) deg2rad(85)]);
            L3 = Link('d',0,'a',0.147,'alpha',0,'qlim',[deg2rad(-10) deg2rad(95)]);
            L4 = Link('d',0,'a',0.04,'alpha',pi/2,'qlim',[deg2rad(-90) deg2rad(90)]);
            % setting the dobot dh parameters, making it an object of the serial link class
            self.model = SerialLink([L1 L2 L3 L4],'name','DOBOT_MAGICIAN','base',self.dobotBasePose);
        end
        
  
        
%         % simulated robot function
        function PlotAndColourRobot(self)
            % initial joint configurations for simulated model
            q0 = [0, -0.0873, 0.9616, -0.8857];
            % plotting the dobot    
            % CHANGE: to 'plot3d' once ply files are in
            self.model.plot(q0,'noarrow','workspace',self.workspace, 'scale', self.scale);
            
            for linkIndex = 0:self.model.n
                
                [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['DBLINK',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end
            
            % Display robot
             self.model.plot3d(q0,'noarrow','workspace',self.workspace, 'scale', self.scale);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.model.delay = 0;
            
            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
            
            
            
            
            axis equal
            hold on
            
           
            
        end    
    end 
end 
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

