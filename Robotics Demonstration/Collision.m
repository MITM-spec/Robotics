% Collision Class
% For DoBot Collision Detection
% note: for function declaration: output = function_name(input) 

classdef Collision
    properties 
    end
    
    methods
        % class constructor 
        function self = Collision()
        end 
        
        function createEllipsoidLinks(self, dobot)
        % calculate dobot's joints transformations
        % CHECK: w/o end effector = 4, w/ end effector = 5
        % tr = zeros(4,4,dobot.model.n+1);
        % trCentre = zeros(4,4,dobot.model.n+1);
        tr = zeros(4,4,dobot.model.n+1);
        trCentre = zeros(4,4,dobot.model.n+1);
        radii = zeros(1,dobot.model.n+1);
        tr(:,:,1) = dobot.model.base;
        trCentre(:,:,1) = 0.5*(transl(0,0,0) + tr(:,:,1));
        radii(1,1) = norm(tr(1:3,4,1)' - trCentre(1:3,4,1)');
        q = dobot.model.getpos;
        L = dobot.model.links;
        for i = 1 : dobot.model.n
            % transformations of joints 
            tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha)
            % transformations of link centres
            trCentre(:,:,i+1) = 0.5*(tr(:,:,i) + tr(:,:,i+1))
            % radii of links (distance between joint to link centre 
            radii(1,i+1) = norm(tr(1:3,4,i+1)' - trCentre(1:3,4,i+1)')
        end   
%         centerPoint = [0,0,0];
%         radii = [1,0.5,0.5];
%         [X,Y,Z] = ellipsoid(centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3));
%         for i = 1 : dobot.model.n+1
%             centerPoint = trCentre(1:3,4,i)'
%             radii = [0.5,0.5
%             
%             robot.points{i} = [X(:),Y(:),Z(:)];
%             warning off
%             robot.faces{i} = delaunay(robot.points{i});    
%             warning on;
%         end
            
        end   
        function algebraicDist = GetAlgebraicDist(self, points, centerPoint, radii)
            algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
                          + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
                          + ((points(:,3)-centerPoint(3))/radii(3)).^2;
        end
    end 
end 
   