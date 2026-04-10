function [R,increaseVideoMove]=moveRobot(node,nodeList,R,slice_time,currentEdgeDisconnected)


if R.moving
    if currentEdgeDisconnected == 1 % if the current edge of the robot is disconnected, then define te robot node.
       parentRobotNode=node(R.robotNode).parent;
       R.robotEdge= [R.robotNode;parentRobotNode];
    end
    R.robotPose=R.nextRobotPose;
    increaseVideoMove=R.numLocalMovePoints;
    R.robotMovePath(R.numRobotMovePoints+1:R.numRobotMovePoints+R.numLocalMovePoints,:) = R.robotLocalPath(1:R.numLocalMovePoints,:);
    R.numRobotMovePoints = R.numRobotMovePoints + R.numLocalMovePoints;
    
    %save stuff for plotting
    R.distAlongRobotEdgeForPlotting = R.distAlongRobotEdge;
%     R.robotEdgeForPlotting = R.robotEdge;
%     R.robotEdgeForPlottingUsed = true;
else
    % movement has just started, so remember that the robot is now moving
    R.moving=true;
    increaseVideoMove=2;
    
    if isempty(node(1).parent)
        % no parent has been found for the node at the robot's position
        R.currentMoveInvalid = true;
    else
        parentRobotNode=node(R.robotNode).parent;
        R.robotEdge= [R.robotNode;parentRobotNode];

%         R.robotEdgeForPlotting= [nodeList(1,R.robotNode),nodeList(1,parentRobotNode);nodeList(2,R.robotNode),nodeList(2,parentRobotNode)];
        R.robotEdgeUsed = true;
%         R.robotEdgeForPlottingUsed = true;
        
        % Distance along robot Edge
        
        R.distAlongRobotEdge = 0.0;
        R.distAlongRobotEdgeForPlotting = 0.0;        
        
    end
end

%if the robot's current move target has been invalidated due to
%dynamic obstacles then we need to attempt to find a new (safe) move target
%NOTE we handle newly invalid moveTarget after moving the robot

if R.currentMoveInvalid
    %findNewTarget(S, KD, R, hyberBallRad)
end

% finally, we calculate the point to which the robot will move to in slice_time
% and remember it for the next time this function is called. we also remember
% all the nodes that it will visit along the way in the local path
% and the part of the edge trajectory that takes the robot to the first local
% point (the latter two things are used for visuialization)

% if !S.spaceHasTime


% % % nextNode= R.robotEdge(2);
% % % parentRobotNode= node(R.robotNode).parent;
% % % %calculate distance from robot to the end of current edge it is following
% % % nextDist = distNeighbMat((R.robotNode),parentRobotNode) - R.distAlongRobotEdge;
% % % 
% % % distRemaining = R.robotVelocity*slice_time ;
% % % 
% % % % save first local path point
% % % R.numLocalMovePoints = 1;
% % % R.robotLocalPath(R.numLocalMovePoints,:) = R.robotPose;

nextNode= R.robotEdge(2);

R.numLocalMovePoints = 1;
R.robotLocalPath(R.numLocalMovePoints,:) = R.robotPose;

targetTime = R.robotPose(3)- slice_time;

while (targetTime <= nodeList(3,nextNode) && nextNode ~= 2 && ~isempty(node(nextNode).parent)&& nextNode ~= node(nextNode).parent)

% can go all the way to nextNode and still have some distance left to spare

    % remember the robot will move through this point
    R.numLocalMovePoints = R.numLocalMovePoints +1;
    R.robotLocalPath(R.numLocalMovePoints,:) = nodeList(:,nextNode);
    
    %update trajectory that the robot will be in the middle of
    R.robotEdge= [nextNode;node(nextNode).parent];
    R.robotNode=nextNode; %% bu esitlemeden emin degilim
    
    % update the next node (at the end of that trajectory)
    nextNode = node(nextNode).parent;
    
end

    % either: 1) targetTime >= nextNode.position[3]
    %or)    2) the path we were following now ends at nextNode
if targetTime >= nodeList(3,nextNode)
    R.timeAlongRobotEdge = nodeList(3,R.robotEdge(1)) - targetTime;
    
    % returns the pose of a robot that is located time along the edge
    ratioAlongEdge= R.timeAlongRobotEdge/ ( nodeList(3,R.robotEdge(1)) - nodeList(3,R.robotEdge(2)) );
    R.nextRobotPose = nodeList(:,R.robotEdge(1)) + ratioAlongEdge*(nodeList(:,R.robotEdge(2)) - nodeList(:,R.robotEdge(1)) );
    
else
    % the next node is the end of this tree and we reach it
    R.nextRobotPose = nodeList(:,nextNode);
    R.timeAlongRobotEdge = nodeList(3,R.robotEdge(1)) - nodeList(3,R.robotEdge(2));
    
end


% % % % starting at current location (and looking ahead to nextNode), follow parent
% % % % pointers back for the approperiate distance (or root or dead end)
% % % while (nextDist <= distRemaining && nextNode ~= 2 && ~isempty(node(nextNode).parent)&& nextNode ~= node(nextNode).parent)
% % % 
% % %     % can go all the way to nextNode and still have some distance left to spare
% % % 
% % %     % remember the robot will move through this point
% % %     R.numLocalMovePoints = R.numLocalMovePoints +1;
% % %     R.robotLocalPath(R.numLocalMovePoints,:) = nodeList(1:2,nextNode);
% % % 
% % %     %recalculate remaining distance
% % %     distRemaining = distRemaining- nextDist;
% % %     %reset distance along edge
% % %     R.distAlongRobotEdge = 0.0;
% % %     
% % %     %update trajectory that the robot will be in the middle of
% % %     R.robotEdge= [nextNode;node(nextNode).parent];
% % %     R.robotNode=nextNode;
% % %     
% % %     % calculate the dist of that trajectory
% % %     nextDist = distNeighbMat(nextNode,node(nextNode).parent);
% % %     
% % %     % update the next node (at the end of that trajectory)
% % %     nextNode = R.robotEdge(2,1);
% % %     
% % % end

% % % if nextDist > distRemaining
% % %     R.distAlongRobotEdge = R.distAlongRobotEdge + distRemaining;
% % %     %R.nextRobotPose= poseAtDistAlongEdge(R.robotEdge, R.distAlongRobotEdge);
% % %     
% % %     if isempty(node(R.robotNode).parent)
% % %         R.nextRobotPose=R.robotNode;
% % %     else
% % %         %calculate the next pose of the robot
% % %         ratioAlongEdge = R.distAlongRobotEdge/distNeighbMat(R.robotNode,node(R.robotNode).parent);
% % %         R.nextRobotPose=nodeList(1:2,R.robotNode)+ ratioAlongEdge*(nodeList(1:2,node(R.robotNode).parent) - nodeList(1:2,R.robotNode));
% % %         
% % %     end
% % %     
% % % else
% % %     % the next node is the end of this tree and we reach it
% % %     R.nextRobotPose = nodeList(1:2,nextNode);
% % %     R.distAlongRobotEdge= distNeighbMat(R.robotNode,node(R.robotNode).parent);
% % % end
% % %     
R.nextMoveTarget= R.robotEdge(2);
R.numLocalMovePoints =R.numLocalMovePoints + 1;
R.robotLocalPath(R.numLocalMovePoints,:) = R.nextRobotPose;


end






% if R.moving
%     R.robotPose=R.nextRobotPose;
%     increaseVideoMove=R.numLocalMovePoints;
%     R.robotMovePath(R.numRobotMovePoints+1:R.numRobotMovePoints+R.numLocalMovePoints,:) = R.robotLocalPath(1:R.numLocalMovePoints,:);
%     R.numRobotMovePoints = R.numRobotMovePoints + R.numLocalMovePoints;
%     
%     %save stuff for plotting
%     R.distAlongRobotEdgeForPlotting = R.distAlongRobotEdge;
%     R.robotEdgeForPlotting = R.robotEdge;
%     R.robotEdgeForPlottingUsed = true;
% else
%     % movement has just started, so remember that the robot is now moving
%     R.moving=true;
%     increaseVideoMove=2;
%     
%     if isempty(node(1).parent)
%         % no parent has been found for the node at the robot's position
%         R.currentMoveInvalid = true;
%     else
%         parentRobotNode=node(R.robotNode).parent;
%         R.robotEdge= [R.robotNode;parentRobotNode];
% 
%         R.robotEdgeForPlotting= [nodeList(1,R.robotNode),nodeList(1,parentRobotNode);nodeList(2,R.robotNode),nodeList(2,parentRobotNode)];
%         R.robotEdgeUsed = true;
%         R.robotEdgeForPlottingUsed = true;
%         
%         % Distance along robot Edge
%         
%         R.distAlongRobotEdge = 0.0;
%         R.distAlongRobotEdgeForPlotting = 0.0;        
%         
%     end
% end
% 
% %if the robot's current move target has been invalidated due to
% %dynamic obstacles then we need to attempt to find a new (safe) move target
% %NOTE we handle newly invalid moveTarget after moving the robot
% 
% if R.currentMoveInvalid
%     %findNewTarget(S, KD, R, hyberBallRad)
% end
% 
% % finally, we calculate the point to which the robot will move to in slice_time
% % and remember it for the next time this function is called. we also remember
% % all the nodes that it will visit along the way in the local path
% % and the part of the edge trajectory that takes the robot to the first local
% % point (the latter two things are used for visuialization)
% 
% % if !S.spaceHasTime
% 
% 
% nextNode= R.robotEdge(2);
% %parentRobotNode= node(R.robotNode).parent;
% %calculate distance from robot to the end of current edge it is following
% EdgeDistance= norm(nodeList(:,R.robotEdge(1))-nodeList(:,R.robotEdge(2)));
% nextDist = EdgeDistance - R.distAlongRobotEdge;
% 
% distRemaining = R.robotVelocity*slice_time ;
% 
% % save first local path point
% R.numLocalMovePoints = 1;
% R.robotLocalPath(R.numLocalMovePoints,:) = R.robotPose;
% 
% % starting at current location (and looking ahead to nextNode), follow parent
% % pointers back for the approperiate distance (or root or dead end)
% while (nextDist <= distRemaining && nextNode ~= 2 && ~isempty(node(nextNode).parent)&& nextNode ~= node(nextNode).parent)
% 
%     % can go all the way to nextNode and still have some distance left to spare
% 
%     % remember the robot will move through this point
%     R.numLocalMovePoints = R.numLocalMovePoints +1;
%     R.robotLocalPath(R.numLocalMovePoints,:) = nodeList(:,nextNode);
% 
%     %recalculate remaining distance
%     distRemaining = distRemaining- nextDist;
%     %reset distance along edge
%     R.distAlongRobotEdge = 0.0;
%     
%     %update trajectory that the robot will be in the middle of
%     R.robotEdge= [nextNode;node(nextNode).parent];
%     R.robotNode=nextNode;
%     
%     % calculate the dist of that trajectory
%     EdgeDistance= norm(nodeList(:,nextNode)-nodeList(:,node(nextNode).parent));
%     nextDist = EdgeDistance;
%     
%     % update the next node (at the end of that trajectory)
%     nextNode = R.robotEdge(2,1);
%     
% end
% 
% if nextDist > distRemaining
%     R.distAlongRobotEdge = R.distAlongRobotEdge + distRemaining;
%     %R.nextRobotPose= poseAtDistAlongEdge(R.robotEdge, R.distAlongRobotEdge);
%     
%     if isempty(node(R.robotNode).parent)
%         R.nextRobotPose=nodeList(:,R.robotNode)';
%     else
%         %calculate the next pose of the robot
%         ratioAlongEdge = R.distAlongRobotEdge/EdgeDistance;
%         R.nextRobotPose=nodeList(:,R.robotEdge(1))+ ratioAlongEdge*(nodeList(:,R.robotEdge(2)) - nodeList(:,R.robotEdge(1)));
%         
%     end
%     
% else
%     % the next node is the end of this tree and we reach it
%     R.nextRobotPose = nodeList(:,nextNode);
%     R.distAlongRobotEdge= distNeighbMat(R.robotEdge(1),R.robotEdge(2));
% end
%     
% R.nextMoveTarget= R.robotEdge(2);
% R.numLocalMovePoints =R.numLocalMovePoints + 1;
% R.robotLocalPath(R.numLocalMovePoints,:) = R.nextRobotPose;
% 
% 
% end