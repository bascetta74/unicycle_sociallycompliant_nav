function [collisionResult,thisEdgeCost]=OneEdgeFuturePredictCollision(startNode,finalNode,obsNumber,obstacle,obsTimePoint,CollisionTolerance,R)

% if Collision occurs, standard future prediction collision algorithm is
% applied with the part that defining the cost for the edge based on the
% possibility

[obsx,obsy]=boundary(obstacle(obsNumber).FuturePredictObject);
if isempty(obstacle(obsNumber).FuturePredictObject)
    obstacle(obsNumber).FuturePredictObject
end 
intersectedStartNode = isinterior(obstacle(obsNumber).FuturePredictObject, startNode(1,1), startNode(2,1)); % first checking if the correspoing new start node is inside of future prediction area
intersectX=polyxpoly([finalNode(1,1) startNode(1,1)],[finalNode(2,1) startNode(2,1)], obsx,obsy); % Or I can check if there is a conflict between the edge and future area
if intersectedStartNode~=1 && isempty(intersectX)
    collisionResult=0;
    thisEdgeCost=0;
    return
end

currentObsPos(1:2,1)=obstacle(obsNumber).Path(obsTimePoint,1:2);
%% if Collision occurs, standard future prediction collision algorithm is
% applied with the part that defining the cost for the edge based on the
% possibility

% Time and position initilization
addedObs= obstacle(obsNumber).Object;
[centx,centy]=centroid(addedObs);
theta=obstacle(obsNumber).Orientation(obsTimePoint);
rotMatInv=[cosd(-theta), -sind(-theta);sind(-theta), cosd(-theta)];
robotCurrentTime=R.robotPose(3);
obstacle(obsNumber).FuturePoint(3,1)=robotCurrentTime;
predictionWeight=1;
tolerance=1; % This is the tolerance to consider the size of robot and consider that robot speed can vary 

% Standard deviation polynominal function coefficients
a=obstacle(obsNumber).DeviationFunction(1);
b=obstacle(obsNumber).DeviationFunction(2);
c=obstacle(obsNumber).DeviationFunction(3);


nextTime=obsTimePoint+1; % Next time that the real position taken
if nextTime > numel(obstacle(obsNumber).Path(:,1))
     nextTime=numel(obstacle(obsNumber).Path(:,1));
end
obs01SecSpeed=norm(obstacle(obsNumber).Path(obsTimePoint,1:2)-obstacle(obsNumber).Path(nextTime,1:2))/0.2;
    
% Here the node positions in the golabal frame is taken
globPoppedPos=rotMatInv*(startNode(1:2,1)-[centx;centy]);
globNeighbPos=rotMatInv*(finalNode(1:2,1)-[centx;centy]);

startNodeTime=startNode(3,1);
endNodeTime=finalNode(3,1);
%we already know that startNodeTime>endNodeTime so 
%time that the robot takes to reach the 



    if globPoppedPos(1) >= globNeighbPos(1)
    timeIntervalPoint=[ ( globNeighbPos(1)-tolerance )/(10*obs01SecSpeed) , ( globPoppedPos(1)+tolerance )/(10*obs01SecSpeed) ];
    else
    timeIntervalPoint=[ ( globPoppedPos(1)-tolerance )/(10*obs01SecSpeed) , ( globNeighbPos(1)+tolerance )/(10*obs01SecSpeed) ];
    end
%     PointCollisionCheck=(robotCurrentTime - startNodeTime > timeIntervalPoint(1) && robotCurrentTime - startNodeTime < timeIntervalPoint(2) ) || (robotCurrentTime - endNodeTime > timeIntervalPoint(1) && robotCurrentTime - endNodeTime < timeIntervalPoint(2) );   
    PointCollisionCheck = 1;
    if PointCollisionCheck
         collisionResult=1;
          cost = 1/norm(R.robotPose(1:2)-currentObsPos(1:2,1));

         thisEdgeCost=(1+cost*predictionWeight)*edgeCost(startNode,finalNode);
    else
         collisionResult=0;
         thisEdgeCost=0;
    end


end

