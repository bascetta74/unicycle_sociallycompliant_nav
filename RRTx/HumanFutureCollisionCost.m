function [node,graphTree,obstacle,Q,QMatrix,orphanNodes,countOrphan,distNeighbMat]= HumanFutureCollisionCost(obstacle,obsNumber,Q,QMatrix,...
    R,node,nodeList,graphTree,orphanNodes,countOrphan,totalNodeNum,distNeighbMat,param)
load('probGMMsegm')
% Here, I make collision check with future prediction area of human. 
% I set to cost value for the conflicted edges based on defined probabilisitic collision function
% I try to use a rectancgle to define the obstacle area of the high probability
% future positions. This square has one center, around this center I find
% the near Nodes by compparing their distances and their time values

obsTimePoint=obstacle(obsNumber).ObsTimePoint; 
centerOriginFutereSquare=[mean(obstacle(obsNumber).futureXPos);mean(obstacle(obsNumber).futureYPos)];
addedObs= obstacle(obsNumber).Object;
% [obsx,obsy]=boundary(addedObs);
theta=obstacle(obsNumber).Orientation(obsTimePoint);
rotMat=[cosd(theta), -sind(theta);sind(theta), cosd(theta)];
rotMatInv=[cosd(-theta), -sind(-theta);sind(-theta), cosd(-theta)];
[centx,centy]=centroid(addedObs);
currentObsPos(1:2,1)=obstacle(obsNumber).Path(obsTimePoint,1:2);
obstacle(obsNumber).FuturePoint= rotMat*centerOriginFutereSquare + currentObsPos;
% robotCurrentTime=R.robotMovePath(imove,3); % this is the current time value of Robot
robotCurrentTime=R.robotPose(3);
obstacle(obsNumber).FuturePoint(3,1)=robotCurrentTime;
predictionWeight=1;
tolerance=param.colTolerance; % This is the tolerance to consider the size of robot and consider that robot speed can vary 


% rotation and translation should be done for future obstacle poisition
% obstacle area
% [centFutureX,centFutureY]=centroid(obstacle(ObsNum).FutureObject);
tempFutureObs=rotate(obstacle(obsNumber).FutureObject,theta);
futurePredictObject=translate(tempFutureObs,currentObsPos(1),currentObsPos(2));
obstacle(obsNumber).FuturePredictObject=futurePredictObject;
[obsxFuture,obsyFuture]=boundary(futurePredictObject);

%finding the nodes near to future possible obstacle area
allQueryNodes=findPointsInConflict(obsNumber,obstacle,nodeList,param.r,totalNodeNum);

%Questa parte non mi serve!!
% ignoring the nodes with a higher time value(because it is impossible to reach there for the robot)
% allQueryNodes(nodeList(3,allQueryNodes)>robotCurrentTime)=[];
tempAllQuery=allQueryNodes;
% Checking if the robot can reach that point within kinematic constraints
checkKinem=zeros(1,numel(allQueryNodes));
for iq=1:numel(tempAllQuery)
    checkKinem(iq)=norm(nodeList(1:2,tempAllQuery(iq))- R.robotPose(1:2)')/( R.robotPose(3) - nodeList(3,tempAllQuery(iq)) );
end
allQueryNodes(checkKinem>2)=[];

queryGvLmc=[node(allQueryNodes).Gv;node(allQueryNodes).Lmc];
minObsNodesKey= min(queryGvLmc,[],1);

% the border of the future prediction area is determined to make collision
% check easier
XInitPredictArea=obstacle(obsNumber).futureXPos(1);
XLastPredictArea=obstacle(obsNumber).futureXPos(end);
YInitPredictArea=obstacle(obsNumber).futureYPos(end-1);
YLastPredictArea=obstacle(obsNumber).futureYPos(end);

QAddObsNode= MinHeap(2000,minObsNodesKey);
QAddObsMatrix = [allQueryNodes;minObsNodesKey;node(allQueryNodes).Gv];

% Grid variables
% xgrid=obstacle(obsNumber).LimitGrids(1:2);
% ygrid=obstacle(obsNumber).LimitGrids(3:4);
% gridSize=obstacle(obsNumber).LimitGrids(5);

% Standard deviation polynominal function coefficients
a=obstacle(obsNumber).DeviationFunction(1);
b=obstacle(obsNumber).DeviationFunction(2);
c=obstacle(obsNumber).DeviationFunction(3);
 
nextTime=obsTimePoint+1; % Next time that the real position taken
if nextTime > numel(obstacle(obsNumber).Path(:,1))
     nextTime=numel(obstacle(obsNumber).Path(:,1));
end
obs02SecSpeed=norm(obstacle(obsNumber).Path(obsTimePoint,1:2)-obstacle(obsNumber).Path(nextTime,1:2))/0.2;
    
% lastFutureObsPoint=obstacle(obsNumber).futureXPos(1,end); % the farest x predicition point
% timeToReachFarestPredictPoint=(lastFutureObsPoint-0.5)/(10*obs01SecSpeed);

while QAddObsNode.Count >0
    poppedNodeLmc= QAddObsNode.ExtractMin;
    poppedNodeLmc= poppedNodeLmc(1);
    poppedNodeColumn= find(QAddObsMatrix(2,1:end)==poppedNodeLmc);
    poppedNodeColumn = poppedNodeColumn(1);
    poppedNodeIdx= QAddObsMatrix(1,poppedNodeColumn);
    QAddObsMatrix(:,poppedNodeColumn)=[];
    prevParentNodeDist=distNeighbMat(poppedNodeIdx,node(poppedNodeIdx).parent);
    
    % The position of PoppedNodeInd in the golabal frame for determining
    % weight cost with respect the global frame
    globPoppedPos=rotMatInv*(nodeList(1:2,poppedNodeIdx)-[centx;centy]);    
% % %IMPORTANT: Prediction of time that obstacle can reach to that popped Node position. 0.3 meters is added for considering the size of obstacle 0.2 meter + 0.1 safety 
%     instantTimePop=globPoppedPos(1)/(10*obs01SecSpeed);
%     timeIntervalPop=[instantTimePop-0.7,instantTimePop+0.7];
%     timeIntervalPop=[( globPoppedPos(1)- tolerance )/(10*obs01SecSpeed) , (globPoppedPos(1)+tolerance)/(10*obs01SecSpeed)];
    
  % checking poppedNode whether is inside of possible future obstacle area  
    popInside=false;
    if isinterior(futurePredictObject, nodeList(1,poppedNodeIdx), nodeList(2,poppedNodeIdx))
        popInside=true;
    end
    
    isThisRootNode=false; % This is for preventing the operation for the root nodes. If this operation was made, after removing obstacle function, the cost between the root node won't be zero as supposed to be our system rquirements.
    
    if nodeList(1:2,poppedNodeIdx)==nodeList(1:2,2)
       isThisRootNode=true; 
    end
    
    
    poppedNodeNeighb= [node(poppedNodeIdx).RadiusOutConnect, node(poppedNodeIdx).InitialOutConnect];
    for i = 1:numel(poppedNodeNeighb)
        nextNeighb= poppedNodeNeighb(i);
        
        % if the connection between two root nodes is checked. Skip this
        % one because it has to be zero always
        if isThisRootNode==1 && nodeList(1,nextNeighb)==nodeList(1,2) && nodeList(2,nextNeighb)==nodeList(2,2)% Root node connection cost remains always zero by this way
            continue
        end
        
                % rota translation of the neighbour node
        globNeighbPos=rotMatInv*(nodeList(1:2,nextNeighb)-[centx;centy]);
        
        % if the position of poppedNode node and neighbor is in negaive
        % side x direction, this edge should be not conflicted so it is discarded
        if (globNeighbPos(1) < XInitPredictArea && globPoppedPos (1) < XInitPredictArea) || (globNeighbPos(1) > XLastPredictArea && globPoppedPos (1) > XLastPredictArea)...
                || (globNeighbPos(2) < YInitPredictArea && globPoppedPos (2) < YInitPredictArea) || (globNeighbPos(2) > YLastPredictArea && globPoppedPos (2) > YLastPredictArea)
            continue
        end
        
        [collisionPredictArea,~]=polyxpoly([nodeList(1,poppedNodeIdx) nodeList(1,nextNeighb)],[nodeList(2,poppedNodeIdx) nodeList(2,nextNeighb)], obsxFuture,obsyFuture);
        if isempty(collisionPredictArea) && ~ popInside
            continue
        end
        
%         timeIntervalNeighb=[( globNeighbPos(1)- tolerance )/(10*obs01SecSpeed) , (globNeighbPos(1)+tolerance)/(10*obs01SecSpeed)];
        
        
        neighbInside=false;
        if isinterior(futurePredictObject, nodeList(1,nextNeighb), nodeList(2,nextNeighb))
            neighbInside=true;
        end
%         intersectXFut=false;

% If the popNode and its Neighbour are both inside of the future predict area
        if popInside && neighbInside

            startNodeTime=nodeList(3,poppedNodeIdx);
            endNodeTime=nodeList(3,nextNeighb);
            startNodeTime1=robotCurrentTime+norm(R.robotPose(1:2) - nodeList(1:2,poppedNodeIdx))/0.55;
            endNodeTime1=robotCurrentTime+norm(R.robotPose(1:2) - nodeList(1:2,nextNeighb))/0.55;
                       
            collisionOccurs=0;
                
%     if globPoppedPos(1) >= globNeighbPos(1)
%     timeIntervalPoint=[ ( globNeighbPos(1)-tolerance )/(10*obs02SecSpeed) , ( globPoppedPos(1)+tolerance )/(10*obs02SecSpeed) ];
%     else
%     timeIntervalPoint=[ ( globPoppedPos(1)-tolerance )/(10*obs02SecSpeed) , ( globNeighbPos(1)+tolerance )/(10*obs02SecSpeed) ];
%     end
    obstacleStartNodeTime = [robotCurrentTime+norm(currentObsPos - nodeList(1:2,poppedNodeIdx))/obs02SecSpeed-tolerance, robotCurrentTime+norm(currentObsPos - nodeList(1:2,poppedNodeIdx))/obs02SecSpeed +tolerance] ;
    obstacleEndNodeTime = [robotCurrentTime+norm(currentObsPos - nodeList(1:2,nextNeighb))/obs02SecSpeed-tolerance, robotCurrentTime+norm(currentObsPos - nodeList(1:2,nextNeighb))/obs02SecSpeed+tolerance];
    PointCollisionCheck =1;
%     if (startNodeTime1> obstacleStartNodeTime(1) && startNodeTime1 < obstacleStartNodeTime(2) || endNodeTime1> obstacleEndNodeTime(1) && endNodeTime1 < obstacleEndNodeTime(2) )
%         PointCollisionCheck = 1;
%     end
%     PointCollisionCheck=(robotCurrentTime - startNodeTime1 > timeIntervalPoint(1) && robotCurrentTime - startNodeTime1 < timeIntervalPoint(2) )|| (robotCurrentTime - endNodeTime1 > timeIntervalPoint(1) && robotCurrentTime - endNodeTime1 < timeIntervalPoint(2) );   

                if PointCollisionCheck 
                    
                    collisionOccurs=1;
         
%          cost=feval(probGMMsegm,globPoppedPos(1) ,globNeighbPos(1),globPoppedPos(2),globNeighbPos(2));
         cost = 1/norm(R.robotPose(1:2)-currentObsPos(1:2,1));
         distNeighbMat(poppedNodeIdx,nextNeighb)=(1+cost*predictionWeight)*edgeCost(nodeList(:,poppedNodeIdx),nodeList(:,nextNeighb)); 
          
                               
                end
          
 
        elseif popInside && ~neighbInside % The case when popped Node is inside but its outhoing neigbour is outisde of predicted area
            
            % Collision Check for popped Node since it is inside(poppedNodeCollisionCheck) and for the point interesction of the edge with predictedArea(intersectPointCollisionCheck). The collision check for theis neigbour of poppedNode is not neccesary since it is out of predicted area  
%             PoppedNodeCollisionCheck=~(robotCurrentTime - nodeList(3,poppedNodeIdx) > timeIntervalPop(1) && robotCurrentTime - nodeList(3,poppedNodeIdx) < timeIntervalPop(2) );
            %First I need to determine the X global loaction of interesction point by ratio and propartion operation
            ratioIntersectPos=(nodeList(1,poppedNodeIdx)-collisionPredictArea(1))/(nodeList(1,poppedNodeIdx)-nodeList(1,nextNeighb));
            IntersectTime=nodeList(3,poppedNodeIdx)-ratioIntersectPos*( nodeList(3,poppedNodeIdx)-nodeList(3,nextNeighb) );

            startNodeTime=nodeList(3,poppedNodeIdx);
            endNodeTime=nodeList(3,nextNeighb);
            startNodeTime1=robotCurrentTime+norm(R.robotPose(1:2) - nodeList(1:2,poppedNodeIdx))/0.55;
            endNodeTime1=robotCurrentTime+norm(R.robotPose(1:2) - nodeList(1:2,nextNeighb))/0.55;
                       
    
            
%             colIntervalX=obstacle(obsNumber).ProbabilityDistribution(:,1); % colIntervalX on the edge is the points that collision checking is made for the step points in the future predcited area 
%             colIntervalX(colIntervalX<endXPos | colIntervalX>startXPos)=[];
%             collisionOccurs=0;
%             if globPoppedPos(1) >= globNeighbPos(1)
%     timeIntervalPoint=[ ( globNeighbPos(1)-tolerance )/(10*obs02SecSpeed) , ( globPoppedPos(1)+tolerance )/(10*obs02SecSpeed) ];
%     else
%     timeIntervalPoint=[ ( globPoppedPos(1)-tolerance )/(10*obs02SecSpeed) , ( globNeighbPos(1)+tolerance )/(10*obs02SecSpeed) ];
%     end
%     PointCollisionCheck=(robotCurrentTime - startNodeTime > timeIntervalPoint(1) && robotCurrentTime - startNodeTime < timeIntervalPoint(2) )|| (robotCurrentTime - endNodeTime > timeIntervalPoint(1) && robotCurrentTime - endNodeTime < timeIntervalPoint(2) );   
%       
            obstacleStartNodeTime = [robotCurrentTime+norm(currentObsPos - nodeList(1:2,poppedNodeIdx))/obs02SecSpeed-tolerance, robotCurrentTime+norm(currentObsPos - nodeList(1:2,poppedNodeIdx))/obs02SecSpeed +tolerance] ;
            obstacleEndNodeTime = [robotCurrentTime+norm(currentObsPos - nodeList(1:2,nextNeighb))/obs02SecSpeed-tolerance, robotCurrentTime+norm(currentObsPos - nodeList(1:2,nextNeighb))/obs02SecSpeed+tolerance];
            if (startNodeTime1> obstacleStartNodeTime(1) && startNodeTime1 < obstacleStartNodeTime(2) || endNodeTime1> obstacleEndNodeTime(1) && endNodeTime1 < obstacleEndNodeTime(2) )
                PointCollisionCheck = 1;
            end
            PointCollisionCheck = 1;
                if PointCollisionCheck 
                   
                    collisionOccurs=1;
                    
%         cost=feval(probGMMsegm,globPoppedPos(1) ,globNeighbPos(1),globPoppedPos(2),globNeighbPos(2));
        cost = 1/norm(R.robotPose(1:2)-currentObsPos(1:2,1));
        distNeighbMat(poppedNodeIdx,nextNeighb)=(1+cost*predictionWeight)*edgeCost(nodeList(:,poppedNodeIdx),nodeList(:,nextNeighb)); 
          
                                  
                end
            
               
  
        elseif neighbInside && ~popInside
            
            %First I need to determine the X global loaction of interesction point by ratio and propartion operation
            ratioIntersectPos=(nodeList(1,poppedNodeIdx)-collisionPredictArea(1))/(nodeList(1,poppedNodeIdx)-nodeList(1,nextNeighb));
%             globIntersectpos=globPoppedPos-ratioIntersectPos*(globNeighbPos-globPoppedPos);
            IntersectTime=nodeList(3,poppedNodeIdx)-ratioIntersectPos*( nodeList(3,poppedNodeIdx)-nodeList(3,nextNeighb) );


            startNodeTime=nodeList(3,poppedNodeIdx);
            endNodeTime=nodeList(3,nextNeighb);
  
            
            collisionOccurs=0;
               
            if globPoppedPos(1) >= globNeighbPos(1)
    timeIntervalPoint=[ ( globNeighbPos(1)-tolerance )/(10*obs02SecSpeed) , ( globPoppedPos(1)+tolerance )/(10*obs02SecSpeed) ];
    else
    timeIntervalPoint=[ ( globPoppedPos(1)-tolerance )/(10*obs02SecSpeed) , ( globNeighbPos(1)+tolerance )/(10*obs02SecSpeed) ];
    end
    PointCollisionCheck=(robotCurrentTime - startNodeTime > timeIntervalPoint(1) && robotCurrentTime - startNodeTime < timeIntervalPoint(2) )|| (robotCurrentTime - endNodeTime > timeIntervalPoint(1) && robotCurrentTime - endNodeTime < timeIntervalPoint(2) );   
    PointCollisionCheck=1;

                if PointCollisionCheck 
                  
                    collisionOccurs=1;
                   
%           cost=feval(probGMMsegm,globPoppedPos(1) ,globNeighbPos(1),globPoppedPos(2),globNeighbPos(2));
         cost = 1/norm(R.robotPose(1:2)-currentObsPos(1:2,1));
         distNeighbMat(poppedNodeIdx,nextNeighb)=(1+cost*predictionWeight)*edgeCost(nodeList(:,poppedNodeIdx),nodeList(:,nextNeighb)); 
          
                                  
                end
            
  
       
        elseif ~isempty(collisionPredictArea)
            
            %First I need to determine the X global loaction of interesction point by ratio and propartion operation
            ratioIntersectPos1=(nodeList(1,poppedNodeIdx)-collisionPredictArea(1))/(nodeList(1,poppedNodeIdx)-nodeList(1,nextNeighb));
%             globIntersectpos=globPoppedPos-ratioIntersectPos*(globNeighbPos-globPoppedPos);
            IntersectTime1=nodeList(3,poppedNodeIdx)-ratioIntersectPos1*( nodeList(3,poppedNodeIdx)-nodeList(3,nextNeighb) );

            ratioIntersectPos2=(nodeList(1,poppedNodeIdx)-collisionPredictArea(2))/(nodeList(1,poppedNodeIdx)-nodeList(1,nextNeighb));
%             globIntersectpos=globPoppedPos-ratioIntersectPos*(globNeighbPos-globPoppedPos);
            IntersectTime2=nodeList(3,poppedNodeIdx)-ratioIntersectPos2*( nodeList(3,poppedNodeIdx)-nodeList(3,nextNeighb) );
            
            
            startNodeTime=nodeList(3,poppedNodeIdx);
            endNodeTime=nodeList(3,nextNeighb);
   
            if globPoppedPos(1) >= globNeighbPos(1)
    timeIntervalPoint=[ ( globNeighbPos(1)-tolerance )/(10*obs02SecSpeed) , ( globPoppedPos(1)+tolerance )/(10*obs02SecSpeed) ];
    else
    timeIntervalPoint=[ ( globPoppedPos(1)-tolerance )/(10*obs02SecSpeed) , ( globNeighbPos(1)+tolerance )/(10*obs02SecSpeed) ];
    end
            if IntersectTime1 > IntersectTime2
                PointCollisionCheck=(robotCurrentTime - IntersectTime1 > timeIntervalPoint(1) && robotCurrentTime - IntersectTime2 < timeIntervalPoint(2) )|| (robotCurrentTime - endNodeTime > timeIntervalPoint(1) && robotCurrentTime - endNodeTime < timeIntervalPoint(2) );   
    
            else
                PointCollisionCheck=(robotCurrentTime - IntersectTime2 > timeIntervalPoint(1) && robotCurrentTime - IntersectTime1 < timeIntervalPoint(2) )|| (robotCurrentTime - endNodeTime > timeIntervalPoint(1) && robotCurrentTime - endNodeTime < timeIntervalPoint(2) );   
    
            end
            
%             colIntervalX=obstacle(obsNumber).ProbabilityDistribution(:,1); % colIntervalX on the edge is the points that collision checking is made for the step points in the future predcited area 
%             colIntervalX(colIntervalX<endXPos | colIntervalX>startXPos)=[];
            collisionOccurs=0;
            PointCollisionCheck=1;
            
                if PointCollisionCheck
                    
                    collisionOccurs=1;
                    
%          cost=feval(probGMMsegm,globPoppedPos(1) ,globNeighbPos(1),globPoppedPos(2),globNeighbPos(2));
         cost = 1/norm(R.robotPose(1:2)-currentObsPos(1:2,1));
         distNeighbMat(poppedNodeIdx,nextNeighb)=(1+cost*predictionWeight)*edgeCost(nodeList(:,poppedNodeIdx),nodeList(:,nextNeighb)); 
                end
                                                
     
              
        

    end
    
    end
    parentThisNode=node(poppedNodeIdx).parent;
    makeMarkedQOperation=false;
    if ~isempty(parentThisNode) && prevParentNodeDist~=distNeighbMat(poppedNodeIdx,node(poppedNodeIdx).parent)
        graphTree = rmedge(graphTree,{int2str(poppedNodeIdx)}, {int2str(node(poppedNodeIdx).parent)}); % removing old parent relation
        graphTree = addedge(graphTree, {int2str(poppedNodeIdx)},{int2str(node(poppedNodeIdx).parent)},distNeighbMat(poppedNodeIdx,node(poppedNodeIdx).parent));
        makeMarkedQOperation=true;
    end

    if node(poppedNodeIdx).MarkedQOB ~=true && makeMarkedQOperation == 1
        node(poppedNodeIdx).MarkedQOB =true;
        countOrphan= countOrphan +1;
        orphanNodes(:,countOrphan)= [poppedNodeIdx;min(node(poppedNodeIdx).Gv,node(poppedNodeIdx).Lmc)];        
    end
    

end

end
