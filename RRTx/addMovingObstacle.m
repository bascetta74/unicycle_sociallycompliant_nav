function [node,futurePredictObject,graphTree,obstacle,Q,QMatrix,orphanNodes,countOrphan,distNeighbMat]= addMovingObstacle(obstacle,obsNumber,Q,QMatrix,obsTimePoint,imove,R,node,nodeList,graphTree,orphanNodes,countOrphan,r,totalNodeNum,distNeighbMat)
% QAddObsNode= MinHeap(250);
% QAddObsMatrix=[];
% I try to use a rectancgle to define the obstacle area of the high probability
% future positions. This square has one center, around this center I find
% the near Nodes by compparing their distances and their time values
centerOriginFutereSquare=[mean(obstacle(obsNumber).futureXPos);mean(obstacle(obsNumber).futureYPos)];
addedObs= obstacle(obsNumber).Object;
[obsx,obsy]=boundary(addedObs);
theta=obstacle(obsNumber).Orientation(obsTimePoint);
rotMat=[cosd(theta), -sind(theta);sind(theta), cosd(theta)];
rotMatInv=[cosd(-theta), -sind(-theta);sind(-theta), cosd(-theta)];
[centx,centy]=centroid(addedObs);
currentObsPos(1:2,1)=obstacle(obsNumber).Path(1:2,obsTimePoint);
obstacle(obsNumber).FuturePoint= rotMat*centerOriginFutereSquare + currentObsPos;
robotCurrentTime=R.robotMovePath(imove-2,3); % this is the current time value of Robot, based on this robot shouldn't pass 1.8 seconds later in the center of this future rectangel obs area
obstacle(obsNumber).FuturePoint(3,1)=robotCurrentTime;

% rotation and translation should be done for future obstacle poisition
% obstacle area
% [centFutureX,centFutureY]=centroid(obstacle(ObsNum).FutureObject);
tempFutureObs=rotate(obstacle(obsNumber).FutureObject,theta);
futurePredictObject=translate(tempFutureObs,currentObsPos(1),currentObsPos(2));
[obsxFuture,obsyFuture]=boundary(futurePredictObject);

%finding the nodes near to future possible obstacle area
allQueryNodes=findPointsInConflict(obsNumber,obstacle,nodeList,r,totalNodeNum);
% ignoring the nodes with a higher time value(because it is impossible to reach there for the robot)
allQueryNodes(nodeList(3,allQueryNodes)>robotCurrentTime+0.5)=[];
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
xgrid=obstacle(obsNumber).LimitGrids(1:2);
ygrid=obstacle(obsNumber).LimitGrids(3:4);
gridSize=obstacle(obsNumber).LimitGrids(5);

% here obs01SecSpeed is the average speed of the obstacle
obs01SecSpeed=norm(obstacle(obsNumber).Path(1:2,obsTimePoint)-obstacle(obsNumber).Path(1:2,obsTimePoint-1));
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
    % weight cost with respect to grid system in global frame
    globPoppedPos=rotMatInv*(nodeList(1:2,poppedNodeIdx)-[centx;centy]);    

% % %IMPORTANT: Prediction of time that obstacle can reach to that popped Node position. 0.3 meters is added for considering the size of obstacle 0.2 meter + 0.1 safety 
%     instantTimePop=globPoppedPos(1)/(10*obs01SecSpeed);
%     timeIntervalPop=[instantTimePop-0.7,instantTimePop+0.7];
    timeIntervalPop=[( globPoppedPos(1)- 0.3 )/(10*obs01SecSpeed) , (globPoppedPos(1)+0.3)/(10*obs01SecSpeed)];
    
  % checking poppedNode whether is inside of possible future obstacle area  
    popInside=false;
    if isinterior(futurePredictObject, nodeList(1,poppedNodeIdx), nodeList(2,poppedNodeIdx))
        popInside=true;
    end
    
    if xgrid(1) < globPoppedPos(1) && globPoppedPos(1) < xgrid(end) && ygrid(1)< globPoppedPos(2) && globPoppedPos(2) < ygrid(end)
     % if the node position inside of our weighted obstacle figur, here it
     % gives a weighted value based on the position in the grid
        matrixXColumnNum=ceil((globPoppedPos(1)-xgrid(1))/gridSize);
        matrixYColumnNum=ceil((globPoppedPos(2)-ygrid(1))/gridSize);
        coefficientPopped= obstacle(obsNumber).WeightMatrix(matrixYColumnNum,matrixXColumnNum);
        poppedNWeight=30*coefficientPopped;
    else
        poppedNWeight=4;
    end
    
    poppedNodeNeighb= [node(poppedNodeIdx).RadiusOutConnect, node(poppedNodeIdx).InitialOutConnect];
    for i = 1:numel(poppedNodeNeighb)
        nextNeighb= poppedNodeNeighb(i);
        
                % rota translation of the neighbour node
        globNeighbPos=rotMatInv*(nodeList(1:2,nextNeighb)-[centx;centy]);
        
        % if the position of poppedNode node and neighbor is in negaive
        % side x direction, this edge should be not conflicted so it is discarded
        if (globNeighbPos(1) < XInitPredictArea && globPoppedPos (1) < XInitPredictArea) || (globNeighbPos(1) > XLastPredictArea && globPoppedPos (1) > XLastPredictArea)...
                || (globNeighbPos(2) < YInitPredictArea && globPoppedPos (2) < YInitPredictArea) || (globNeighbPos(2) > YLastPredictArea && globPoppedPos (2) > YLastPredictArea)
            continue
        end
        
        [collisionPredictArea,yy]=polyxpoly([nodeList(1,poppedNodeIdx) nodeList(1,nextNeighb)],[nodeList(2,poppedNodeIdx) nodeList(2,nextNeighb)], obsxFuture,obsyFuture);
        if isempty(collisionPredictArea) && ~ popInside
            continue
        end
        
        % first check if this edge collides with obstacle at current time
        if isinterior(addedObs, nodeList(1,nextNeighb), nodeList(2,nextNeighb))
            distNeighbMat(poppedNodeIdx,nextNeighb)=inf;
            continue
        end
        intersectX = polyxpoly([nodeList(1,nextNeighb) nodeList(1,poppedNodeIdx)],[nodeList(2,nextNeighb) nodeList(2,poppedNodeIdx)], obsx,obsy);
        if ~isempty(intersectX)
            distNeighbMat(poppedNodeIdx,nextNeighb)=inf;
            continue
        end
        
% % %IMPORTANT:Prediction of time which obstacle can reach to that Neighbor node position. 0.3 meters is added for considering the size of obstacle 0.2 meter + 0.1 safety 
%         instantTimeNeighb=[(globNeighbPos(1)-0.3)/(10*obs01SecSpeed) , (globNeighbPos(1)+0.3)/(10*obs01SecSpeed)];
%         timeIntervalNeighb=[instantTimeNeighb-0.7,instantTimeNeighb+0.7];
        timeIntervalNeighb=[( globNeighbPos(1)- 0.3 )/(10*obs01SecSpeed) , (globNeighbPos(1)+0.3)/(10*obs01SecSpeed)];
        
        if xgrid(1) < globNeighbPos(1) && globNeighbPos(1) < xgrid(end) && ygrid(1)< globNeighbPos(2) && globNeighbPos(2) < ygrid(end)
         % if the node position inside of our weighted obstacle figure
            matrixNeighXColumnNum=ceil((globNeighbPos(1)-xgrid(1))/gridSize);
            matrixNeighYColumnNum=ceil((globNeighbPos(2)-ygrid(1))/gridSize);
            coefficientPopped= obstacle(obsNumber).WeightMatrix(matrixNeighYColumnNum,matrixNeighXColumnNum);
            NeighbNWeight=30*coefficientPopped;
        else
            NeighbNWeight=4;
        end
        
        neighbInside=false;
        if isinterior(futurePredictObject, nodeList(1,nextNeighb), nodeList(2,nextNeighb))
            neighbInside=true;
        end
%         intersectXFut=false;
        if popInside && neighbInside
%             intersectXFut=true;
            if ~(robotCurrentTime -nodeList(3,nextNeighb) > timeIntervalNeighb(1) && robotCurrentTime - nodeList(3,nextNeighb) < timeIntervalNeighb(2) )...
                    || ~(robotCurrentTime - nodeList(3,poppedNodeIdx) > timeIntervalPop(1) && robotCurrentTime - nodeList(3,poppedNodeIdx) < timeIntervalPop(2) )
                continue
            end
            distNeighbMat(poppedNodeIdx,nextNeighb) = ((NeighbNWeight+poppedNWeight)/2)*norm(nodeList(:,poppedNodeIdx)-nodeList(:,nextNeighb));
        
        elseif popInside && ~neighbInside % The case when popped Node is inside but its outhoing neigbour is outisde of predicted area
            % Collision Check for popped Node since it is inside(poppedNodeCollisionCheck) and for the point interesction of the edge with predictedArea(intersectPointCollisionCheck). The collision check for theis neigbour of poppedNode is not neccesary since it is out of predicted area  
            PoppedNodeCollisionCheck=~(robotCurrentTime - nodeList(3,poppedNodeIdx) > timeIntervalPop(1) && robotCurrentTime - nodeList(3,poppedNodeIdx) < timeIntervalPop(2) );
            %First I need to determine the X global loaction of interesction point
            ratioIntersectPos=(norm(nodeList(1:2,poppedNodeIdx)-[collisionPredictArea(1);yy(1)]))/(norm(nodeList(1:2,poppedNodeIdx)-nodeList(1:2,nextNeighb)));
            globIntersectXpos=globPoppedPos(1)+ratioIntersectPos*(globNeighbPos(1)-globPoppedPos(1));
            timeIntervalIntersect=[( globIntersectXpos- 0.3 )/(10*obs01SecSpeed) , (globIntersectXpos + 0.3)/(10*obs01SecSpeed)];
            timeRobotReachIntersectPoint=nodeList(3,poppedNodeIdx)-ratioIntersectPos*(nodeList(3,poppedNodeIdx)-nodeList(3,nextNeighb));
            intersectPointCollisionCheck=~(robotCurrentTime - timeRobotReachIntersectPoint > timeIntervalIntersect(1) && robotCurrentTime - timeRobotReachIntersectPoint < timeIntervalIntersect(2) );
            if PoppedNodeCollisionCheck && intersectPointCollisionCheck
                continue
            end
            distNeighbMat(poppedNodeIdx,nextNeighb) = ((NeighbNWeight+poppedNWeight)/2)*norm(nodeList(:,poppedNodeIdx)-nodeList(:,nextNeighb));
        
        elseif neighbInside && ~popInside
            % Collision Check for outgoing Neighbour Node since it is inside(NeighnNodeCollisionCheck) and for the point interesction of the edge with predictedArea(intersectPointCollisionCheck). The collision check for the popped node is not neccesary since it is out of predicted area  
            NeighnNodeCollisionCheck=~(robotCurrentTime - nodeList(3,nextNeighb) > timeIntervalNeighb(1) && robotCurrentTime - nodeList(3,nextNeighb) < timeIntervalNeighb(2) );
            %First I need to determine the X global loaction of interesction point
            ratioIntersectPos=(norm(nodeList(1:2,poppedNodeIdx)-[collisionPredictArea(1);yy(1)]))/(norm(nodeList(1:2,poppedNodeIdx)-nodeList(1:2,nextNeighb)));
            globIntersectXpos=globPoppedPos(1)+ratioIntersectPos*(globNeighbPos(1)-globPoppedPos(1));
            timeIntervalIntersect=[( globIntersectXpos- 0.3 )/(10*obs01SecSpeed) , (globIntersectXpos + 0.3)/(10*obs01SecSpeed)];
            timeRobotReachIntersectPoint=nodeList(3,poppedNodeIdx)-ratioIntersectPos*(nodeList(3,poppedNodeIdx)-nodeList(3,nextNeighb));
            intersectPointCollisionCheck=~(robotCurrentTime - timeRobotReachIntersectPoint > timeIntervalIntersect(1) && robotCurrentTime - timeRobotReachIntersectPoint < timeIntervalIntersect(2) );
            if NeighnNodeCollisionCheck && intersectPointCollisionCheck
                continue
            end
            distNeighbMat(poppedNodeIdx,nextNeighb) = ((NeighbNWeight+poppedNWeight)/2)*norm(nodeList(:,poppedNodeIdx)-nodeList(:,nextNeighb));

        else
%             [xx,yy]=polyxpoly([nodeList(1,poppedNodeIdx) nodeList(1,nextNeighb)],[nodeList(2,poppedNodeIdx) nodeList(2,nextNeighb)], obsxFuture,obsyFuture);
            if ~isempty(collisionPredictArea)
    %           intersectXFut=true;
                % Since two nodes aren't inside the future box, I do check
                % collision in the three points in the edge; middle point,
                % 2 intersection points
                midPointTime=( nodeList(3,nextNeighb) + nodeList(3,poppedNodeIdx) ) / 2;
                midPontXDistance=(globPoppedPos(1)+globNeighbPos(1))/2; % The distance of mid point in the x axis
%                 instantTimeMidPooint=(globPoppedPos(1)+globNeighbPos(1))/2;
                timeIntervalMidpoint=[(midPontXDistance-0.3)/(10*obs01SecSpeed),(midPontXDistance+0.3)/(10*obs01SecSpeed)];
                midCheck=~(robotCurrentTime - midPointTime > timeIntervalMidpoint(1) && robotCurrentTime - midPointTime < timeIntervalMidpoint(2) );
                %I am checking for the two intersection points.This is the first one
                %First I need to determine the X global location of the first interesction point
                ratioIntersect1Pos=(norm(nodeList(1:2,poppedNodeIdx)-[collisionPredictArea(1);yy(1)]))/(norm(nodeList(1:2,poppedNodeIdx)-nodeList(1:2,nextNeighb)));
                globIntersectXpos=globPoppedPos(1)+ratioIntersect1Pos*(globNeighbPos(1)-globPoppedPos(1));
                timeIntervalIntersect1=[( globIntersectXpos- 0.3 )/(10*obs01SecSpeed) , (globIntersectXpos + 0.3)/(10*obs01SecSpeed)];
                timeRobotReachIntersectPoint1=nodeList(3,poppedNodeIdx)-ratioIntersect1Pos*(nodeList(3,poppedNodeIdx)-nodeList(3,nextNeighb));
                intersect1check=~(robotCurrentTime - timeRobotReachIntersectPoint1 > timeIntervalIntersect1(1) && robotCurrentTime - timeRobotReachIntersectPoint1 < timeIntervalIntersect1(2) );
                % Second Intersection Point
                ratioIntersect2Pos=(norm(nodeList(1:2,poppedNodeIdx)-[collisionPredictArea(2);yy(2)]))/(norm(nodeList(1:2,poppedNodeIdx)-nodeList(1:2,nextNeighb)));
                globIntersectXpos=globPoppedPos(1)+ratioIntersect2Pos*(globNeighbPos(1)-globPoppedPos(1));
                timeIntervalIntersect2=[( globIntersectXpos- 0.3 )/(10*obs01SecSpeed) , (globIntersectXpos + 0.3)/(10*obs01SecSpeed)];
                timeRobotReachIntersectPoint2=nodeList(3,poppedNodeIdx)-ratioIntersect2Pos*(nodeList(3,poppedNodeIdx)-nodeList(3,nextNeighb));
                intersect2check=~(robotCurrentTime - timeRobotReachIntersectPoint2 > timeIntervalIntersect2(1) && robotCurrentTime - timeRobotReachIntersectPoint2 < timeIntervalIntersect2(2) );
                
                if midCheck && intersect1check && intersect2check
                    continue
                end
                distNeighbMat(poppedNodeIdx,nextNeighb) = ((NeighbNWeight+poppedNWeight)/2)*norm(nodeList(:,poppedNodeIdx)-nodeList(:,nextNeighb));
            end
        end

    end
    
%     if isinterior(futurePredictObject, nodeList(1,poppedNodeIdx), nodeList(2,poppedNodeIdx))
%          poppedNodeChilds= node(poppedNodeIdx).child;
%          for k=1:numel(poppedNodeChilds)
%              poppedChild=poppedNodeChilds(k);
%              distNeighbMat(poppedChild,poppedNodeIdx)=30;
%          end
%     end
    
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
% denenmeqsda 
end
