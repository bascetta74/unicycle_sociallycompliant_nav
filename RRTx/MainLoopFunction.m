function [ivideo,VideoData,lastFrame]= MainLoopFunction(posStart,nodeGoal,startFrame,ivideo,simVisualTime,simulationRobotMoveTime,simulationObsMoveTime,PrevVideoData)

% close all

if nargin==8 % This is for taking the previous video, start writing over it. We use for the second and third goal.

    VideoData=PrevVideoData;
end
    

numSample= 3000; % The max number of Samples in the graph
% nodeGoal= [13;10]; %Goal node
% posStart= [2;8]; %Starting Node
delta= 2;
% edgeCostTimeConstant=0.1; % This constant increase the importance of time vaule in the edge cost calculations 
% control random number generation for having same random nodes 
rand_seed=4;
rng(rand_seed);
%Max_time= norm(nodeList(1:2,1)-nodeList(1:2,2));
Max_time=20;
nodeList(3,1)=Max_time; % Max_time is set to time value od start node.The maximum time to reach the goal node from start node, it is set to t
% Robot Velocity
robotMaxVelocity=2;
robotMinVelocity=0.5;
numKdNode=1;
ballconstant= 100;  
nodeList(1,1)= posStart(1,1);
nodeList(2,1)= posStart(2,1);
nodeList(1,2)= nodeGoal(1,1);
nodeList(2,2)= nodeGoal(2,1);
nodeList(3,2)= 0.0; % Time is set to 0 which is the time parameter of the goal node 
%nodeListWoutStart= nodeList(:,2:end);

%Simulation Time parameters
% simulationRobotMoveTime=70;
% simulationObsMoveTime=70;

% Making grapgh
graphTree= digraph;
graphTree = addnode(graphTree, {'1'}); % start node adding in the tree
graphTree = addnode(graphTree, {'2'}); % end node adding in the tree
node(numSample).parent= [];
node(2).Gv = 0.001;
node(1).Gv= Inf;
node(numSample).Gv=[];
[node.Gv]= deal(Inf);
node(numSample).Lmc=[];
[node.Lmc]= deal(Inf);
node(2).Lmc=0;
node(numSample).child=[];
QMatrix=[];
node(numSample).MarkedQ=[]; 
[node.MarkedQ]= deal(false); % initiliaze all the nodes of Q as false
node(numSample).InitialOutConnect=[];
node(numSample).RadiusInConnect=[];
node(numSample).RadiusOutConnect=[];
node(numSample).InitialInConnect=[];
node(numSample).MarkedQOB=[];
[node.MarkedQOB]= deal(false); % initiliaze all the nodes of obstacle intersected Q values as false
eConstant = 1;
Q = MinHeap(numSample/2);
countUpdateLmc = 0;
countRewire = 0;
maxPathNodes= 300;
intersectedNode = 0;
removeObsSample =[];
orphanNodeMat=zeros(2,numSample/10);
distNeighbMat=NaN(numSample,numSample);

%OBSTACLE UPDATE
%updateObsMat=zeros(3,obsUpdateCount);
%updateObsMat=[25 55 75 ; 4 3 3; 1 -1 1];
%updateObsMat=[35  90; 4 1; 1 -1];
updateObsMat=[20000; 1; -1];
% updateObsMat=[];
sizeUpdateObsMat=size(updateObsMat);
obsUpdateCount=sizeUpdateObsMat(2);
iterObsTime=1;
noMoreUpdate=false;
% obstacle(3).StartTime = 69.5;
% obstacle(4).StartTime = 70;

% Robot Data initializing 
R = RobotData(nodeList(:,1)', 1, maxPathNodes,robotMaxVelocity,robotMinVelocity);
MoveRobotFlag=1;
sliceTime=0.1;
iMoveIncrease = 0;
robotElapse= zeros(1,2000);
sensorRange= 4;
robotPosT = posStart;

figure(7);
set(gca, 'XLim', [0 16],'YLim', [0 16]);
hold on;
% curve = animatedline('Color','r','Marker','p','MarkerSize',6,'LineWidth',2);
imove=1; % iteration for robot move video
% ivideo=1; % iteration parameter for video
averageTime=zeros(10000,1);
loopTimeCounter=1;
prevtimeObsMove=0;
% addedNode= 0;
 % add other "times" to root of tree
[nodeList,graphTree,node,addedNode,distNeighbMat]= addOtherTimesToRoot(nodeList,node,graphTree,R,distNeighbMat);


i=3+addedNode;

% OBSTACLE DATA INITIALIZATION

totalNumberOfStaticObs=4; % the number of static Obstacles
maxNumberOfMovingObs=70; % the maximum number of moving Obstacles

% IMPORTANT- MAIN INITILIZATION FUNCTION OF OBSTACLES- 
% startSecond=36;
% I have defined a starting Frame in the input of MainLoopFunction
[obstacle,totalNumberOfMovingObs] = loadObstacle(totalNumberOfStaticObs,maxNumberOfMovingObs,simulationObsMoveTime,startFrame); % The obstacle is loaded in to map, the number inside shows the total number of obstacle are defined
totalNumberObs=totalNumberOfStaticObs + totalNumberOfMovingObs;
% By checking one specific frame, we determine the number of moving
% Obstacles
timeRobot=1;
% totalNumberObs=numel([obstacle(1:end).PosX]);
% obstacle(1).Used=0;
% obstacle(4).Used=0; % The obstacle with obstacle(n).Used=0 means this obstacles is not active



% Timing stuff
tic;
saveElapsedTime=0.0;
sliceCounter= 0;
setSliceCounter=true;
elapsedTime = 0.0;
startTimeNs = 0.0;

% The loop start here
while i <= numSample    % i equals to node number

nowTime = toc;
%calculate the end time of the first slice
sliceEndTime= (1+sliceCounter)*sliceTime;
r = min(delta, ballconstant*sqrt(log(numKdNode+1)/numKdNode)); %Shrinking Ball Radius
nodeListT = nodeList(:,1:i-1)';

elapsedTime= (toc - startTimeNs) - saveElapsedTime;
%timeObsUpdate=toc;

%%% Updating the STATIC obstacles based on updateObsMat
if noMoreUpdate~=true && elapsedTime > updateObsMat(1,iterObsTime)
% %Update Obstacle operation
    [obstacle,removeObsSample,node,graphTree,Q,QMatrix,distNeighbMat]= updateObstacles(updateObsMat(2,iterObsTime),updateObsMat(3,iterObsTime),removeObsSample,obstacle,node,nodeList,graphTree,R,i,Q,QMatrix,distNeighbMat,r,eConstant,orphanNodeMat,countRewire,countUpdateLmc);
    iterObsTime=iterObsTime +1;
    if iterObsTime == obsUpdateCount +1
        noMoreUpdate=true;
    end
end

% This is the part for translation of the moving obstacles in the map
for movingObs= totalNumberOfStaticObs+1:totalNumberObs
    
    obstacle=humanPositionChange(obstacle,movingObs,elapsedTime,sliceEndTime);
    
end

if imove > 3
robotPosT=R.robotMovePath(imove-2,1:2)';
end

%%% IMPORTANT: Updating the cost of Moving obstacle region WHEN it is detected
%%% by robot sensors
%First don't count the time that is spent for removing obs and add mocing
%obstacle
    elapsedTime= (toc - startTimeNs) - saveElapsedTime;
    beforeVideoTime = toc;
%%% Remove the effect of areas previously occupied by the moving
%%% obstacles by checking old trajectory of the obstacle 
for movingObs= totalNumberOfStaticObs+1:totalNumberObs
    
    RobotObsDist=norm(obstacle(movingObs).Path(1:2,obstacle(movingObs).ObsTimePoint)- robotPosT);
    obsMoveStep=obstacle(movingObs).ObsTimePoint;
    obsTotMoveStep=numel(obstacle(movingObs).Orientation);
    
    if obstacle(movingObs).Active==1 && sensorRange > RobotObsDist && obsMoveStep<= obsTotMoveStep...
            && obstacle(movingObs).ElapseTime && ~obstacle(movingObs).Waiting 
%         obstacle(movingObs).Used=1;

        % Remove Moving Obsatacle Function
    %     [node,graphTree,Q,QMatrix,removeObsSample,distNeighbMat]=removeObstacle(obstacle(obsNumber).Object,obstacle,obsNumber,removeObsSample,node,nodeList,graphTree,i,Q,QMatrix,distNeighbMat,r,countRewire,countUpdateLmc);
        [node,graphTree,Q,QMatrix,distNeighbMat]=removeMovingObstacle(obstacle,movingObs,node,nodeList,obstacle(movingObs).ObsTimePoint,imove,R,graphTree,i,Q,QMatrix,distNeighbMat,r,countRewire,countUpdateLmc);

    %     % Reduce Inconsistency
        [graphTree,node,Q,QMatrix,countUpdateLmc,countRewire]= reduceInconsistency(node,nodeList,graphTree,r,eConstant,Q,QMatrix,distNeighbMat,R,countRewire,countUpdateLmc);
    end
end

%%% Updating the cost area of the moving obstacle at each sampling time and
%%% define cost on the nodes in these areas based cost function
for movingObs= totalNumberOfStaticObs+1:totalNumberObs
    
    RobotObsDist=norm(obstacle(movingObs).Path(1:2,obstacle(movingObs).ObsTimePoint)- robotPosT);
    obsMoveStep=obstacle(movingObs).ObsTimePoint;
    obsTotMoveStep=numel(obstacle(movingObs).Orientation);
    
    if obstacle(movingObs).Active==1  && obsMoveStep <= obsTotMoveStep...
            && obstacle(movingObs).ElapseTime && ~obstacle(movingObs).Waiting 
        % Update the cost of the nodes for hard constraint
        
        % if it is in sensor range make future prediction collision check.
        % Mark as detected
        if sensorRange  > RobotObsDist
            
            [obstacle,removeObsSample,node,graphTree,Q,QMatrix,distNeighbMat]= updatePedestrianCurrentPos(movingObs,1,removeObsSample,obstacle,node,nodeList,graphTree,R,i,Q,QMatrix,distNeighbMat,r,eConstant,orphanNodeMat,countRewire,countUpdateLmc);

            % weight adding around moving obstacle FOR PERSONAL SPACE
    %         countOrphan= 0;

            %%% Update the cost of the nodes around Moving obstacle for SOFT
            %%% CONSTRAINT
            [obstacle,node,removeObsSample,graphTree,Q,QMatrix,distNeighbMat, obstacle(movingObs).FuturePredictObject]= updateMovingObstacles(movingObs,obstacle,removeObsSample,node,nodeList,obstacle(movingObs).ObsTimePoint,imove,graphTree,R,i,Q,QMatrix,distNeighbMat,r,eConstant,orphanNodeMat,countRewire,countUpdateLmc);

            obstacle(movingObs).Detected=true;
            obstacle(movingObs).ElapseTime= false;
            
        else % otherwise neglected mark as not detected 
            
            obstacle(movingObs).Detected=false;
        end

    end
end

timeObsMove=toc;
if i > 1 && i < 10000
    averageTime(loopTimeCounter,1)=timeObsMove-prevtimeObsMove;
    loopTimeCounter=loopTimeCounter+1;
end
prevtimeObsMove=  timeObsMove;

% Moving Obstacle starts moving
% if elapsedTime > obstacle(5).StartTime && (waitMoving==false) && obstacle(5).Moving ~= false
% % remove moving obstacle when time comes    
%     if removeMovingObsTime == 1 && elapsedTime > obstacle(5).FinishTime
%         [node,graphTree,Q,QMatrix,distNeighbMat]=removeMovingObstacle(obstacle(5).Object,obstacle,5,node,nodeList,graphTree,i,Q,QMatrix,distNeighbMat,r,countRewire,countUpdateLmc);
% 
%     %     % Reduce Inconsistency
%         [graphTree,node,Q,QMatrix,countUpdateLmc,countRewire]= reduceInconsistency(node,nodeList,graphTree,r,eConstant,Q,QMatrix,distNeighbMat,R,countRewire,countUpdateLmc);
% 
%         removeMovingObsTime = false;
%     end
%     
%     if changeObstacleDirection == true
%         [obstacle,node,removeObsSample,pathIter,graphTree,Q,QMatrix,distNeighbMat]= updateMovingObstacles(5,obstacle,removeObsSample,pathIter,node,nodeList,graphTree,R,i,Q,QMatrix,distNeighbMat,r,eConstant,orphanNodeMat,countRewire,countUpdateLmc);
%         changeObstacleDirection = false;
%         removeMovingObsTime= true;
%     end
%     
% 
%     
% %     startMove=true;
% %     waitMoving=true;
%     numberofMovement= numel(obstacle(5).Path(3,:));
%     if numberofMovement ==pathIter
%         obstacle(5).Moving = 0;
%     else
%         translateX=obstacle(5).Path(1,pathIter+1)-obstacle(5).Path(1,pathIter);
%         translateY=obstacle(5).Path(2,pathIter+1)-obstacle(5).Path(2,pathIter);
%         obstacle(5).Object=translate((obstacle(5).Object),translateX,translateY);
%         pathIter = pathIter+1;
%     end
% end


%%% IMPORTANT VIDEO DATA SAVING and moving robot function, time is  
%savedElapsedTime= toc;
% robotMoveTime= toc;
% elapsedTime= (toc - startTimeNs) - saveElapsedTime;
if elapsedTime >= sliceEndTime  && elapsedTime > simVisualTime
    % calculate the end time of the next slice
    if setSliceCounter
        sliceCounter=round(elapsedTime*10,0);
        setSliceCounter=false;
    end
    sliceEndTime = (1+ sliceCounter)* sliceTime;
    
    robotSliceStart= nowTime;
    
    sliceCounter = 1 + sliceCounter;
    
    robotMoveTime= elapsedTime;
%     beforeVideoTime = toc;
    if MoveRobotFlag==1 && robotMoveTime > simulationRobotMoveTime && ~isempty(node(1).parent)
       [R,iMoveIncrease]=moveRobot(node,nodeList,R,distNeighbMat,sliceTime);
       obstacleMoveTime=robotMoveTime;
       robotElapse(1,timeRobot)=robotMoveTime;
       timeRobot=timeRobot+1;
    end    
    
%     beforeVideoTime = toc;
    
    set(gca, 'XLim', [0 16],'YLim', [0 16]);
    clf
    hold on
%     [centx,centy]=centroid(obstacle(6).Object);
%     scaleAddedObs= scale(obstacle(6).Object,2,[centx,centy]);
    for movingObs= totalNumberOfStaticObs+1:totalNumberObs
        if obstacle(movingObs).Used == 1 && obstacle(movingObs).ObsTimePoint <= numel(obstacle(movingObs).Orientation)
    %         plot(obstacle(movingObs).WeightArea{obsTimePoint},'FaceColor','white','FaceAlpha',0.1,'LineStyle','--','EdgeColor','red')
            if obstacle(movingObs).Detected && ~obstacle(movingObs).Waiting
            plot(obstacle(movingObs).FuturePredictObject,'FaceColor','white','FaceAlpha',0.1,'LineStyle','--','EdgeColor','black')
            end
        end
    end
%     obsPlot= cellfun(@plot,{obstacle([obstacle.Used]==1).Object});
    numberCurrentObs=[obstacle.Used]==1;
%     numberCurrentObs(numberCurrentObs==0)=[];
    for obsi=1:numel(numberCurrentObs)
        if numberCurrentObs(obsi)==1
            if obstacle(obsi).Moving==1
                plot(obstacle(obsi).Object,'FaceColor','b')
%                 obsPlot(obsi).FaceColor='b';
            else
                plot(obstacle(obsi).Object,'FaceColor','m')
            end
        end    
    end
    set(gca, 'XLim', [0 16],'YLim', [0 16]);
    % sensor Range drawings
    th = 0:pi/50:2*pi;
    xunit = sensorRange * cos(th) + R.robotMovePath(imove,1);
    yunit = sensorRange * sin(th) + R.robotMovePath(imove,2);
    headSensor = plot(xunit, yunit,'--y','LineWidth',1);
    % shortest path and robot position drawings
    %addpoints(curve,R.robotMovePath(imove,1),R.robotMovePath(imove,2));
    pmovie=plot(graphTree,'XData',nodeList(1,:),'YData',nodeList(2,:),'Marker','o', 'EdgeColor','b', 'MarkerSize',1.5,'NodeColor','c','EdgeAlpha',0.3);
    TRmovie = shortestpathtree(graphTree,{num2str(R.nextMoveTarget)},{'2'}); % find the shortest path from node 1 to 2
    currentEdgeMovie= shortestpathtree(graphTree,{num2str(R.robotNode)},{num2str(R.nextMoveTarget)});
    highlight(pmovie,TRmovie,'EdgeColor','r', 'LineWidth', 2,'NodeColor','g') % Shortest Path highlight on the graph
    highlight(pmovie,currentEdgeMovie,'EdgeColor','g', 'LineWidth', 2,'NodeColor','k')
    head = scatter(R.robotMovePath(imove,1),R.robotMovePath(imove,2),60,'filled','p','MarkerFaceColor','r');
    drawnow
    %ax= gca;
    VideoData(ivideo)= getframe(gcf,[0 0 500 400]);
    %size(F(ivideo).cdata)
    ivideo= ivideo +1;
    imove= imove+iMoveIncrease;
    
    saveElapsedTime = saveElapsedTime + (toc- beforeVideoTime);
    
%     mypose=R.robotMovePath(end,1:2);
%     if nodeGoal == mypose'
%         R.robotPose(1:2)=nodeListT(2,1:2);
%         break
%     end
    
% Robot reaches to final goal or if it is too close to final goal (this is added to preveent small calculation errors in the path planning)
    if (R.robotPose(1)==nodeListT(2,1) && R.robotPose(2)==nodeListT(2,2)) || ( abs( R.robotPose(1)-nodeList(2,1)) < 0.0001 && abs( R.robotPose(2)-nodeList(2,2)) < 0.0001 ) 
        for k= 1:totalNumberObs
            if obstacle(k).Used==1
                if obstacle(k).Moving == 1
                    % I first check the time of the obstacle to give an output. the simulation stop time is to use for the
                    % the next target
                    stopSecond=obstacle(k).StartTime-simulationObsMoveTime+(obstacle(k).ObsTimePoint-1)/10;
                    stopFrame=stopSecond*25;
                    if mod(stopFrame,10)~=0
                        stopWaitFrame=10-(mod(stopFrame,10)-1);
                    else
                        stopWaitFrame=1;
                    end
                    lastFrame=stopFrame+stopWaitFrame+startFrame-1; % The last frame is the taken refence by the simulation for the next movement
                    break
                end
            end
        end
        
        %  If there is no obstacle in the map, I check the time of Robot to
        %  find the last frame of the video after the simulation is
        %  completed
        checkLastFrame=exist('lastFrame','var');
        if checkLastFrame == 0
            timePassedRobot=nodeList(3,1)-R.robotPose(3);
            stopSecond=simulationRobotMoveTime-simulationObsMoveTime+timePassedRobot;
            stopFrame=stopSecond*25;
            if mod(stopFrame,10)~=0
                stopWaitFrame=10-(mod(stopFrame,10)-1);
            else
                stopWaitFrame=1;
            end
            lastFrame=stopFrame+stopWaitFrame+startFrame-1; % The last frame is the taken refence by the simulation for the next movement
        end
        
        % The goal is reached this is the exiting from the function
        return
    end

end

%%% If the parent of Robot node changes during the movement in the edge,
%%% then sample a point in the current robot Pos and connect with neighbour
parentEdgeDisconnected=false;
if ~isempty(node(R.robotNode).parent)&& timeRobot>1 && node(R.robotNode).parent~= R.nextMoveTarget 
    removeObsSample=[removeObsSample,R.nextRobotPose'];
    R.robotPose=R.nextRobotPose';
    parentEdgeDisconnected=true;
%     R.moving=0;
    R.robotNode=i;
%     imove=imove-2;
end

%%% Randomly sampling a point or sampling from a determined list
%%% RemoveObsSample
if isempty(removeObsSample)
    newSample= [14;7;nodeList(3,1)-nodeList(3,2)].*rand(3,1)+[0;6;0]; % Random sampling
    
    minTimeToReachNode = nodeList(3,2)+ norm(nodeList(1:2,2)-newSample(1:2,end))/ (R.robotMaxVelocity);
    
    %This if condition is applied to saturate the the sampling time parameter for the new node
    if newSample(3,1) < minTimeToReachNode || (newSample(3,1) > nodeList(3,R.robotNode) && R.robotNode ~= 1)
        %resample time in ok range
        newSample(3,1)= minTimeToReachNode + rand * (nodeList(3,R.robotNode)- minTimeToReachNode);
    end
    
else
    newSample= removeObsSample(:,end); 
    if parentEdgeDisconnected  ~=1 % Similarly,This if condtion is for the sampling time parameter for the new node for the case that the empty space behind a big obstacle to be filled with new sampled nodes.
        newSample(3,1)= (nodeList(3,1)-nodeList(3,2))*rand;
        if newSample(3,1) < minTimeToReachNode || (newSample(3,1) > nodeList(3,R.robotNode) && R.robotNode ~= 1)
            %resample time in ok range
            newSample(3,1)= minTimeToReachNode + rand * (nodeList(3,R.robotNode)- minTimeToReachNode);
        end
    end
    % Note: if the parent of the the the current Robot Node is empty. Then
    % no need to find a rondom time, directly take sampling at the robotPos
    % with defined time
    removeObsSample = removeObsSample(:,1:end-1); % Taking the sample from the area where removed Obstacles located before
end

%Vnearest <<< nearest(v) - Finding the nearest node
nodeListWoutStart= nodeList(:,2:end);
addedNodeT = nodeListWoutStart(:,1:i-2)';
% here I try to find the node with a smaller time value becauce this
% closest node may be the parent node. Therefore, I neglect all nodes whose the
% time values are greater than time vale of newSample
addedNodeT(addedNodeT(:,3)>newSample(3,1),3)=1000;
newSampleT= newSample(:,1)';


% Knn Search.Idx is the index of the closest Node in nodeList
[Idx, nnodeDist]= knnsearch(addedNodeT,newSampleT); 
% nnodeDist is the distance to closest node

% Closest node position 
closestPoint= nodeListWoutStart(:,Idx);

% Saturation operation
% moves newPoint toward closestPoint such that each robot is no 
% further than delta, points repesent the cartesian product of R robots
if nnodeDist > delta %  If newSample is far away than the radius then find a new point which is delta  distance away in the direction newSample  
    
    saturNode= closestPoint + (newSample - closestPoint)*(2)/norm(newSample - closestPoint) ;
    
else %If the newSample is less than r, use this point as a saturNode
    saturNode=newSample;
%     nnodeDist = norm(newSample-closestPoint);
end

% newSampleTime=toc;
% Define a new time variable for nodeList
% saturNode(3,1)=newSampleTime;



% Checking whether new Node intersects with the obstacles
% buraya yeni bir future position of obstacle check yapacam
for k= 1:totalNumberObs
    if obstacle(k).Used==1
        if obstacle(k).Moving ~= 1
            checkobs=obstacle(k).Object;
            %checkobs=currentObsCellList{existedObsMat(k),1};
            intersectedNode = isinterior(checkobs, saturNode(1,1), saturNode(2,1));
            if intersectedNode
                break
            end
        end
    end
end
% THE ALGORITHM 2 EXTEND(v,r) STARTS

% Vnear <<< near(v,r) Finding Vnear operation
saturNodeT= saturNode(:,1)';
NearNodesIdxCell= rangesearch(nodeListT,saturNodeT,r+0.001); % Finding the near nodes in the circle of r for saturated Node(saturNode)
NearNodesIdx=cell2mat(NearNodesIdxCell);


if (~intersectedNode)  && (~isempty(NearNodesIdx))

%    node(i).NearNodes= NearNodesIdx;

%FIND PARENT (v,Vnear)
    nearSaturNode=zeros(3,numel(NearNodesIdx));
    node(i).Gv = Inf;
    % nearStartNodeFound can be deleted if there is time dimension in the system.
    %nearStartNodeFound function is used when there is no time parameter.
    % The reason to not allow exploring the graph from the Goal node. 
%     nearStartNodeFound =false; % nearStartNodeFound function is used when
    noValueSaturatedNodeLmc = false;
    parentFound= false;
    
    collisionFreeEdgesMat = zeros(1,numel(NearNodesIdx));
    for k = 1:numel(NearNodesIdx) % Finding the position of near nodes in nodeList
        nearSaturNode(:,k)= nodeList(:,NearNodesIdx(k));
%         nearDist= norm(nearSaturNode(:,k)-saturNode); % The distance between the saturated Node and its neighbours 
%         distNeighbMat(i,NearNodesIdx(k))=nearDist;
%         distNeighbMat(NearNodesIdx(k),i)=nearDist;
        currentIdx = NearNodesIdx(k);        
%         if currentIdx == 1 % Don't use when there is time dimension
%             nearStartNodeFound = true;
%             % bu durum Start node parent yapmayi engellemek icin
%             %continue
%         else
%             nearStartNodeFound = false;
%         end
%         intersectX =0; % IntersectX shows if there is a collision with obstacle
        collisionEdge = false;
%         CollisionCost=0;
        futureCollisinCost=0;
        for kr= 1:totalNumberObs % obstacle avoidance checking if the new connection between saturated node and nearest node
            if obstacle(kr).Used==1
                % Firstly checking collision check with the future
                % prediction area of the obstacle
                if parentEdgeDisconnected==true && obstacle(kr).Moving==1 && obstacle(kr).Detected==1
                    [oneEdgeCollision,CollisionCost]=OneEdgeFuturePredictCollision(saturNode,nearSaturNode(:,k),kr,obstacle,obstacle(kr).ObsTimePoint,R);
                    if oneEdgeCollision==1
                        if CollisionCost > futureCollisinCost
                            futureCollisinCost=CollisionCost; % Future CollisionCost will be used to assign the cost of this conflicted edge
                        end
                    end
                end
                [obsx,obsy]=boundary(obstacle(kr).Object);
                intersectX = polyxpoly([saturNode(1,1) nodeList(1,currentIdx)],[saturNode(2,1) nodeList(2,currentIdx)], obsx,obsy);
                if isempty(intersectX) ~= 1 % if there is a conflict between the possible connection and obstacle
                    distNeighbMat(i,currentIdx)=inf;
                    break
                end

            end
        end
        
        
        % For validMove function in time based on the robot Velocity
        edge2dDist= norm(nearSaturNode(1:2,k)-saturNode(1:2,1));
        % Valid move of the robot depends on the 
        validMove= ( (saturNode(3,1)-nearSaturNode(3,k))*R.robotMinVelocity )<= edge2dDist && edge2dDist <= ( (saturNode(3,1)-nearSaturNode(3,k))*R.robotMaxVelocity );
%         validMove=edge2dDist <= ( (saturNode(3,1)-nearSaturNode(3,k))*R.robotMaxVelocity );
            if isempty(intersectX) && validMove
%                 nearDist= norm(nearSaturNode(:,k)-saturNode(:,1));
                nearDist=edgeCost(saturNode(:,1),nearSaturNode(:,k));
%                 nearDist= timeCostConstant*(saturNode(3,1)-nearSaturNode(3,k))+norm(nearSaturNode(1:2,k)-saturNode(1:2,1)); % The distance between the saturated Node and its neighbours 
                if futureCollisinCost ==0 % this is a cost value if the robot's edge connection is lost, this gives futureCollisionCost value to the directly edge
                    distNeighbMat(i,currentIdx)=nearDist;
                else
                    distNeighbMat(i,currentIdx)=futureCollisinCost; % Giving the weighted cost for the conflicted edge
                end
%                 distNeighbMat(currentIdx,i)=nearDist;               
            else
                collisionEdge=true;
                collisionFreeEdgesMat(1,k)=1;
                distNeighbMat(i,currentIdx)=inf;
%                 distNeighbMat(currentIdx,i)=inf;                    
            end
        % To find The optimum and collision free obstacle with this if
        % condition
%         stringofCurrentIdx= int2str(currentIdx);            
%         stringofSaturNode= int2str(i);
        
        % finding the parent if neccesary conditions are satisfied
        if (distNeighbMat(i,currentIdx) <= r+0.00001) && node(i).Lmc > (distNeighbMat(i,currentIdx) + node(currentIdx).Lmc)  && (~collisionEdge) %% && (~nearStartNodeFound)
            
            parentFound= true;
            node(i).parent = currentIdx;
            node(i).Lmc = distNeighbMat(i,currentIdx) + node(currentIdx).Lmc;
            if noValueSaturatedNodeLmc
                
                noValueSaturatedNodeLmc= false;
                
            end
        end
        % Connect Start Node to Graph
% % %         if nearStartNodeFound && (~collisionEdge)
% % %             distStartNeighb= norm(nodeList(:,1)-saturNode(:,1));
% % %             if node(i).Lmc ~= inf
% % %                 startNodeConnected= successors(graphTree,'1');
% % %                 if isempty(startNodeConnected)
% % %                     graphTree= addedge(graphTree,{'1'},{stringofSaturNode},distStartNeighb);
% % %                     node(i).child= [node(i).child,1];
% % %                     node(1).parent=i;
% % %                     node(1).Lmc = distStartNeighb + node(i).Lmc;
% % %                 elseif node(1).Lmc > ( node(i).Lmc + distStartNeighb)
% % %                     prevStartParent=successors(graphTree,'1');
% % %                     %deleting old parent of start Node
% % %                     graphTree = rmedge(graphTree,{'1'}, prevStartParent);
% % %                     prevStartParentIdx=str2double(cell2mat(prevStartParent));
% % %                     node(prevStartParentIdx).child(node(prevStartParentIdx).child==1)=[];
% % %                     graphTree = addedge(graphTree,{'1'}, {stringofSaturNode},distStartNeighb);
% % %                     node(i).child= [node(i).child,1];
% % %                     node(1).parent=i;
% % %                     node(1).Lmc = distStartNeighb + node(i).Lmc;
% % %                 end
% % %                 nearStartNodeFound = false;
% % %             else
% % %                 noValueSaturatedNodeLmc= true;
% % %                 nearStartNodeFound = false;
% % %             end
% % %         end
    end
    


    newNode= saturNode;
    % Extend line 3 for parent checking and the rest of lines for extend
    collisionCheckAllEdges= isempty(find(collisionFreeEdgesMat==0,1));
    if ~collisionCheckAllEdges && parentFound
        nodeList(:,i)= newNode(:,1);
        currentParent=node(i).parent;
        stringofParentIdx= int2str(currentParent);            
        stringofNewNode= int2str(i);
        graphTree = addedge(graphTree, {stringofNewNode},{stringofParentIdx},distNeighbMat(i,currentParent));
        % Verify node is added
        % Making the new node the child of its parent
        if ~ismember(i, node(currentParent).child)
            node(currentParent).child = [node(currentParent).child, i];
        else
            numOfChild=numel(node(currentParent).child); 
            node(currentParent).child(1,numOfChild+1)=i; 
        end  
        isNodeAdded = true;

        % rvalues(1,i)=r; % to check the hyperball radius
        for k = 1:numel(NearNodesIdx) % Finding the position of near nodes in nodeList
            %intersectOut = [];
%             intersectIn= [];
            nearNode = NearNodesIdx(k);
            % obtacle lari tek tek degerlendirip break koymam gerekecek gibi

            % Checking collision between the connected path of two neighbours and obstacle
            if distNeighbMat(i,nearNode) ~= Inf
                numOutConnect=numel(node(i).InitialOutConnect);
                node(i).InitialOutConnect(1,numOutConnect+1)= nearNode;
                numInConnect=numel(node(nearNode).RadiusInConnect);
                node(nearNode).RadiusInConnect(1,numInConnect+1)=i;
            end
            
            % For validMove function in time
           edge2dDist= norm(nodeList(1:2,nearNode)-newNode(1:2,1));
%             validMove2= edge2dDist <= ( (nodeList(3,nearNode)-newNode(3,1))*R.robotMaxVelocity );
            validMove2=( (nodeList(3,nearNode)-newNode(3,1))*R.robotMinVelocity ) <= edge2dDist && edge2dDist <= ( (nodeList(3,nearNode)-newNode(3,1))*R.robotMaxVelocity );
            if validMove2 == 0
                continue
            end
           edgeCollision2 =false;
            for kr= 1:totalNumberObs % obstacle avoidance checking if the new connection between saturated node and nearest node
                collision=explicitEdgeCheck(kr,obstacle,nodeList(:,nearNode),newNode(:,1));                
                if collision == 1
                    edgeCollision2= true;
                    break
                end
            end
            
            if ~edgeCollision2 && validMove2
                numOutConnect=numel(node(nearNode).RadiusOutConnect);
                node(nearNode).RadiusOutConnect(1,numOutConnect+1)= i;
                numInConnect= numel(node(i).InitialInConnect);
                node(i).InitialInConnect(1,numInConnect+1)= nearNode;
                neighbDist= norm(nodeList(:,nearNode)-newNode(:,1));
%                 neighbDist= timeCostConstant*(nodeList(3,nearNode)-newNode(3,1))+norm(nodeList(1:2,nearNode)-newNode(1:2,1)); 
                neighbDist=edgeCost(nodeList(:,nearNode),newNode(:,1)); % The cost between the saturated Node and its neighbours
                distNeighbMat(nearNode,i)=neighbDist;
            end
            % nearNodeDist = norm(nodeList(1,i)-nodeList(1,nearNode));

            % collisionEdgeNeighnour = false;
        end
        % i=i+1;
    else
        isNodeAdded = false;
    end
end

%Extend operation is finished here and if the node is added pass to
%reduceInconsistency and the rest of the code
if ~intersectedNode && ~isempty(NearNodesIdx) && isNodeAdded


% First Rewire Operation

    [node,QMatrix,Q,graphTree,countRewire]= rewire(i,r,eConstant,nodeList,node,graphTree,Q,QMatrix,distNeighbMat,countRewire,countUpdateLmc);
    
% Reduce Inconsistency

    [graphTree,node,Q,QMatrix,countUpdateLmc,countRewire]= reduceInconsistency(node,nodeList,graphTree,r,eConstant,Q,QMatrix,distNeighbMat,R,countRewire,countUpdateLmc);
    
    i=i+1;
end


numKdNode= i;
end

% pathdistance = ['The final shortest path distance is ', num2str(distances(graphTree,'1','2'))];
% display(pathdistance)

% Timing
time_e= toc;
display(time_e)

%adjTree= adjacency(graphTree, 'weighted');
%shortestPathNodes= shortestpath(graphTree, '80', '1');
%GraphDistance = distances(graphTree); % shows all the distance between connected nodes
%subDistance = distances(Gra,'20','5')
figure
hold on;
set(gca, 'XLim', [0 16],'YLim', [0 16]);
obsPlot= cellfun(@plot,{obstacle([obstacle.Used]==1).Object});
%cellfun(@plot,currentObsCellList);
p= plot(graphTree,'XData',nodeList(1,:),'YData',nodeList(2,:),'Marker','o', 'MarkerSize',2);
TR = shortestpathtree(graphTree,{'1'},{'2'}); % find the shortest path from node 1 to 2
highlight(p,TR,'EdgeColor','r', 'LineWidth', 3) % Shortest Path highlight on the graph

% hold on;
% cellfun(@plot,obstacles);
% hold off;

% Video Creating
% video= VideoWriter('RRTXMultipleObstaclesMultipleGoal.mp4','MPEG-4');
% video.FrameRate = 5;
% open(video)
% writeVideo(video, VideoData)
% close(video)

% pathNodes = shortestpath(graphTree, {'1'},{'2'});
% pathNodesMat= str2double(pathNodes);
% 
% XpathNodes= nodeList(1,pathNodesMat);
% YpathNodes= nodeList(2,pathNodesMat);
% 
% curve = animatedline('Color','g','Marker','p','MarkerSize',8);
% set(gca, 'XLim', [0 100],'YLim', [0 100]);
% grid on;
% for i = 1:length(XpathNodes)
% addpoints(curve,XpathNodes(i),YpathNodes(i));
% drawnow
% pause(0.5)
% end
% 

% graphLmc creating process
% 
% graphLmc = graph;
% 
% for i = 1: numSample
%     stringInt= int2str(i);
%     graphLmc = addnode(graphLmc, {stringInt});
%     for kn=1:numel(node(i).InitialOutConnect) % adding new node connections to Lmc graph
%         initNeighbour= node(i).InitialOutConnect(1,kn);
%         stringofInitNeighbour = int2str(initNeighbour);
%         distanceNeighb= norm(nodeList(:,i)-nodeList(:,initNeighbour));
%         graphLmc = addedge(graphLmc,{stringInt},{stringofInitNeighbour},distanceNeighb);
%     end
% end
%    dLmc= distances(graphLmc);
% figure 
% plotLmc = plot(graphLmc,'XData',nodeList(1,:),'YData',nodeList(2,:),'Marker','*', 'MarkerSize',5);
% TRLmc = shortestpathtree(graphLmc,{'1'},{'2'});
% shortestPath= shortestpath(graphLmc,{'1'},{'2'});
% dLmc= distances(graphLmc);
% highlight(plotLmc,TRLmc,'EdgeColor','m','LineWidth', 3)
% hold on;
% cellfun(@plot,currentObsCellList);


end