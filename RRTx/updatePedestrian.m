function [obstacle,node,removeObsSample,graphTree,Q,...
    QMatrix,distNeighbMat]= updatePedestrian(obstacle,removeObsSample,node,nodeList,graphTree,...
    R,i,Q,QMatrix,distNeighbMat,orphanNodeMat,param)

IsThereDetectedObstacle=false;
counterObsOrphanNodes=0;
totalNumberOfStaticObs=numel(find(cellfun(@isempty,{obstacle.Orientation}))); % It gives total number of static obstacles
robotPosT=R.robotPose(1:2)'; % Robot Position

for movingObs= totalNumberOfStaticObs+1:numel(obstacle)
    
    RobotObsDist=norm(obstacle(movingObs).Path(obstacle(movingObs).ObsTimePoint,1:2)- robotPosT'); %Distance betwen Robot and Obstacle 
    obsMoveStep=obstacle(movingObs).ObsTimePoint; 
    obsTotMoveStep=numel(obstacle(movingObs).Orientation); % The total number of moving steps of Obstacle
    
    % if obstacle;
    % is a moving  obstacle (R.moving==1), 
    % is on the map (obstacle(movingObs).Active==1)
    % is wating, which means it is stable at the current position 
    % set to cost of the edges that intersects with current position of the obstacle. Don't make prediction for the future

    if obstacle(movingObs).Active==1
        
        if param.sensorRange  > RobotObsDist
            % Update the cost of the nodes for hard constraint
            obstacle(movingObs).Used=1;
            [node,graphTree,Q,QMatrix,orphanNodeMat,counterObsOrphanNodes,distNeighbMat]=...
                addPedestrianCurrentPos(movingObs,Q,...
                QMatrix,node,nodeList,graphTree,R,obstacle,orphanNodeMat,counterObsOrphanNodes,distNeighbMat,param);
%             [obstacle,removeObsSample,node,graphTree,Q,QMatrix,distNeighbMat]= updatePedestrianCurrentPos(movingObs,removeObsSample,obstacle,node,nodeList,graphTree,R,i,Q,QMatrix,distNeighbMat,r,eConstant,orphanNodeMat);

            obstacle(movingObs).Detected=true;
%             obstacle(movingObs).ElapseTime= false;
            IsThereDetectedObstacle=true;
        else
            obstacle(movingObs).Detected=false;
        end
    end
end

if IsThereDetectedObstacle==true
    %PropogateDescendants();
    [node,graphTree,Q,QMatrix]=propogateDescendants(node,graphTree,Q,QMatrix,orphanNodeMat,counterObsOrphanNodes);
    %     %VerrifyInQueue(Vbot);
    [node,QMatrix,Q] = verifyInQueue(node,R.robotNode,QMatrix,Q);
    %     %reduceInconsistency();
    [graphTree,node,Q,QMatrix]= reduceInconsistency(node,nodeList,graphTree,param,Q,QMatrix,distNeighbMat,R);
end



IsThereDetectedObstacle=false;
counterObsOrphanNodes=0;
% totalNumberOfStaticObs=numel(find(cellfun(@isempty,{obstacle.Orientation}))); % It gives total number of static obstacles
% robotPosT=R.robotPose(1:2)';
    
%addObstacle Part
for movingObs= totalNumberOfStaticObs+1:numel(obstacle)
    
    RobotObsDist=norm(obstacle(movingObs).Path(obstacle(movingObs).ObsTimePoint,1:2)- robotPosT'); %Distance betwen Robot and Obstacle 
    obsMoveStep=obstacle(movingObs).ObsTimePoint; % The current step of the obstacle in the defined path
    obsTotMoveStep=numel(obstacle(movingObs).Orientation); % The total number of moving steps of Obstacle
    
    % if obstacle;
    % is a moving  obstacle (R.moving==1), 
    % is on the map (obstacle(movingObs).Active==1)
    % is not wating, whic means not stable at the current position make prediction for the future and set to edge cost based on human motion model
    if obstacle(movingObs).Active==1
        
        if param.sensorRange  > RobotObsDist
            % Update the cost of the nodes for hard constraint
            obstacle(movingObs).Used=1;
            [node,graphTree,obstacle,Q,QMatrix,orphanNodeMat,counterObsOrphanNodes,...
                distNeighbMat]=HumanFutureCollisionCost(obstacle,movingObs,Q,QMatrix,R,node,...
                nodeList,graphTree,orphanNodeMat,counterObsOrphanNodes,i,distNeighbMat,param);

            obstacle(movingObs).Detected=true;
            obstacle(movingObs).ElapseTime= false;
            IsThereDetectedObstacle=true;
        else
            obstacle(movingObs).Detected=false;
        end
    end
end

% if there is a detected obstacle, do the rest of the updateObstacle Funtion
if IsThereDetectedObstacle==true
    %PropogateDescendants();
    [node,graphTree,Q,QMatrix]=propogateWeightDescendants(node,graphTree,Q,QMatrix,orphanNodeMat,counterObsOrphanNodes);
    %     %VerrifyInQueue(Vbot);
    [node,QMatrix,Q] = verifyInQueue(node,R.robotNode,QMatrix,Q);
    %     %reduceInconsistency();
    [graphTree,node,Q,QMatrix]= reduceInconsistency(node,nodeList,graphTree,param,Q,QMatrix,distNeighbMat,R);
end
%     %addObstacle function
%     [node,futurePredictObject,graphTree,obstacle,Q,QMatrix,orphanNodeMat,counterObsOrphanNodes,distNeighbMat]=HumanFutureCollisionCost(obstacle,obsNumber,Q,QMatrix,obsTimePoint,imove,R,node,nodeList,graphTree,orphanNodeMat,counterObsOrphanNodes,r,i,distNeighbMat);
%     
%     %PropogateDescendants();
%     [node,graphTree,Q,QMatrix]=propogateWeightDescendants(obsNumber,obstacle,nodeList,node,graphTree,Q,QMatrix,orphanNodeMat,counterObsOrphanNodes);
% 
% %     %VerrifyInQueue(Vbot);
%      [node,QMatrix,Q] = verifyInQueue(node,R.robotNode,QMatrix,Q);
% %     %reduceInconsistency();
%      [graphTree,node,Q,QMatrix]= reduceInconsistency(node,nodeList,graphTree,r,eConstant,Q,QMatrix,distNeighbMat,R);


end