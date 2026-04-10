function [obstacle,node,removeObsSample,graphTree,Q,...
    QMatrix,distNeighbMat]= updateMovingObstacles(obstacle,removeObsSample,CollisionTolerance,node,nodeList,graphTree,...
    R,i,Q,QMatrix,distNeighbMat,r,eConstant,orphanNodeMat,sensorRange)

IsThereDetectedObstacle=false;
counterObsOrphanNodes=0;
totalNumberOfStaticObs=numel(find(cellfun(@isempty,{obstacle.Orientation}))); % It gives total number of static obstacles
robotPosT=R.robotPose(1:2)';
    
%addObstacle Part
for movingObs= totalNumberOfStaticObs+1:numel(obstacle)
    
    RobotObsDist=norm(obstacle(movingObs).Path(1:2,obstacle(movingObs).ObsTimePoint)- robotPosT); %Distance betwen Robot and Obstacle 
    obsMoveStep=obstacle(movingObs).ObsTimePoint; 
    obsTotMoveStep=numel(obstacle(movingObs).Orientation); % The total number of moving steps of Obstacle
    
    if obstacle(movingObs).Active==1 && obsMoveStep <= obsTotMoveStep...
            && R.moving== 1 && ~obstacle(movingObs).Waiting
        
        if sensorRange  > RobotObsDist
            % Update the cost of the nodes for hard constraint
            obstacle(movingObs).Used=1;
            [node,graphTree,obstacle,Q,QMatrix,orphanNodeMat,counterObsOrphanNodes,...
                distNeighbMat]=HumanFutureCollisionCost(obstacle,movingObs,Q,QMatrix,R,node,...
                nodeList,graphTree,orphanNodeMat,counterObsOrphanNodes,r,i,distNeighbMat,CollisionTolerance);

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

end