function [obstacle,removeObsSample,node,graphTree,Q,QMatrix,...
    distNeighbMat]= updatePedestrianCurrentPos(removeObsSample,...
    obstacle,node,nodeList,graphTree,R,Q,QMatrix,distNeighbMat,r,eConstant,orphanNodeMat,sensorRange)

IsThereDetectedObstacle=false;
counterObsOrphanNodes=0;
totalNumberOfStaticObs=numel(find(cellfun(@isempty,{obstacle.Orientation}))); % It gives total number of static obstacles
robotPosT=R.robotPose(1:2)'; % Robot Position

for movingObs= totalNumberOfStaticObs+1:numel(obstacle)
    
    RobotObsDist=norm(obstacle(movingObs).Path(1:2,obstacle(movingObs).ObsTimePoint)- robotPosT); %Distance betwen Robot and Obstacle 
    obsMoveStep=obstacle(movingObs).ObsTimePoint; 
    obsTotMoveStep=numel(obstacle(movingObs).Orientation); % The total number of moving steps of Obstacle
    
    if obstacle(movingObs).Active==1 && obsMoveStep <= obsTotMoveStep && R.moving== 1
        
        if sensorRange  > RobotObsDist
            % Update the cost of the nodes for hard constraint
            obstacle(movingObs).Used=1;
            [node,graphTree,Q,QMatrix,orphanNodeMat,counterObsOrphanNodes,distNeighbMat]=addPedestrianCurrentPos(movingObs,Q,QMatrix,node,nodeList,graphTree,R,obstacle,orphanNodeMat,counterObsOrphanNodes,distNeighbMat,r);
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
    [graphTree,node,Q,QMatrix]= reduceInconsistency(node,nodeList,graphTree,r,eConstant,Q,QMatrix,distNeighbMat,R);
end



end