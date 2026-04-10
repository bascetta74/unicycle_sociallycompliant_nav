function [node,graphTree,Q,QMatrix,distNeighbMat]= removeMovingObsPastPosition(obstacle,node,nodeList,R,graphTree,Q,QMatrix,distNeighbMat,param)
% This removes the effect of the pedestrian in the old trajectory set                                                                


IsThereDetectedObstacle=false;
totalNumberOfStaticObs=numel(find(cellfun(@isempty,{obstacle.Orientation}))); % It gives total number of static obstacles
robotPosT=R.robotPose(1:2)';
    
%removeObstacle old position
for movingObs= totalNumberOfStaticObs+1:numel(obstacle)
    
    RobotObsDist=norm(obstacle(movingObs).Path(obstacle(movingObs).ObsTimePoint,1:2)- robotPosT);
    obsMoveStep=obstacle(movingObs).ObsTimePoint;
    obsTotMoveStep=numel(obstacle(movingObs).Orientation);
    
    if obstacle(movingObs).Active==1 && param.sensorRange > RobotObsDist && obsMoveStep <= obsTotMoveStep...
            && R.moving== 1 && ~obstacle(movingObs).Waiting 
%         obstacle(movingObs).Used=1;

        % Remove Moving Obsatacle Function
        [node,graphTree,Q,QMatrix,distNeighbMat]=removeMovingObstacle(obstacle,movingObs,node,nodeList,R,graphTree,Q,QMatrix,distNeighbMat,param);
        
        % verify there is an obstacle in the senor range of the obstalce
        IsThereDetectedObstacle=true;
    end
    
    
    % if there is a detected obstalce, do the reduceInconsistency
    % Operetation
    if IsThereDetectedObstacle==true

    %     % Reduce Inconsistency
        [graphTree,node,Q,QMatrix]= reduceInconsistency(node,nodeList,graphTree,param,Q,QMatrix,distNeighbMat,R);

    end
    
    
end

end