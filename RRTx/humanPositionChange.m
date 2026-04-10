function obstacle= humanPositionChange(obstacle,obsNum,simulationTime)
% This is the function for translating the obstacle in the map within its determined
% time interval. It also makes some initilization when an obstacle starts moving


if obstacle(obsNum).StartTime <= simulationTime ...
        && obstacle(obsNum).ObsTimePoint <= numel(obstacle(obsNum).Orientation)

    obstacle(obsNum).Active =1;
    obstacle(obsNum).Used=1;
%     if obstacle(obsNum).ObsTimePoint == numel(obstacle(obsNum).Orientation)
%         obstacle(obsNum).Used=false;
%     end
    xchange=obstacle(obsNum).Path(obstacle(obsNum).ObsTimePoint+1,1)-obstacle(obsNum).Path(obstacle(obsNum).ObsTimePoint,1);
    ychange=obstacle(obsNum).Path(obstacle(obsNum).ObsTimePoint+1,2)-obstacle(obsNum).Path(obstacle(obsNum).ObsTimePoint,2);
    xchange=round(xchange,5);
    ychange=round(ychange,5);
    % If human obstacle waits at the his current place or very small movemments less than 0.3 meter, human is considered as wating so no need to make future predict operation
%     if (ychange==0 && xchange == 0) || 10*sqrt(xchange^2+ychange^2)< 0.3 % If human obstacle waits at the his current place, no need to make operation
%         obstacle(obsNum).Waiting=true;
%         obstacle(obsNum).ObsTimePoint = obstacle(obsNum).ObsTimePoint +1;
%         obstacle(obsNum).ElapseTime=true;
%         return
%     end
    obstacle(obsNum).Waiting=false;
    rotateDeg=atand(ychange/xchange);
    currentRotate=rotateDeg-obstacle(obsNum).RotateDeg;
    [centX,centY]=centroid(obstacle(obsNum).Object);
    obstacle(obsNum).Object=rotate(obstacle(obsNum).Object,currentRotate,[centX,centY]);
    obstacle(obsNum).Object=translate((obstacle(obsNum).Object),xchange,ychange);
    obstacle(obsNum).RotateDeg=rotateDeg;
    obstacle(obsNum).ObsTimePoint = obstacle(obsNum).ObsTimePoint +1;
    obstacle(obsNum).ElapseTime=true;

end

end