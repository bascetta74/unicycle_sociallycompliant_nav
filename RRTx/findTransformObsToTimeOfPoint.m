function collision=findTransformObsToTimeOfPoint(checkedNode,ObsNum,obstacle)
if numel(obstacle(ObsNum).Path(1,:)) < 1
    dx=0;
    dy=0;
else
    totalInd=numel(obstacle(ObsNum).Path(1,:));
    indBef=1;
    while indBef + 1 <= totalInd && obstacle(ObsNum).Path(3,indBef+1) < checkedNode(3)
        indBef= indBef+1;
    end

    if indBef == totalInd
        dx= obstacle(ObsNum).Path(1,end);
        dy= obstacle(ObsNum).Path(2,end);
    else 
        indAft = indBef +1;
        ratioInd= (checkedNode(3,1) - obstacle(ObsNum).Path(3,indBef)) / ( obstacle(ObsNum).Path(3,indAft) - obstacle(ObsNum).Path(3,indBef));

        dx= obstacle(ObsNum).Path(1,indBef) + ratioInd* ( obstacle(ObsNum).Path(1,indAft) - obstacle(ObsNum).Path(1,indBef));
        dy= obstacle(ObsNum).Path(2,indBef) + ratioInd* ( obstacle(ObsNum).Path(2,indAft) - obstacle(ObsNum).Path(2,indBef));
    end
end
[centX,centY]= centroid(obstacle(ObsNum).Object);
queryX=centX-obstacle(ObsNum).PosX+dx;
queryY=centY-obstacle(ObsNum).PosY+dy;
tempObs=translate(obstacle(ObsNum).Object,queryX,queryY);

collision=isinterior(tempObs,checkedNode(1),checkedNode(2));

end