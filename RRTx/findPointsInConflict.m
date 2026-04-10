function NearNodesIdx=findPointsInConflict(ObsNum,obstacle,nodeList,r,totalNode)
pathLength= numel(obstacle(ObsNum).FuturePoint(1,:));
nodeListT=nodeList(:,1:totalNode-1)';
for i = 1:pathLength
    
    if pathLength == 1
        j = 1;
    else
        j = i + 1;
    end
    
    queryPose(1,1)= ( obstacle(ObsNum).FuturePoint(1,i) + obstacle(ObsNum).FuturePoint(1,j))/ 2.0;
    queryPose(1,2)= ( obstacle(ObsNum).FuturePoint(2,i) + obstacle(ObsNum).FuturePoint(2,j))/ 2.0;
    queryPose(1,3)= (obstacle(ObsNum).FuturePoint(3,i) + obstacle(ObsNum).FuturePoint(3,j))/ 2.0;
    
    searchRange = 3 + norm(obstacle(ObsNum).FuturePoint(1:2,i) - obstacle(ObsNum).FuturePoint(1:2,j))/ 2.0;
    
    if i == 1
        NearNodesIdxCell= rangesearch(nodeListT(:,1:2),queryPose(1,1:2),searchRange);
        NearNodesIdx=cell2mat(NearNodesIdxCell);
    else
        NearNodesIdxCell= rangesearch(nodeListT,queryPose,searchRange);        
        NearNodesIdx=[NearNodesIdx,cell2mat(NearNodesIdxCell)];
        NearNodesIdx=unique(NearNodesIdx,'stable');
    end
    
    if j == pathLength
       break 
    end
    
end
end