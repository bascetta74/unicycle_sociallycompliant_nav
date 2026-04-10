    function [obsStruct,removeObsSample,node,graphTree,Q,QMatrix,distNeighbMat]= updateObstacles(obstacleOperated,operationType,removeObsSample,obsStruct,node,nodeList,graphTree,R,i,Q,QMatrix,distNeighbMat,param,orphanNodeMat)
%removeObsSample=[];
if operationType < 0

    if obsStruct(obstacleOperated).Used==1
        obsStruct(obstacleOperated).Used=0;
        [node,graphTree,Q,QMatrix,removeObsSample,distNeighbMat]=removeObstacle(obsStruct(obstacleOperated).Object,obsStruct,obstacleOperated,removeObsSample,node,nodeList,graphTree,i,Q,QMatrix,distNeighbMat,param);
    end
%     % Reduce Inconsistency
    [graphTree,node,Q,QMatrix]= reduceInconsistency(node,nodeList,graphTree,param,Q,QMatrix,distNeighbMat,R);
end

if operationType > 0
    counterObsOrphanNodes=0;

        obsStruct(obstacleOperated).Used=1;
        [node,graphTree,Q,QMatrix,orphanNodeMat,counterObsOrphanNodes,distNeighbMat]=addObstacle(obstacleOperated,Q,QMatrix,node,nodeList,graphTree,obsStruct,i,orphanNodeMat,counterObsOrphanNodes,distNeighbMat,param);

    %PropogateDescendants Function
    [node,graphTree,Q,QMatrix]=propogateDescendants(node,graphTree,Q,QMatrix,orphanNodeMat,counterObsOrphanNodes);
%     %VerrifyInQueue(Vbot);
     [node,QMatrix,Q] = verifyInQueue(node,R.robotNode,QMatrix,Q);
%     %reduceInconsistency();
     [graphTree,node,Q,QMatrix]= reduceInconsistency(node,nodeList,graphTree,param,Q,QMatrix,distNeighbMat,R);
end

end