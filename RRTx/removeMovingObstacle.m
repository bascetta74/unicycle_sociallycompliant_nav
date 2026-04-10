function [node,graphTree,Q,QMatrix,distNeighbMat]=removeMovingObstacle(obstacle,rmvObsNmbr,node,nodeList,R,graphTree,Q,QMatrix,distNeighbMat,r)
         
% checkobs=obstacle(rmvObsNmbr).Object;
obsTimePoint=obstacle(rmvObsNmbr).ObsTimePoint;

prevObsTimePoint=obsTimePoint-20;
if prevObsTimePoint<1
    return;
end
% theta=obstacle(rmvObsNmbr).Orientation(prevObsTimePoint);
% rotMat=[cosd(theta), -sind(theta);sind(theta), cosd(theta)];
prevObsPos(1:2,1)=obstacle(rmvObsNmbr).Path(prevObsTimePoint,1:2);

if R.numRobotMovePoints > 2
    robotCurrentTime=R.robotPose(1,3); % this is the current time value of Robot, based on this robot shouldn't pass 1.8 seconds later in the center of this future rectangel obs area
    prevObsPos(3,1)=robotCurrentTime-2;
else
    prevObsPos(3,1)=nodeList(3,1);
end


totalNumberObs=numel([obstacle(1:end).Used]);


% removeObstacle starting

%QRemoveObsMatrix=[];
%intersectedNode1 = isinterior(checkobs, nodeList(1,1:totalNodeNum-1), nodeList(2,1:totalNodeNum-1));
% allQueryNodes=findPointsInConflictRemove(rmvObsNmbr,obstacle,nodeList,r,totalNodeNum);
nodeListT=nodeList';
queryPose=prevObsPos';
searchRange=3;
NearNodesIdxCell= rangesearch(nodeListT,queryPose,searchRange);
NearNodesIdx=cell2mat(NearNodesIdxCell);

queryGvLmc=[node(NearNodesIdx).Gv;node(NearNodesIdx).Lmc];
minObsNodesKey= min(queryGvLmc,[],1);

QObsNode= MinHeap(1000,minObsNodesKey);
QRemoveObsMatrix = [NearNodesIdx;minObsNodesKey;node(NearNodesIdx).Gv];
% for m= 1:totalNodeNum-1
%     
%     intersectedNode1 = isinterior(checkobs, nodeList(1,m), nodeList(2,m));
%     if intersectedNode1
%         checkInRemoveObsNodeKey= min(node(m).Gv, node(m).Lmc);
%         QObsNode.InsertKey(checkInRemoveObsNodeKey);
%         checkInNodeGv= [m;checkInRemoveObsNodeKey;node(m).Gv];
%         QRemoveObsMatrix= [QRemoveObsMatrix,checkInNodeGv];
%     end
% end

while QObsNode.Count >0
    poppedNodeLmc= QObsNode.ExtractMin;
    poppedNodeLmc= poppedNodeLmc(1);
    [~, poppedNodeColumn]= find(QRemoveObsMatrix(2,1:end)==poppedNodeLmc);
    poppedNodeColumn = poppedNodeColumn(1);
    poppedNodeIdx= QRemoveObsMatrix(1,poppedNodeColumn);
    %stringPoppedNodeIdx=int2str(poppedNodeIdx);
    QRemoveObsMatrix(:,poppedNodeColumn)=[];
    isThisRootNode=false; % This is for preventing the operation for the root nodes. If this operation was made, after removing obstacle function, the cost between the root node won't be zero as supposed to be our system rquirements.
    
    if nodeList(1:2,poppedNodeIdx)==nodeList(1:2,2)
       isThisRootNode=true; 
    end
    
    poppedNodeNeighb= [node(poppedNodeIdx).RadiusOutConnect,node(poppedNodeIdx).InitialOutConnect];
%     node(poppedNodeIdx).DistancesRemObsNodeOut(1,:)=outConnectNeighb;
    for ir = 1:numel(poppedNodeNeighb)
        nextNeighb= poppedNodeNeighb(ir);
        if isThisRootNode==1 && nodeList(1,nextNeighb)==nodeList(1,2) && nodeList(2,nextNeighb)==nodeList(2,2)% Root node connection cost remains always zero by this way
            continue
        end
        if distNeighbMat(poppedNodeIdx,nextNeighb)==edgeCost(nodeList(:,poppedNodeIdx),nodeList(:,nextNeighb))
%             edgeCost(nodeList(:,poppedNodeIdx),nodeList(:,nextNeighb),CostTimeConstant)
            continue
        end
%         collisionEdge=explicitEdgeCheck(rmvObsNmbr,obstacle,nodeList(:,poppedNodeIdx),nodeList(:,nextNeighb));
        collisionEdge=true;
        intersectedNode=false;
        if collisionEdge        
            for k= 1:totalNumberObs
                if obstacle(k).Used==1 && obstacle(k).Moving ~= 1
                    checkobs=obstacle(k).Object;
                    intersectedNode = isinterior(checkobs, nodeList(1,nextNeighb), nodeList(2,nextNeighb));
                    if intersectedNode
                        break
                    end
                end
            end
        end
        if distNeighbMat(poppedNodeIdx,nextNeighb)~=edgeCost(nodeList(:,nextNeighb),nodeList(:,poppedNodeIdx)) && ~intersectedNode
            distNeighbMat(poppedNodeIdx,nextNeighb)=edgeCost(nodeList(:,nextNeighb),nodeList(:,poppedNodeIdx));
            %distNeighbMat(poppedNodeIdx,nextNeighb)=distNeighbMat(nextNeighb,poppedNodeIdx);
        end
%         node(poppedNodeIdx).DistancesRemObsNodeOut(2,ir)= norm(nodeList(:,poppedNodeIdx)-nodeList(:,outConnectNeighb(ir)));
    end
%     
    [node,graphTree]= updateLmc(poppedNodeIdx,node,r,nodeList,graphTree,distNeighbMat);
    
    if node(poppedNodeIdx).Lmc ~= node(poppedNodeIdx).Gv
        [node,QMatrix,Q] = verifyInQueue(node,poppedNodeIdx,QMatrix,Q);
    end
    
end

end