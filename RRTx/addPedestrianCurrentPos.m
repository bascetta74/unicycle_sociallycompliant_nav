function [node,graphTree,Q,QMatrix,orphanNodes,countOrphan,distNeighbMat]=addPedestrianCurrentPos(obsNmbr,Q,QMatrix,node,nodeList,graphTree,R,obstacle,orphanNodes,countOrphan,distNeighbMat,param)
QAddObsNode= MinHeap(250);
QAddObsMatrix=[];
addedObs= obstacle(obsNmbr).Object;
[obsx,obsy]=boundary(addedObs);
[centx,centy]=centroid(addedObs);
% scaleAddedObs= scale(addedObs,1.3,[centx,centy]);
nodeListT=nodeList';
robotCurrentTime=R.robotPose(1,3);
currentStepObs=obstacle(obsNmbr).ObsTimePoint;

% % % % Check whether the obstacle is approaching or not, if it is not appraching
% % % % then don't need this operation
% % % robotCurrentDist=norm(R.robotPose(1,1:2)-obstacle(obsNmbr).Path(1:2,currentStepObs)');
% % % robotPrevDist=norm(R.robotPose(1,1:2)-obstacle(obsNmbr).Path(1:2,currentStepObs-1)');
% % % 
% % % if robotCurrentDist>robotPrevDist && currentStepObs~=1
% % %     robotCurrentDist=norm(R.robotPose(1,1:2)-obstacle(obsNmbr).Path(1:2,currentStepObs)'); % the current distance between robot and obstacle
% % %     robotPrevDist=norm(R.robotPose(1,1:2)-obstacle(obsNmbr).Path(1:2,currentStepObs-1)'); % the previous distance between robot and obstacle
% % %     
% % %     if robotCurrentDist>robotPrevDist % if the robot is getting far, it becomes robotCurrentDist>robotPrevDist, continue
% % %         return           
% % %     end
% % %     
% % % end


%%% Near Nodes around the obstacle is detected at the beginning. The moving
%%% obstacle(human) has 
    if ~isempty(obstacle(obsNmbr).Path)
    NearNodesIdxCell= rangesearch(nodeListT(:,1:2),[centx,centy],param.r);
    scaleAddedObs= scale(addedObs,(param.r/sqrt(2))/obstacle(obsNmbr).Radius,[centx,centy]);
    else
    NearNodesIdxCell= rangesearch(nodeListT(:,1:2),[centx,centy],obstacle(obsNmbr).Radius);
    scaleAddedObs= scale(addedObs,1.3,[centx,centy]);
    end
    NearNodesIdx=cell2mat(NearNodesIdxCell);
    %parentThisNode=node(m).parent;
for it=1:numel(NearNodesIdx)
    m=NearNodesIdx(it);
    intersectedNode1 = isinterior(addedObs, nodeList(1,m), nodeList(2,m));
    
    %intersectX = polyxpoly([nodeList(1,parentThisNode) nodeList(1,m)],[nodeList(2,parentThisNode) nodeList(2,m)], obsx,obsy);
    if intersectedNode1
        checkInAddObsNodeKey= min(node(m).Gv, node(m).Lmc);
        QAddObsNode.InsertKey(checkInAddObsNodeKey);
        checkInNodeGv= [m;checkInAddObsNodeKey;node(m).Gv];
        QAddObsMatrix= [QAddObsMatrix,checkInNodeGv];
    else
        if isinterior(scaleAddedObs, nodeList(1,m), nodeList(2,m))
            poppedNodeNeighb= [node(m).RadiusOutConnect, node(m).InitialOutConnect];
            for i =1:numel(poppedNodeNeighb)
                nextNeighb= poppedNodeNeighb(i);
                if ~isempty(polyxpoly([nodeList(1,nextNeighb) nodeList(1,m)],[nodeList(2,nextNeighb) nodeList(2,m)], obsx,obsy))
                    checkInAddObsNodeKey= min(node(m).Gv, node(m).Lmc);
                    QAddObsNode.InsertKey(checkInAddObsNodeKey);
                    checkInNodeGv= [m;checkInAddObsNodeKey;node(m).Gv];
                    QAddObsMatrix= [QAddObsMatrix,checkInNodeGv];
                    break
%                     if ~isinterior(addedObs, nodeList(1,nextNeighb), nodeList(2,nextNeighb))
%                         distNeighbMat(nextNeighb,m)=inf;
%                         distNeighbMat(m,nextNeighb)=inf;
%                         % removing from graphTree
%                         if node(m).parent==nextNeighb
%                             graphTree=rmedge(graphTree,num2str(m),num2str(nextNeighb));
%                         elseif node(nextNeighb).parent==m
%                             graphTree=rmedge(graphTree,num2str(nextNeighb),num2str(m));
%                         end
%                     end
                end
            end
        end
     end
end

while QAddObsNode.Count >0
    poppedNodeLmc= QAddObsNode.ExtractMin;
    poppedNodeLmc= poppedNodeLmc(1);
    poppedNodeColumn= find(QAddObsMatrix(2,1:end)==poppedNodeLmc);
    poppedNodeColumn = poppedNodeColumn(1);
    poppedNodeIdx= QAddObsMatrix(1,poppedNodeColumn);
    QAddObsMatrix(:,poppedNodeColumn)=[];
    
    % If the robot has more than 2 second time far away(or in the past time) from the node that
    % is conflicted with obstacle neglect this node. The reason for that no
    % need to make calculation for new edge calculation if the moving obstacle human constantly moves in the map.
    timeDifference=robotCurrentTime-nodeList(3,poppedNodeIdx);
    % ObsSpeed gives the speed of the obstacle, if the speed is less than 0.2
    % meter seconds(very slow motion), I would consider the people stop and make collision
    % by asuming the people can stay there for  a while
    if timeDifference < 0 || timeDifference > 1 % && ObsSpeed > 0.2
        continue
    end
    poppedNodeNeighb= [node(poppedNodeIdx).RadiusOutConnect, node(poppedNodeIdx).InitialOutConnect];
    % equating all the intersected Neighbours to Inf
    
    if isinterior(addedObs, nodeList(1,poppedNodeIdx), nodeList(2,poppedNodeIdx))
        for i =1:numel(poppedNodeNeighb)
            nextNeighb= poppedNodeNeighb(i);
%             distNeighbMat(nextNeighb,poppedNodeIdx)=inf;
            distNeighbMat(poppedNodeIdx,nextNeighb)=inf;
            %intersectX = polyxpoly([nodeList(1,nextNeighb) nodeList(1,poppedNodeIdx)],[nodeList(2,nextNeighb) nodeList(2,poppedNodeIdx)], obsx,obsy);
        end
    else
        for i =1:numel(poppedNodeNeighb)
            nextNeighb= poppedNodeNeighb(i);
            if ~isinterior(addedObs, nodeList(1,nextNeighb), nodeList(2,nextNeighb))
                intersectX = polyxpoly([nodeList(1,nextNeighb) nodeList(1,poppedNodeIdx)],[nodeList(2,nextNeighb) nodeList(2,poppedNodeIdx)], obsx,obsy);
                if ~isempty(intersectX)
%                     distNeighbMat(nextNeighb,poppedNodeIdx)=inf;
                    distNeighbMat(poppedNodeIdx,nextNeighb)=inf;
                end
            end
            %intersectX = polyxpoly([nodeList(1,nextNeighb) nodeList(1,poppedNodeIdx)],[nodeList(2,nextNeighb) nodeList(2,poppedNodeIdx)], obsx,obsy);
        end
    end
% VerrifyOrphan

    % operation
    parentThisNode=node(poppedNodeIdx).parent;
    if ~isempty(parentThisNode) && distNeighbMat(poppedNodeIdx,parentThisNode)~= inf
        parentCollison=isinterior(addedObs, nodeList(1,parentThisNode), nodeList(2,parentThisNode)) || isinterior(addedObs, nodeList(1,poppedNodeIdx), nodeList(2,poppedNodeIdx)) || ~isempty(polyxpoly([nodeList(1,parentThisNode) nodeList(1,poppedNodeIdx)],[nodeList(2,parentThisNode) nodeList(2,poppedNodeIdx)], obsx,obsy));
    else
        parentCollison = true;
    end
    
    if ~isempty(parentThisNode) && parentCollison
        
        % buraya bir addedge yapabilirim eger gerek olursa UpdatLmc ile
        if node(poppedNodeIdx).MarkedQ == true
            rmvNodeColumn=find(QMatrix(1,1:end)==poppedNodeIdx);
            rmvNodeLmc=QMatrix(2,rmvNodeColumn);
            tempQMat= Q.Sort;
            QHeapCap= Q.Capacity;
            totalNodeQ= Q.Count;
            for kr= 1:totalNodeQ
                if tempQMat(kr)== rmvNodeLmc
                    tempQMat(kr)=[];
                    break
                end
            end
            Q = MinHeap(QHeapCap,tempQMat);
            QMatrix(:,rmvNodeColumn)=[];
            node(poppedNodeIdx).MarkedQ = false;
        end


        if node(poppedNodeIdx).MarkedQOB ~=true
            node(poppedNodeIdx).MarkedQOB =true;
            countOrphan= countOrphan +1;
            orphanNodes(:,countOrphan)= [poppedNodeIdx;min(node(poppedNodeIdx).Gv,node(poppedNodeIdx).Lmc)];        
        end
%         parentThisNode=node(poppedNodeIdx).parent;
%         graphTree = rmedge(graphTree, {num2str(poppedNodeIdx)},{num2str(parentThisNode)});
%         node(parentThisNode).child(node(parentThisNode).child==poppedNodeIdx)=[];
%         node(poppedNodeIdx).parent = [];
    end
%         node(poppedNodeIdx).parent=[];
%         node(parentThisNode).child(node(parentThisNode).child==poppedNodeIdx)= [];
        % if the PoppedNodeIdx is outside of obstacle, try to find a new
        % parent by looking all around neighbours
%         if ~isinterior(addedObs, nodeList(1,poppedNodeIdx), nodeList(2,poppedNodeIdx))
%             node(poppedNodeIdx).Lmc=Inf;
%             [node,graphTree,countUpdateLmc]= updateLmcAddObs(addedObs,poppedNodeIdx,node,r,nodeList,graphTree,distNeighbMat, countUpdateLmc,countRewire);        
%         end

    % adding this node to Obstacle Collision Nodes Vct
    
%     if node(poppedNodeIdx).MarkedQOB ~=true
%         node(poppedNodeIdx).MarkedQOB =true;
%         countOrphan= countOrphan +1;
%         orphanNodes(:,countOrphan)= [poppedNodeIdx;min(node(poppedNodeIdx).Gv,node(poppedNodeIdx).Lmc)];        
%     end
    
end

end