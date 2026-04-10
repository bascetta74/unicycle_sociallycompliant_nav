function [node,graphTree,Q,QMatrix,orphanNodes,countOrphan,distNeighbMat]=addObstacleWeight(obsNmbr,Q,QMatrix,node,nodeList,graphTree,obstacle,orphanNodes,countOrphan,distNeighbMat,timePoint)
QAddObsNode= MinHeap(250);
QAddObsMatrix=[];
addedObs= obstacle(obsNmbr).Object;
%intersectedNode1 = isinterior(checkobs, nodeList(1,1:totalNodeNum-1), nodeList(2,1:totalNodeNum-1));
[obsx,obsy]=boundary(addedObs);
[centx,centy]=centroid(addedObs);
theta=obstacle(obsNmbr).Orientation(timePoint);
rotMat=[cosd(-theta), -sind(-theta);sind(-theta), cosd(-theta)];
scaleAddedObs= obstacle(obsNmbr).WeightArea{timePoint};
[obsScaledx,obsScaledy]=boundary(scaleAddedObs);
nodeListT=nodeList';
% for m= 1:totalNodeNum-1
%     if m==2
%         continue
%     end
%     
%     NearNodesIdxCell= rangesearch(nodeListT(:,1:2),[centx,centy],weightScaleArea*obstacle(obsNmbr).Radius);
    NearNodesIdxCell= rangesearch(nodeListT(:,1:2),[centx,centy],4);
    NearNodesIdx=cell2mat(NearNodesIdxCell);
    %parentThisNode=node(m).parent;
for it=1:numel(NearNodesIdx)
    m=NearNodesIdx(it);
    intersectedNode1 = isinterior(addedObs, nodeList(1,m), nodeList(2,m));
    
    %intersectX = polyxpoly([nodeList(1,parentThisNode) nodeList(1,m)],[nodeList(2,parentThisNode) nodeList(2,m)], obsx,obsy);
    if intersectedNode1
        continue
    else
        checkInAddObsNodeKey= min(node(m).Gv, node(m).Lmc);
        QAddObsNode.InsertKey(checkInAddObsNodeKey);
        checkInNodeGv= [m;checkInAddObsNodeKey;node(m).Gv];
        QAddObsMatrix= [QAddObsMatrix,checkInNodeGv];
%         if isinterior(scaleAddedObs, nodeList(1,m), nodeList(2,m))
%             poppedNodeNeighb= [node(m).RadiusOutConnect, node(m).InitialOutConnect];
%             for i =1:numel(poppedNodeNeighb)
%                 nextNeighb= poppedNodeNeighb(i);
%                 if ~isempty(polyxpoly([nodeList(1,nextNeighb) nodeList(1,m)],[nodeList(2,nextNeighb) nodeList(2,m)], obsx,obsy))
%                     checkInAddObsNodeKey= min(node(m).Gv, node(m).Lmc);
%                     QAddObsNode.InsertKey(checkInAddObsNodeKey);
%                     checkInNodeGv= [m;checkInAddObsNodeKey;node(m).Gv];
%                     QAddObsMatrix= [QAddObsMatrix,checkInNodeGv];
%                 end
%             end
%         end
     end
end

while QAddObsNode.Count >0
    poppedNodeLmc= QAddObsNode.ExtractMin;
    poppedNodeLmc= poppedNodeLmc(1);
    poppedNodeColumn= find(QAddObsMatrix(2,1:end)==poppedNodeLmc);
    poppedNodeColumn = poppedNodeColumn(1);
    poppedNodeIdx= QAddObsMatrix(1,poppedNodeColumn);
    QAddObsMatrix(:,poppedNodeColumn)=[];
    prevParentNodeDist=distNeighbMat(poppedNodeIdx,node(poppedNodeIdx).parent);
    
    % The position of PoppedNodeInd in the golabal frame for determining
    % weight cost with respect to grid system in global frame
    globPoppedPos=rotMat*(nodeList(1:2,poppedNodeIdx)-[centx;centy]);
    point = [globPoppedPos(1), globPoppedPos(2)];
    xgrid= -2:0.2:3;
    ygrid= -2:0.2:2;
    if xgrid(1) < globPoppedPos(1) && globPoppedPos(1) < xgrid(end) && ygrid(1)< globPoppedPos(2) && globPoppedPos(2) < ygrid(end)
     % if the node position inside of our weighted obstacle figure
        [x_gridPop] = find(xgrid <= globPoppedPos(1), 1, 'last');
        [y_gridPop] = find(ygrid <= globPoppedPos(2), 1, 'last');
        poppedNWeight=(20-abs(x_gridPop-11)-abs(y_gridPop-11));
    else
        poppedNWeight=3;
    end
    
    poppedNodeNeighb= [node(poppedNodeIdx).RadiusOutConnect, node(poppedNodeIdx).InitialOutConnect];
    % equating all the intersected Neighbours to Inf
    for i =1:numel(poppedNodeNeighb)
        nextNeighb= poppedNodeNeighb(i);
        % NeighbourNode weight cost
        globNeighbPos=rotMat*(nodeList(1:2,nextNeighb)-[centx;centy]);
    if xgrid(1) < globNeighbPos(1) &&  globNeighbPos(1) < xgrid(end) && ygrid(1)< globNeighbPos(2) && globNeighbPos(2) < ygrid(end)
     % if the node position inside of our weighted obstacle figure
        [x_gridNegb] = find(xgrid <= globNeighbPos(1), 1, 'last');
        [y_gridNegb] = find(ygrid <= globNeighbPos(2), 1, 'last');
        NeighbNWeight=(20-abs(x_gridNegb-11)-abs(y_gridNegb-11));
    else
        NeighbNWeight=3;
    end
        
        %check first if the next neighBour intersects with real obstacle
        if isinterior(addedObs, nodeList(1,nextNeighb), nodeList(2,nextNeighb))
            continue
        end
        intersectX = polyxpoly([nodeList(1,nextNeighb) nodeList(1,poppedNodeIdx)],[nodeList(2,nextNeighb) nodeList(2,poppedNodeIdx)], obsx,obsy);
        if ~isempty(intersectX)
            continue
        end
        
        % assigning weight costs for different nodes based on the location
        % whether inside of ScaleAddedObs or not
        if ~isinterior(scaleAddedObs, nodeList(1,nextNeighb), nodeList(2,nextNeighb))
            % case with Neighbour Node is outside of ScaledWieght Obstacle area
            
            if ~isinterior(scaleAddedObs, nodeList(1,poppedNodeIdx), nodeList(2,poppedNodeIdx))
                intersectScaledX = polyxpoly([nodeList(1,nextNeighb) nodeList(1,poppedNodeIdx)],[nodeList(2,nextNeighb) nodeList(2,poppedNodeIdx)], obsScaledx,obsScaledy);
                if isempty(intersectScaledX)
                    continue
                else
                    % Case with Neighbour Outside, poppedNeigh Outside
                    distNeighbMat(poppedNodeIdx,nextNeighb)=((NeighbNWeight+poppedNWeight)/2)*norm(nodeList(:,poppedNodeIdx)-nodeList(:,nextNeighb));
%                         distNeighbMat(nextNeighb,poppedNodeIdx)=distNeighbMat(poppedNodeIdx,nextNeighb);
                end
            else
                % Case with Neighbour Outside, poppedNeigh Inside
                distNeighbMat(poppedNodeIdx,nextNeighb)=((NeighbNWeight+poppedNWeight)/2)*norm(nodeList(:,poppedNodeIdx)-nodeList(:,nextNeighb));
%                 distNeighbMat(nextNeighb,poppedNodeIdx)=distNeighbMat(poppedNodeIdx,nextNeighb);
            end
            
        else % case with Neighbour Node is inside of ScaledWieght Obstacle area
            
            if ~isinterior(scaleAddedObs, nodeList(1,poppedNodeIdx), nodeList(2,poppedNodeIdx))
                intersectScaledX = polyxpoly([nodeList(1,nextNeighb) nodeList(1,poppedNodeIdx)],[nodeList(2,nextNeighb) nodeList(2,poppedNodeIdx)], obsScaledx,obsScaledy);
                if isempty(intersectScaledX)
                    continue
                else
                    % Case with Neighbour Inside, poppedNeigh Out
                    distNeighbMat(poppedNodeIdx,nextNeighb)=((NeighbNWeight+poppedNWeight)/2)*norm(nodeList(:,poppedNodeIdx)-nodeList(:,nextNeighb));
%                         distNeighbMat(nextNeighb,poppedNodeIdx)=distNeighbMat(poppedNodeIdx,nextNeighb);
                end
                
            else
                % Case with Neighbour Inside, poppedNeigh Inside
                distNeighbMat(poppedNodeIdx,nextNeighb)=((poppedNWeight+NeighbNWeight)/2)*norm(nodeList(:,poppedNodeIdx)-nodeList(:,nextNeighb));
            end
            
        end
            %intersectX = polyxpoly([nodeList(1,nextNeighb) nodeList(1,poppedNodeIdx)],[nodeList(2,nextNeighb) nodeList(2,poppedNodeIdx)], obsx,obsy);
        
    end
    
    % This is the case for the children of nodes PoppedNode which the point
    % is inside but its children are far enough to cover 
    if isinterior(scaleAddedObs, nodeList(1,poppedNodeIdx), nodeList(2,poppedNodeIdx))
         poppedNodeChilds= node(poppedNodeIdx).child;
         for k=1:numel(poppedNodeChilds)
             poppedChild=poppedNodeChilds(k);
             distNeighbMat(poppedChild,poppedNodeIdx)=(poppedNWeight+3)*norm(nodeList(:,poppedNodeIdx)-nodeList(:,poppedChild));
         end
    end
    
    makeMarkedQOperation=false;
    if prevParentNodeDist~=distNeighbMat(poppedNodeIdx,node(poppedNodeIdx).parent)
        graphTree = rmedge(graphTree,{int2str(poppedNodeIdx)}, {int2str(node(poppedNodeIdx).parent)}); % removing old parent relation
        graphTree = addedge(graphTree, {int2str(poppedNodeIdx)},{int2str(node(poppedNodeIdx).parent)},distNeighbMat(poppedNodeIdx,node(poppedNodeIdx).parent));
        makeMarkedQOperation=true;
    end
% VerrifyOrphan

    % operation
%     parentThisNode=node(poppedNodeIdx).parent;
%     if ~isempty(parentThisNode) && distNeighbMat(poppedNodeIdx,parentThisNode)~=inf


        % buraya bir addedge yapabilirim eger gerek olursa UpdatLmc ile
%         if node(poppedNodeIdx).MarkedQ == true
%             rmvNodeColumn=find(QMatrix(1,1:end)==poppedNodeIdx);
%             rmvNodeLmc=QMatrix(2,rmvNodeColumn);
%             tempQMat= Q.Sort;
%             QHeapCap= Q.Capacity;
%             totalNodeQ= Q.Count;
%             for kr= 1:totalNodeQ
%                 if tempQMat(kr)== rmvNodeLmc
%                     tempQMat(kr)=[];
%                     break
%                 end
%             end
%             Q = MinHeap(QHeapCap,tempQMat);
%             QMatrix(:,rmvNodeColumn)=[];
%             node(poppedNodeIdx).MarkedQ = false;
%         end
        

        if node(poppedNodeIdx).MarkedQOB ~=true && makeMarkedQOperation == 1
            node(poppedNodeIdx).MarkedQOB =true;
            countOrphan= countOrphan +1;
            orphanNodes(:,countOrphan)= [poppedNodeIdx;min(node(poppedNodeIdx).Gv,node(poppedNodeIdx).Lmc)];        
        end
%         parentThisNode=node(poppedNodeIdx).parent;
%         graphTree = rmedge(graphTree, {num2str(poppedNodeIdx)},{num2str(parentThisNode)});
%     end
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