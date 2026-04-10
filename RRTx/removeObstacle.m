function [node,graphTree,Q,QMatrix,sampleOb,distNeighbMat]=removeObstacle(removedObst,obstacle,rmvObsNmbr,sampleOb,node,nodeList,graphTree,totalNodeNum,Q,QMatrix,distNeighbMat,param)
checkobs=removedObst;
totalNumberObs=numel([obstacle(1:end).Used]);

if obstacle(rmvObsNmbr).Used==0
%Sampling for new nodes in the empty space of Removed Obstacle 
    [borderx,bordery]= boundingbox(checkobs);
    numObsSample= round(area(checkobs)/10);
    iobs=0;
    sampleObAdd=zeros(2,numObsSample);
    sampleOb=[sampleOb,sampleObAdd];
    while iobs<numObsSample
        sampleTryOb= [borderx(1)+randn()*(borderx(2)-borderx(1));
        bordery(1)+randn()*(bordery(2)-bordery(1))];
        if isinterior(checkobs, sampleTryOb(1,1), sampleTryOb(2,1))
            iobs=iobs+1;
            sampleOb(:,iobs)=sampleTryOb;
        end
    end
end
% removeObstacle starting
QObsNode= MinHeap(300);
QRemoveObsMatrix=[];
%intersectedNode1 = isinterior(checkobs, nodeList(1,1:totalNodeNum-1), nodeList(2,1:totalNodeNum-1));
for m= 1:totalNodeNum-1
    intersectedNode1 = isinterior(checkobs, nodeList(1,m), nodeList(2,m));
    if intersectedNode1
        checkInRemoveObsNodeKey= min(node(m).Gv, node(m).Lmc);
        QObsNode.InsertKey(checkInRemoveObsNodeKey);
        checkInNodeGv= [m;checkInRemoveObsNodeKey;node(m).Gv];
        QRemoveObsMatrix= [QRemoveObsMatrix,checkInNodeGv];
    end
end

while QObsNode.Count >0
    poppedNodeLmc= QObsNode.ExtractMin;
    poppedNodeLmc= poppedNodeLmc(1);
    [poppedNoderow, poppedNodeColumn]= find(QRemoveObsMatrix(2,1:end)==poppedNodeLmc);
    poppedNodeColumn = poppedNodeColumn(1);
    poppedNodeIdx= QRemoveObsMatrix(1,poppedNodeColumn);
    %stringPoppedNodeIdx=int2str(poppedNodeIdx);
    QRemoveObsMatrix(:,poppedNodeColumn)=[];
    
    poppedNodeNeighb= [node(poppedNodeIdx).RadiusOutConnect,node(poppedNodeIdx).InitialOutConnect];
%     node(poppedNodeIdx).DistancesRemObsNodeOut(1,:)=outConnectNeighb;
    for ir = 1:numel(poppedNodeNeighb)
        nextNeighb= poppedNodeNeighb(ir);
        for k= 1:totalNumberObs
            if obstacle(k).Used==1
                checkobs=obstacle(k).Object;
                intersectedNode = isinterior(checkobs, nodeList(1,nextNeighb), nodeList(2,nextNeighb));
                if intersectedNode
                    break
                end
            end
        end
        
        if distNeighbMat(poppedNodeIdx,nextNeighb)==inf && ~intersectedNode
%             distNeighbMat(poppedNodeIdx,nextNeighb)=norm(nodeList(:,nextNeighb)-nodeList(:,poppedNodeIdx));
            distNeighbMat(poppedNodeIdx,nextNeighb)=edgeCost(nodeList(:,nextNeighb),nodeList(:,poppedNodeIdx));
            %distNeighbMat(poppedNodeIdx,nextNeighb)=distNeighbMat(nextNeighb,poppedNodeIdx);
        end
%         node(poppedNodeIdx).DistancesRemObsNodeOut(2,ir)= norm(nodeList(:,poppedNodeIdx)-nodeList(:,outConnectNeighb(ir)));
    end
%     
    [node,graphTree]= updateLmc(poppedNodeIdx,node,param,nodeList,graphTree,distNeighbMat);
    
    if node(poppedNodeIdx).Lmc ~= node(poppedNodeIdx).Gv
        [node,QMatrix,Q] = verifyInQueue(node,poppedNodeIdx,QMatrix,Q);
    end
    
end

end