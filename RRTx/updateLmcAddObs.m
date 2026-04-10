function [node,graphTree,countUpdateLmc]= updateLmcAddObs(addedObs,poppedNodeIdx,node,r,nodeList,graphTree,distNeighbMat, countUpdateLmc,countRewire)
[node]=cullCurrentNeighbours(poppedNodeIdx,node,r,nodeList,distNeighbMat);
allOutConnect= [node(poppedNodeIdx).RadiusOutConnect, node(poppedNodeIdx).InitialOutConnect];
for i= 1:numel(allOutConnect)
    if node(allOutConnect(i)).MarkedQOB==true
       allOutConnect(1,i)=-100; % Mark them as a minus value and then we delete from the allOutConnect matrice
    end
end
allOutConnect(allOutConnect==-100)=[];
% The start of Update Lmc's for loop for all out Neighbours
updatedParent = false;
%newPoppedParentMat=[];
for kt = 1:numel(allOutConnect)
    checkingOutNode = allOutConnect(1,kt);
    if isinterior(addedObs, nodeList(1,checkingOutNode), nodeList(2,checkingOutNode)) % if the parent of checkingNNode is our prvious cecking node, just pass to other node 
        continue
    end
    if node(poppedNodeIdx).Lmc >= (distNeighbMat(poppedNodeIdx,checkingOutNode) + node(checkingOutNode).Lmc)
    
        newParent = checkingOutNode; % ve newParent matrisi olusturmam gerek
        updatedParent = true; % updatedParentlari yazmam gerek yoksa sifirlaniyor
        %node(poppedNodeIdx).Lmc= norm(nodeList(:,poppedNodeIdx) - nodeList(:,checkingOutNode)) + node(checkingOutNode).Lmc;
        node(poppedNodeIdx).Lmc= distNeighbMat(poppedNodeIdx,checkingOutNode) + node(checkingOutNode).Lmc;
    end                
end
% UpdateLmc-- makeParentof(nodeNeigbour, v)

if updatedParent
    if node(newParent).parent == poppedNodeIdx % if the parent of checkingNNode is our prvious cecking node, just pass to other node 
        graphTree=rmedge(graphTree,int2str(newParent),int2str(poppedNodeIdx));
        node(newParent).parent=[];        
    end
    % If we update parent relation then go inside MakeParentof
    % Function below
    [node,countUpdateLmc,graphTree]= makeParentOf(newParent,poppedNodeIdx,countRewire,nodeList,graphTree,node,distNeighbMat,countUpdateLmc);
end
end