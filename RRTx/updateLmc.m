function [node,graphTree]= updateLmc(poppedNodeIdx,node,param,nodeList,graphTree,distNeighbMat)
[node]=cullCurrentNeighbours(poppedNodeIdx,node,param,nodeList,distNeighbMat);
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
    if node(checkingOutNode).parent == poppedNodeIdx % if the parent of checkingNNode is our prvious cecking node, just pass to other node 
         continue
    end
    if ~isempty(node(poppedNodeIdx).parent)
    node(poppedNodeIdx).Lmc = node(node(poppedNodeIdx).parent).Lmc + distNeighbMat(poppedNodeIdx,node(poppedNodeIdx).parent);
    end
    if node(poppedNodeIdx).Lmc > (distNeighbMat(poppedNodeIdx,checkingOutNode) + node(checkingOutNode).Lmc)
        newParent = checkingOutNode; % ve newParent matrisi olusturmam gerek
        updatedParent = true; % updatedParentlari yazmam gerek yoksa sifirlaniyor
        %node(poppedNodeIdx).Lmc= norm(nodeList(:,poppedNodeIdx) - nodeList(:,checkingOutNode)) + node(checkingOutNode).Lmc;
        node(poppedNodeIdx).Lmc= distNeighbMat(poppedNodeIdx,checkingOutNode) + node(checkingOutNode).Lmc;
    end                
end
% UpdateLmc-- makeParentof(nodeNeigbour, v)

if updatedParent
    % If we update parent relation then go inside MakeParentof
    % Function below
    [node,graphTree]= makeParentOf(newParent,poppedNodeIdx,graphTree,node,distNeighbMat);
end
end