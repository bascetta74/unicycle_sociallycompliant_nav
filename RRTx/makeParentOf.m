function [node,graphTree]= makeParentOf(parentNodeIdx,childNodeIdx,graphTree,node,distNeighbMat)
%emptyParent=isempty(node(childNodeIdx).parent);
%  if node(childNodeIdx).parent ~= parentNodeIdx
    oldcheckInNode = node(childNodeIdx).parent;
    noOldParent= isempty(oldcheckInNode);
    distanceParent = distNeighbMat(childNodeIdx,parentNodeIdx);
    if ~noOldParent
    graphTree = rmedge(graphTree,{int2str(childNodeIdx)}, {int2str(oldcheckInNode)}); % removing old parent relation
    node(oldcheckInNode).child(node(oldcheckInNode).child==childNodeIdx)=[];    
    end
    graphTree = addedge(graphTree, {int2str(childNodeIdx)},{int2str(parentNodeIdx)},distanceParent);
    node(childNodeIdx).parent = parentNodeIdx;
    node(childNodeIdx).Lmc= node(parentNodeIdx).Lmc + distanceParent;
    if ~ismember(childNodeIdx, node(parentNodeIdx).child)
        node(parentNodeIdx).child = [node(parentNodeIdx).child, childNodeIdx];
    end                            

end