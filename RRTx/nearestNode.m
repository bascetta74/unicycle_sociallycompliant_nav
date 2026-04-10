function [idxNearest, distance]= nearestNode(nodesL, newNode, listNodeNum)

nodesL = nodesL(:,1:listNodeNum-1);
nodesL = nodesL';
newNode= newNode';

[idxNearest, distance] = knnsearch(nodesL,newNode);

end