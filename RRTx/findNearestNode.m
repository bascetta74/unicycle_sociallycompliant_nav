function [closestPoint,nnodeDist]=findNearestNode(newSample,nodeList,i)

%Vnearest <<< nearest(v) - Finding the nearest node
nodeListWoutStart= nodeList(:,2:end);
addedNodeT = nodeListWoutStart(:,1:i-2)';

% here I try to find the node with a smaller time value becauce this
% closest node may be the parent node. Therefore, I neglect all nodes whose the
% time values are greater than time vale of newSample
addedNodeT(addedNodeT(:,3)>newSample(3,1),3)=1000;
newSampleT= newSample(:,1)';


% Knn Search.Idx is the index of the closest Node in nodeList
[Idx, nnodeDist]= knnsearch(addedNodeT,newSampleT); 
% nnodeDist is the distance to closest node

% Closest node position 
closestPoint= nodeListWoutStart(:,Idx);




end