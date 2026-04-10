function [cost]= edgeCost(firstNode,secondNode)
% This function calculates the cost of an edge when there is no obstacle.
% The cost calculation such that cost=sqrt(Ct*deltaTime+ ||P1-P2||), where
% ||P1-P2|| is the eucladian distance in spatial distance, deltaTime is 
% the time difference between two nodes. ct is time constant of the cost.

startNode = firstNode;
endNode = secondNode;
distanceConstant=1;
timeConstant=1;

% The cost between the first node and second Node
timeCostSq=timeConstant*(startNode(3)-endNode(3))^2;
distCostSq=distanceConstant*((startNode(1)-endNode(1))^2+(startNode(2)-endNode(2))^2);
cost=sqrt(timeCostSq+distCostSq); 


end