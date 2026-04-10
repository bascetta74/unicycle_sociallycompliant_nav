% if C-Space has a time dimension, add a sequence of descendents
% to the root, where each {(great)^n}-grandchild is at the same
% position as root, but at a sequence of times from 0 to the last ("earliest")
% time that the robot could arrive at that position (assuming no obstacles)
% but at a tree distance defined to be 0 from the root. 
% this helps the robot to reach the goal location as quickly as possible instead of burning time

function[nodeList,graphTree,node,addedNode,distNeighbMat]= addOtherTimesToRoot(nodeList,node,graphTree,R,distNeighbMat)
insertStep = 2;
lastNodeNum= 2;
previosNodeNum = 2;
addedNode=0;
node(2).Gv=0.001;

lastTimetoInsert = nodeList(3,1)- norm(nodeList(1:2,1)-nodeList(1:2,2))/R.robotMaxVelocity;
firstTimetoInsert = nodeList(3,2) + insertStep;

for timeToInsert = firstTimetoInsert:insertStep:lastTimetoInsert
    newPose= nodeList(:,2);
    
    newPose(3,1)= timeToInsert;
    
    lastNodeNum = lastNodeNum + 1;
    nodeList(:,lastNodeNum)= newPose;
    distNeighbMat(lastNodeNum,previosNodeNum)=0;
    
    graphTree=addedge(graphTree,int2str(lastNodeNum),int2str(lastNodeNum-1),0);
    
    % make parent operation
    node(lastNodeNum).parent = previosNodeNum;
    node(previosNodeNum).child = lastNodeNum;
    
    % create neighbooor relations
    node(lastNodeNum).InitialOutConnect= previosNodeNum;
    node(previosNodeNum).InitialInConnect=lastNodeNum;
    
    node(lastNodeNum).Lmc = 0;
    node(lastNodeNum).Gv = 0;
    
    previosNodeNum= lastNodeNum;
    addedNode = addedNode +1;
    
end


end