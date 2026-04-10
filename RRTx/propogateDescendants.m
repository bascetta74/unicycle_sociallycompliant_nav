function [node,graphTree,Q,QMatrix,orphanNodeMat,counterOrphan]=propogateDescendants(node,graphTree,Q,QMatrix,orphanNodeMat,counterOrphan)
if counterOrphan < 1
    return
end
% First pass for the neighbour of the Vct nodes
orphanNodeList=orphanNodeMat(1,1:counterOrphan); % orphan node list

for i = 1:numel(orphanNodeList)
    currentNode=orphanNodeMat(1,i);
    updatedOrphNodes= node(currentNode).child;
    for k = 1:numel(updatedOrphNodes)
        nextChild=updatedOrphNodes(k);
        if node(nextChild).MarkedQOB ==true
            continue
        else
            counterOrphan=counterOrphan+1;
            orphanNodeMat(:,counterOrphan)= [nextChild;node(nextChild).Lmc];    
            node(nextChild).MarkedQOB=true;
        end
        
%         if node(nextChild).MarkedQ == true
%             rmvNodeColumn=find(QMatrix(1,1:end)==nextChild);
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
%             node(nextChild).MarkedQ = false;
%         end
    end
end

% Second pass for the neighbour of the Vct nodes
orphanNodeList1=orphanNodeMat(1,1:counterOrphan);

for i = 1:numel(orphanNodeList1)
    currentNode1=orphanNodeMat(1,i);
    updatedOutNodes=[node(currentNode1).RadiusOutConnect,node(currentNode1).InitialOutConnect];
    for k=1:numel(updatedOutNodes)
        if node(updatedOutNodes(k)).MarkedQOB
            continue
        end
        % Make RRTTree Cost of this neighbour equals to infinity
        node(updatedOutNodes(k)).Gv= Inf;
        % Then use verrifyQueue operation for this neighbour
        [node,QMatrix,Q] = verifyInQueue(node,updatedOutNodes(k),QMatrix,Q);
    end
    % Second pass for the parent of this node
    currentParent=node(currentNode1).parent;
    if ~isempty(currentParent) && node(currentParent).MarkedQOB ~= true
        node(currentParent).Gv= Inf;
        [node,QMatrix,Q] = verifyInQueue(node,currentParent,QMatrix,Q);
    end
end

%  third pass, remove all nodes from OS, unmark them, and remove
% their connections to their parents, and if one was the robot's target
% then take approperiate measures
i=numel(orphanNodeList1);
while i > 0
    currentNode2=orphanNodeMat(1,counterOrphan);
    % deleting the this node from orphan node List
    orphanNodeMat(:,counterOrphan)=zeros;
    counterOrphan = counterOrphan - 1;
    node(currentNode2).MarkedQOB=false;
    i = i - 1;
    % Equating Gv and Lmc of this node to inf
    node(currentNode2).Gv= inf;
    node(currentNode2).Lmc= inf;
    
    if ~isempty(node(currentNode2).parent)
        currentNode2Parent=node(currentNode2).parent;
        % remove thisNode from its parent's successor list
        childOfParentList= node(currentNode2Parent).child;
        childOfParentList(childOfParentList==currentNode2)=[];
        node(currentNode2Parent).child = childOfParentList;
        % thisNode now has no parent
        node(currentNode2).parent= [];
        graphTree=rmedge(graphTree,num2str(currentNode2),num2str(currentNode2Parent));
    end
    
    
    
    
end


end