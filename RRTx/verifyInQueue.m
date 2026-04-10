function [node,QMatrix,Q] = verifyInQueue(node,checkInNode,QMatrix,Q)
if node(checkInNode).MarkedQ ~= true
    stringcheckInNode= int2str(checkInNode);
    checkInNodeKey= min(node(checkInNode).Gv, node(checkInNode).Lmc);
    Q.InsertKey(checkInNodeKey);
    checkInNodeGv= [checkInNode;checkInNodeKey;node(checkInNode).Gv];
    QMatrix= [QMatrix,checkInNodeGv];
    node(checkInNode).MarkedQ = true;
end

%                     if node(checkInNode).MarkedQ ~= true
%                         checkInNodeKey= min(node(checkInNode).Gv, dLmc(checkInNode,1));
%                         Q.InsertKey(checkInNodeKey);
%                         checkInNodeGv= [checkInNode;checkInNodeKey;node(checkInNode).Gv];
%                         QMatrix= [QMatrix,checkInNodeGv];
%                         node(checkInNode).MarkedQ = true;
%                     end
end