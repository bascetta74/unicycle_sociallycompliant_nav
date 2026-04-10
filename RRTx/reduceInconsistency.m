function [graphTree,node,Q,QMatrix]= reduceInconsistency(node,nodeList,graphTree,param,Q,QMatrix,distNeighbMat,R)

while Q.Count > 0 && (Q.ReturnMin <= min(node(R.robotNode).Gv, node(R.robotNode).Lmc) + 10  || node(R.robotNode).Lmc~=node(R.robotNode).Gv || node(R.robotNode).Gv==inf ||node(R.robotNode).MarkedQ==true)
    poppedNodeLmc= Q.ExtractMin;
    poppedNodeLmc= poppedNodeLmc(1);
    [poppedNoderow, poppedNodeColumn]= find(QMatrix(2,1:end)==poppedNodeLmc);
    poppedNodeColumn = poppedNodeColumn(1);
    poppedNodeIdx= QMatrix(1,poppedNodeColumn);
    poppedNodeIdx = poppedNodeIdx(1);
    node(poppedNodeIdx).MarkedQ = false;
    QMatrix(:,poppedNodeColumn)=[];
    %poppedNodeMat= [poppedNodeMat,poppedNodeIdx];        
    if node(poppedNodeIdx).Gv - node(poppedNodeIdx).Lmc > param.eConstant
% UpdateLMC(v)

        [node,graphTree]= updateLmc(poppedNodeIdx,node,param,nodeList,graphTree,distNeighbMat);

%Rewire(poppedNodeIdx) - Making rewire operation for the poppedNode

        [node,QMatrix,Q,graphTree,distNeighbMat]= rewire(poppedNodeIdx,param,nodeList,node,graphTree,Q,QMatrix,distNeighbMat);

    end
     node(poppedNodeIdx).Gv = node(poppedNodeIdx).Lmc;
end     
    

end