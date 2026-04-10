function [node]= cullCurrentNeighbours(poppedNodeIdx,node,param,nodeList,distNeighbMat)
    NearOutNode=node(poppedNodeIdx).RadiusOutConnect;
    temporaryOutConnect = node(poppedNodeIdx).RadiusOutConnect;
    for k= 1:numel(NearOutNode)
        %distance = norm(nodeList(:,poppedNodeIdx)-nodeList(:,NearOutNode(k)));
        distance = distNeighbMat(poppedNodeIdx,NearOutNode(k));
        if param.r*20 < distance && (isempty(node(poppedNodeIdx).parent) ||node(poppedNodeIdx).parent ~= NearOutNode(k))
            % Nr+(v) << Nr+(v)\ u Algorithm 3 line 3
            %denemem = denemem +1;
            temporaryOutConnect(temporaryOutConnect == NearOutNode(k))= [];
            % Nr-(u) << Nr+(u)\ v Algorithm 3 line 4
%             node(NearOutNode(k)).InitialInConnect(node(NearOutNode(k)).InitialInConnect == poppedNodeIdx)= [];
        end
    end
    node(poppedNodeIdx).RadiusOutConnect= temporaryOutConnect;
end