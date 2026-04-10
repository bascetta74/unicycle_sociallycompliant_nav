function [node,QMatrix,Q,graphTree,distNeighbMat]= rewire(currentNodeIdx,param,nodeList,node,graphTree,Q,QMatrix,distNeighbMat)
    stringPoppedNodeIdx= int2str(currentNodeIdx);
    if node(currentNodeIdx).Gv - node(currentNodeIdx).Lmc > param.eConstant
        [node]=cullCurrentNeighbours(currentNodeIdx,node,param,nodeList,distNeighbMat);
        allInConnect= [node(currentNodeIdx).RadiusInConnect, node(currentNodeIdx).InitialInConnect];

        %Check all inside neighbours connections u el N-(V)\{Pt+(v)}
        for kin= 1:numel(allInConnect)
            checkInNode = allInConnect(1,kin);
            stringCheckInNode=int2str(checkInNode);

            if node(currentNodeIdx).parent == checkInNode
                continue
            end
            
%             if distNeighbMat(checkInNode,currentNodeIdx)==inf
%                 continue
%             end

            if node(checkInNode).Lmc > ( distNeighbMat(checkInNode,currentNodeIdx) + node(currentNodeIdx).Lmc )
                % equating Lmc value of the neighnour
                node(checkInNode).Lmc = ( distNeighbMat(checkInNode,currentNodeIdx) + node(currentNodeIdx).Lmc ); % Gereksiz bir islem olabilir benim alogritma icin

                %Make parentof(currentNodeIdx, checkInNode)
                [node,graphTree]= makeParentOf(currentNodeIdx,checkInNode,graphTree,node,distNeighbMat);
                
                if node(checkInNode).Gv - node(checkInNode).Lmc  > param.eConstant
                    %verrify Queue(u)
                    [node,QMatrix,Q] = verifyInQueue(node,checkInNode,QMatrix,Q);
                end
            end
        end
    end
end