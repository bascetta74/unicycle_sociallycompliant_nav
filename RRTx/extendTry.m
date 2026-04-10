function [node,graphTree,isNodeAdded,distNeighbMat,nodeList]=extendTry(nodeList,node,saturNode,i,distNeighbMat,graphTree,param,obstacle,robotEdgeDisconnected,R)

% Vnear <<< near(v,r) Finding Vnear operation
nodeListT = nodeList(:,1:i-1)'; % Taking all the nodes to find the nearest node
saturNodeT= saturNode(:,1)';
NearNodesIdxCell= rangesearch(nodeListT,saturNodeT,param.r+0.001); % Finding the near nodes in the circle of r for saturated Node(saturNode)
NearNodesIdx=cell2mat(NearNodesIdxCell);


if (~isempty(NearNodesIdx))

%    node(i).NearNodes= NearNodesIdx;

%FIND PARENT (v,Vnear)
    nearSaturNode=zeros(3,numel(NearNodesIdx));
    node(i).Gv = Inf;
    % % nearStartNodeFound can be deleted if there is time dimension in the system.
    %nearStartNodeFound function is used when there is no time parameter.
    % The reason to not allow exploring the graph from the Goal node. 
    parentFound= false;
    
    collisionFreeEdgesMat = zeros(1,numel(NearNodesIdx));
    for k = 1:numel(NearNodesIdx) % Finding the position of near nodes in nodeList
        nearSaturNode(:,k)= nodeList(:,NearNodesIdx(k));
%         nearDist= norm(nearSaturNode(:,k)-saturNode); % The distance between the saturated Node and its neighbours 
%         distNeighbMat(i,NearNodesIdx(k))=nearDist;
%         distNeighbMat(NearNodesIdx(k),i)=nearDist;
        currentIdx = NearNodesIdx(k);        

        collisionEdge = false;
        CollisionCost=0;
        futureCollisinCost=0;
        totalNumberObs=numel(obstacle);
        
        for kr= 1:totalNumberObs % obstacle avoidance checking if the new connection between saturated node and nearest node
            if obstacle(kr).Used==1
                
                % % This part is for the cost calculation of conflicted
                % edge with the predicted future area 
                
                % Firstly checking collision check with the future
                % prediction area of the obstacle
                if robotEdgeDisconnected==true && obstacle(kr).Moving==1 && obstacle(kr).Detected==1 && obstacle(kr).Waiting~=1
                    [oneEdgeCollision,CollisionCost]=OneEdgeFuturePredictCollision(saturNode,nearSaturNode(:,k),kr,obstacle,obstacle(kr).ObsTimePoint,param.colTolerance,R);
                    if oneEdgeCollision==1
                        if CollisionCost > futureCollisinCost
                            futureCollisinCost=CollisionCost; % Future CollisionCost will be used to assign the cost of this conflicted edge
                        end
                    end
                end
                
                % % This is a regular collision check with the all obstacles inside map. 
                [obsx,obsy]=boundary(obstacle(kr).Object);
%                 edgeCollision= isCollisionFreeEdge(saturNode,nodeList(1:2,currentIdx),obstacle(kr).Object);
                intersectX = polyxpoly([saturNode(1,1) nodeList(1,currentIdx)],[saturNode(2,1) nodeList(2,currentIdx)], obsx,obsy);
                if isempty(intersectX) ~= 1 % if there is a conflict between the possible connection and obstacle
                    distNeighbMat(i,currentIdx)=inf;
                    break
                end
%                 if edgeCollision == 1 % if there is a conflict between the possible connection and obstacle
%                     distNeighbMat(i,currentIdx)=inf;
%                     break
%                 end

            end
        end
        
        
        % For validMove function in time based on the robot Velocity
        edge2dDist= norm(nearSaturNode(1:2,k)-saturNode(1:2,1));
        % Valid move of the robot depends on the 
        validMove= ( (saturNode(3,1)-nearSaturNode(3,k))*R.robotMinVelocity )<= edge2dDist && edge2dDist <= ( (saturNode(3,1)-nearSaturNode(3,k))*R.robotMaxVelocity );
%         validMove=edge2dDist <= ( (saturNode(3,1)-nearSaturNode(3,k))*R.robotMaxVelocity );
            if isempty(intersectX) && validMove
                
                % Edge Cost Calculation
                nearDist=edgeCost(saturNode(:,1),nearSaturNode(:,k)); 
                
                % this is a cost value if the robot's edge connection is lost, this gives futureCollisionCost value to the directly edge
                if futureCollisinCost ==0 
                    distNeighbMat(i,currentIdx)=nearDist;
                else
                    distNeighbMat(i,currentIdx)=futureCollisinCost; % Giving the weighted cost for the conflicted edge
                end
%                 distNeighbMat(currentIdx,i)=nearDist;               
            else
                collisionEdge=true;
                collisionFreeEdgesMat(1,k)=1;
                distNeighbMat(i,currentIdx)=inf;
%                 distNeighbMat(currentIdx,i)=inf;                    
            end
        % To find The optimum and collision free obstacle with this if
        % condition

        
        % finding the parent if neccesary conditions are satisfied
        if (distNeighbMat(i,currentIdx) <= param.r+0.00001) && node(i).Lmc > (distNeighbMat(i,currentIdx) + node(currentIdx).Lmc)  && (~collisionEdge) %% && (~nearStartNodeFound)
            
            parentFound= true;
            node(i).parent = currentIdx;
            node(i).Lmc = distNeighbMat(i,currentIdx) + node(currentIdx).Lmc;

        end
    end
    


    newNode= saturNode;
    % Extend line 3 for parent checking and the rest of lines for extend
    collisionCheckAllEdges= isempty(find(collisionFreeEdgesMat==0,1));
    if ~collisionCheckAllEdges && parentFound
        nodeList(:,i)= newNode(:,1);
        currentParent=node(i).parent;
        stringofParentIdx= int2str(currentParent);            
        stringofNewNode= int2str(i);
        graphTree = addedge(graphTree, {stringofNewNode},{stringofParentIdx},distNeighbMat(i,currentParent));
        % Verify node is added
        % Making the new node the child of its parent
        if ~ismember(i, node(currentParent).child)
            node(currentParent).child = [node(currentParent).child, i];
        else
            numOfChild=numel(node(currentParent).child); 
            node(currentParent).child(1,numOfChild+1)=i; 
        end  
        isNodeAdded = true;

        % rvalues(1,i)=r; % to check the hyperball radius
        for k = 1:numel(NearNodesIdx) % Finding the position of near nodes in nodeList

            nearNode = NearNodesIdx(k);
            % obtacle lari tek tek degerlendirip break koymam gerekecek gibi

            % Checking collision between the connected path of two neighbours and obstacle
            if distNeighbMat(i,nearNode) ~= Inf
                numOutConnect=numel(node(i).InitialOutConnect);
                node(i).InitialOutConnect(1,numOutConnect+1)= nearNode;
                numInConnect=numel(node(nearNode).RadiusInConnect);
                node(nearNode).RadiusInConnect(1,numInConnect+1)=i;
            end
            
            % For validMove function in time
           edge2dDist= norm(nodeList(1:2,nearNode)-newNode(1:2,1));
%             validMove2= edge2dDist <= ( (nodeList(3,nearNode)-newNode(3,1))*R.robotMaxVelocity );
           validMove2=( (nodeList(3,nearNode)-newNode(3,1))*R.robotMinVelocity ) <= edge2dDist && edge2dDist <= ( (nodeList(3,nearNode)-newNode(3,1))*R.robotMaxVelocity );
            if validMove2 == 0
                continue
            end
           edgeCollision2 =false;
            for kr= 1:totalNumberObs % obstacle avoidance checking if the new connection between saturated node and nearest node
                collision=explicitEdgeCheck(kr,obstacle,nodeList(:,nearNode),newNode(:,1));                
                if collision == 1
                    edgeCollision2= true;
                    break
                end
            end
            
            if ~edgeCollision2 && validMove2
                numOutConnect=numel(node(nearNode).RadiusOutConnect);
                node(nearNode).RadiusOutConnect(1,numOutConnect+1)= i;
                numInConnect= numel(node(i).InitialInConnect);
                node(i).InitialInConnect(1,numInConnect+1)= nearNode;
                neighbDist=edgeCost(nodeList(:,nearNode),newNode(:,1)); % The cost between the saturated Node and its neighbours
                distNeighbMat(nearNode,i)=neighbDist;
            end
            % nearNodeDist = norm(nodeList(1,i)-nodeList(1,nearNode));

            % collisionEdgeNeighnour = false;
        end
        % i=i+1;
    else
        isNodeAdded = false;
    end
end


end