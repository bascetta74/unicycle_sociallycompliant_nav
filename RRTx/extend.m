function [node,graphTree,isNodeAdded,distNeighbMat,nodeList]=extend(nodeList,node,saturNode,cardin,distNeighbMat,graphTree,param,obstacle,robotEdgeDisconnected,R)

% Vnear <<< near(v,r) Finding Vnear operation
nodeListT = nodeList(:,1:cardin-1)'; % Taking all the nodes to find the nearest node
saturNodeT= saturNode(:,1)';
NearNodesIdxCell= rangesearch(nodeListT,saturNodeT,param.r+0.001); % Finding the near nodes in the circle of r for saturated Node(saturNode)
NearNodesIdx=cell2mat(NearNodesIdxCell);

% if there is not any near node to the saturNode, then skip the rest of
% operation by setting isNodeAdded to false and exit from this function
if isempty(NearNodesIdx)

    isNodeAdded=false;
    return    
end


%    node(cardin).NearNodes= NearNodesIdx;

%FIND PARENT (v,Vnear)
nearSaturNode=zeros(3,numel(NearNodesIdx));
node(cardin).Gv = Inf;
% % nearStartNodeFound can be deleted if there is time dimension in the system.
%nearStartNodeFound function is used when there is no time parameter.
% The reason to not allow exploring the graph from the Goal node. 
parentFound= false;

collisionFreeEdgesMat = zeros(1,numel(NearNodesIdx));
for k = 1:numel(NearNodesIdx) % Finding the position of near nodes in nodeList
    nearSaturNode(:,k)= nodeList(:,NearNodesIdx(k));

    currentIdx = NearNodesIdx(k);        

    collisionEdge = false;
    futureCollisinCost=0;
    totalNumberObs=numel(obstacle); % the total number of obstacles

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
%                 [obsx,obsy]=boundary(obstacle(kr).Object);
            edgeCollision= isCollisionFreeEdge(saturNode,nodeList(1:2,currentIdx),obstacle(kr).Object);
%                 intersectX = polyxpoly([saturNode(1,1) nodeList(1,currentIdx)],[saturNode(2,1) nodeList(2,currentIdx)], obsx,obsy);
%                 if isempty(intersectX) ~= 1 % if there is a conflict between the possible connection and obstacle
%                     distNeighbMat(cardin,currentIdx)=inf;
%                     break
%                 end
            if edgeCollision == 1 % if there is a conflict between the possible connection and obstacle
                distNeighbMat(cardin,currentIdx)=inf;
                break
            end

        end
    end


    % For validMove function in time based on the robot Velocity
    edge2dDist= norm(nearSaturNode(1:2,k)-saturNode(1:2,1));
    % Valid move of the robot depends on the 
    validMove= ( (saturNode(3,1)-nearSaturNode(3,k))*R.robotMinVelocity )<= edge2dDist && edge2dDist <= ( (saturNode(3,1)-nearSaturNode(3,k))*R.robotMaxVelocity );

%         % if there is no validMove found and if there is one NearNodesIDx manipulate the position of the
%         % saturnode
%         if validMove ==0 && numel(NearNodesIdx)==1
%             
%             saturNode(1:2,1)= nearSaturNode(1:2,k) + (saturNode(1:2,1) - nearSaturNode(1:2,k))/edge2dDist ; % one meter difference
%             saturNode(3,1)= nearSaturNode(3,k)+1; % one second difference
%             validMove=1;
%             
%         end

%         validMove=edge2dDist <= ( (saturNode(3,1)-nearSaturNode(3,k))*R.robotMaxVelocity );
        if edgeCollision== 0 && validMove

            % Edge Cost Calculation
            nearDist=edgeCost(saturNode(:,1),nearSaturNode(:,k)); 

            % this is a cost value if the robot's edge connection is lost, this gives futureCollisionCost value to the directly edge
            if futureCollisinCost ==0 
                distNeighbMat(cardin,currentIdx)=nearDist;
            else
                distNeighbMat(cardin,currentIdx)=futureCollisinCost; % Giving the weighted cost for the conflicted edge
            end
%                 distNeighbMat(currentIdx,cardin)=nearDist;               
        else
            collisionEdge=true;
            collisionFreeEdgesMat(1,k)=1;
            distNeighbMat(cardin,currentIdx)=inf;
%                 distNeighbMat(currentIdx,cardin)=inf;                    
        end
    % To find The optimum and collision free obstacle with this if
    % condition


    % finding the parent if neccesary conditions are satisfied
    if (distNeighbMat(cardin,currentIdx) <= param.r+0.00001) && node(cardin).Lmc > (distNeighbMat(cardin,currentIdx) + node(currentIdx).Lmc)  && (~collisionEdge) %% && (~nearStartNodeFound)

        parentFound= true;
        node(cardin).parent = currentIdx;
        node(cardin).Lmc = distNeighbMat(cardin,currentIdx) + node(currentIdx).Lmc;

    end
end



newNode= saturNode;
% Extend line 3 for parent checking and the rest of lines for extend
collisionCheckAllEdges= isempty(find(collisionFreeEdgesMat==0,1));
if ~collisionCheckAllEdges && parentFound
    nodeList(:,cardin)= newNode(:,1);
    currentParent=node(cardin).parent;
    stringofParentIdx= int2str(currentParent);            
    stringofNewNode= int2str(cardin);
    graphTree = addedge(graphTree, {stringofNewNode},{stringofParentIdx},distNeighbMat(cardin,currentParent));
    % Verify node is added
    % Making the new node the child of its parent
    if ~ismember(cardin, node(currentParent).child)
        node(currentParent).child = [node(currentParent).child, cardin];
    else
        numOfChild=numel(node(currentParent).child); 
        node(currentParent).child(1,numOfChild+1)=cardin; 
    end  
    isNodeAdded = true;

    % rvalues(1,cardin)=r; % to check the hyperball radius
    for k = 1:numel(NearNodesIdx) % Finding the position of near nodes in nodeList

        nearNode = NearNodesIdx(k);
        % obtacle lari tek tek degerlendirip break koymam gerekecek gibi

        % Checking collision between the connected path of two neighbours and obstacle
        if distNeighbMat(cardin,nearNode) ~= Inf
            numOutConnect=numel(node(cardin).InitialOutConnect);
            node(cardin).InitialOutConnect(1,numOutConnect+1)= nearNode;
            numInConnect=numel(node(nearNode).RadiusInConnect);
            node(nearNode).RadiusInConnect(1,numInConnect+1)=cardin;
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
            node(nearNode).RadiusOutConnect(1,numOutConnect+1)= cardin;
            numInConnect= numel(node(cardin).InitialInConnect);
            node(cardin).InitialInConnect(1,numInConnect+1)= nearNode;
            neighbDist=edgeCost(nodeList(:,nearNode),newNode(:,1)); % The cost between the saturated Node and its neighbours
            distNeighbMat(nearNode,cardin)=neighbDist;
        end
        % nearNodeDist = norm(nodeList(1,cardin)-nodeList(1,nearNode));

        % collisionEdgeNeighnour = false;
    end
    % cardin=cardin+1;
else
    isNodeAdded = false;
end


end