function [graphTree, node, path_x, path_y, path_plot, path_graph, param, obstacle, map] = RRTXmain(path_ped, r_c, Ts, startLoc, endLoc, v_ped, n_obs, N, mapFile, firstPlanningFlag, current_time, replanningFlag)

persistent graphParams
persistent oldNode

% control random number generation for having same random nodes
rand_seed=560;
rng(rand_seed);

% Number of static obstacles
n_st = numel(find(v_ped(:,1).^2 + v_ped(:,2).^2 == 0));

delta= 2;% The longest edge distance in the graph
% if (firstPlanningFlag)
%     delta=2;
% end
ballconstant= 100;  % gama value of the shrinking ball radius, it is a constant which depends on the size of environment area

%First static OBSTACLE UPDATE, if there is a need
updateObsMat=[20000; 1; 1];
obsUpdateCount=size(updateObsMat(1,:));
iterObsTime=1;
noMoreUpdate=false;

[nodeList,graphTree,node,distNeighbMat, R, env, numSample, remObsSampleList, orphanNodeMat, obstacle, Q, QMatrix, param, TRmovie, map] = environmentParam(startLoc, endLoc, path_ped, n_obs, n_st, N, Ts, firstPlanningFlag, graphParams, oldNode, r_c, current_time, mapFile);

% Extract walls point objects
wallPoints = length(obstacle) - n_obs;

% add other "times" to root of tree if in first planning phase
if firstPlanningFlag
    [nodeList,graphTree,node,addedNode,distNeighbMat] = addOtherTimesToRoot(nodeList,node,graphTree,R,distNeighbMat);
end

currNodeList = length(nodeList);
% The loop start here
i=3+currNodeList-2; %  addedNode is the number of node added to root of the graph (Check addOtherTimesToRoot function). Their ositions are the same with goal node postion but their time is an increasing order

% Adjust nodes count for replanning
if firstPlanningFlag
    nodesCond = numSample;
else
    nodesCond = numSample+currNodeList;
end

while i <=  nodesCond    % i equals to node number at current replanning phase

    %Shrinking Ball Radius
    r = min(delta, ballconstant*sqrt(log(i+1)/i));
    param.r=r; % put inside the param struct

    % % Simulation Time parameters
    % simTimeCounter=simTimeCounter+1; % Increase counter
    % simTime=simTimeCounter*simSliceTime;
    % %timeObsUpdatemc=toc;


    % %Update Obstacle operation
    if (~firstPlanningFlag)
        [obstacle,remObsSampleList,node,graphTree,Q,QMatrix,distNeighbMat]= updateObstacles(updateObsMat(2,iterObsTime),updateObsMat(3,iterObsTime),...
            remObsSampleList,obstacle,node,nodeList,graphTree,R,i,Q,QMatrix,distNeighbMat,param,orphanNodeMat);

        % This is the part for translation of the moving obstacles in the map
        for movingObs= n_st+wallPoints+1:n_obs+wallPoints
            obstacle=humanPositionChange(obstacle,movingObs,current_time);
        end
    end

    %%% IMPORTANT: Updating the cost of Moving obstacle region WHEN it is detected
    %%% by robot sensors

    %%% Remove the effect of areas previously occupied by the moving
    %%% obstacles by checking old trajectory of the obstacle
    [node,graphTree,Q,QMatrix,distNeighbMat]= removeMovingObsPastPosition(obstacle,node,nodeList,R,graphTree,...
        Q,QMatrix,distNeighbMat,param);

    % Future Position Cost Calculation

    %%% Updating the cost area of the moving obstacle at each sampling time and
    %%% define cost on the nodes in these areas based cost function
    %%% Update the cost of the nodes around Moving obstacle
    %If i'm at the first planning phase i don't need to update the pedestrians
    %position
    if (~firstPlanningFlag)
        [obstacle,node,remObsSampleList,graphTree,Q,QMatrix,distNeighbMat] = updatePedestrian(obstacle,remObsSampleList,node,...
            nodeList,graphTree,R,i,Q,QMatrix,distNeighbMat,orphanNodeMat,param);
    end

    %%% If the parent of Robot node changes during the movement on the edge,
    %%% then sample a point on the current robot Pos and connect with neighbour
    parentEdgeDisconnected=false;
    if ~isempty(node(R.robotNode).parent)&& R.moving == 1 && node(R.robotNode).parent~= R.nextMoveTarget
        remObsSampleList=[remObsSampleList,R.nextRobotPose'];
        R.robotPose=R.nextRobotPose';
        parentEdgeDisconnected=true;
        R.robotNode=i;
        imove=imove-2;
    end

    %%% Randomly sampling a point or sampling from a determined list
    %%% if priority RemObsSampleList list is empty, sample randomly from free
    %%% space
    [newSample,remObsSampleList]=randomNodeGenerate(R,nodeList,remObsSampleList,env,parentEdgeDisconnected);

    % find closest node to the new node among the inserted nodes
    [closestPoint,nnodeDist]=findNearestNode(newSample,nodeList,i);
    %closestPoint, the position of the nearest node
    % nnodeDist is the distance to closest node

    % Saturation operation
    % moves newPoint toward closestPoint such that each robot is no
    % further than delta, points repesent the cartesian product of R robots
    if nnodeDist > delta %  If newSample is far away than the radius then find a new point which is delta  distance away in the direction newSample

        saturNode= closestPoint + (newSample - closestPoint)*(2)/norm(newSample - closestPoint) ;

        edge2dDist= norm(closestPoint(1:2,1)-saturNode(1:2,1));

    else %If the newSample is less than r, use this point as a saturNode
        saturNode=newSample;
        nnodeDist = norm(newSample-closestPoint);
    end

    % check for collision with static obstacles
    collisionFreeNode=nodeCollisionCheck(obstacle,saturNode);

    % If the saturated node is not in free space, go back the first line of while loop to
    % sample a new node
    if collisionFreeNode==0
        continue
    end
    % THE ALGORITHM 2 EXTEND(v,r) STARTS

    % [node,graphTree,isNodeAdded,distNeighbMat,nodeList]=extendTry(nodeList,node,saturNode,i,distNeighbMat,graphTree,param,obstacle,parentEdgeDisconnected,R);

    % Vnear <<< near(v,r) Finding Vnear operation
    nodeListT = nodeList(:,1:i-1)'; % Taking all the nodes to find the nearest node
    saturNodeT= saturNode(:,1)';
    NearNodesIdxCell= rangesearch(nodeListT(:,1:2),saturNodeT(1:2),param.r+1e-10); % Finding the near nodes in the circle of r for saturated Node(saturNode)
    NearRobotNodesCell = rangesearch(nodeListT(:,1:2),R.robotPose(1:2),3); % Find nodes near the robot
    NearRobotNodes = cell2mat(NearRobotNodesCell);
    NearNodesIdx=cell2mat(NearNodesIdxCell);

    checkNode=zeros(3,numel(NearRobotNodes));
    if (~firstPlanningFlag)
        for k=1:numel(NearRobotNodes)
            checkNode(:,k) = nodeList(:,NearRobotNodes(k));
            currentIdx = NearRobotNodes(k);
            collisionEdge = false;
            futureCollisinCost = 0;
            CollisionCost = 0;
            edgeCollision = 0;
            for kr= n_st+wallPoints+1:n_obs+wallPoints % obstacle avoidance checking if the new connection between saturated node and nearest node
                if obstacle(kr).Used==1

                    if obstacle(kr).Moving==1 && obstacle(kr).Detected==1 && obstacle(kr).Waiting~=1
                        [oneEdgeCollision,CollisionCost]=OneEdgeFuturePredictCollision(R.robotPose',checkNode(:,k),kr,obstacle,obstacle(kr).ObsTimePoint,param.colTolerance,R);
                        if oneEdgeCollision==1
                            if CollisionCost > futureCollisinCost
                                futureCollisinCost=CollisionCost; % Future CollisionCost will be used to assign the cost of this conflicted edge
                            end
                        end
                    end

                    % % This is a regular collision check with the all obstacles inside map.
                    %                 [obsx,obsy]=boundary(obstacle(kr).Object);
                    edgeCollision= isCollisionFreeEdge(R.robotPose',nodeList(1:2,currentIdx),obstacle(kr));
                    %                 end
                    if edgeCollision == 1 % if there is a conflict between the possible connection and obstacle
                        distNeighbMat(i,currentIdx)=inf;
                        break
                    end
                end
                if edgeCollision== 0

                    % Edge Cost Calculation
                    nearDist=edgeCost(R.robotPose,checkNode(:,k));

                    % this is a cost value if the robot's edge connection is lost, this gives futureCollisionCost value to the directly edge
                    if futureCollisinCost ==0
                        distNeighbMat(i,currentIdx)=nearDist;
                    else
                        distNeighbMat(i,currentIdx)=futureCollisinCost; % Giving the weighted cost for the conflicted edge
                    end
                else
                    collisionEdge=true;
                    collisionFreeEdgesMat(1,k)=1;
                    distNeighbMat(i,currentIdx)=inf;
                    %                 distNeighbMat(currentIdx,i)=inf;
                end

                try
                    node(currentIdx).Lmc = distNeighbMat(i,currentIdx) + node(currentIdx).Lmc;
                catch ME
                    rethrow(ME)
                end
            end

        end
    end
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
            currentIdx = NearNodesIdx(k);

            collisionEdge = false;
            CollisionCost=0;
            futureCollisinCost=0;
            edgeCollision = 0;
            for kr= n_st+wallPoints+1:n_obs+wallPoints % obstacle avoidance checking if the new connection between saturated node and nearest node
                if obstacle(kr).Used==1

                    % % This part is for the cost calculation of conflicted
                    % edge with the predicted future area

                    % Firstly checking collision check with the future
                    % prediction area of the obstacle
                    %
                    % parentEdgeDisconnected==true (possibile problema dato che
                    % non mi interessa delle valid move?)

                    if obstacle(kr).Moving==1 && obstacle(kr).Detected==1 && obstacle(kr).Waiting~=1
                        [oneEdgeCollision,CollisionCost]=OneEdgeFuturePredictCollision(saturNode,nearSaturNode(:,k),kr,obstacle,obstacle(kr).ObsTimePoint,param.colTolerance,R);
                        if oneEdgeCollision==1
                            if CollisionCost > futureCollisinCost
                                futureCollisinCost=CollisionCost; % Future CollisionCost will be used to assign the cost of this conflicted edge
                            end
                        end
                    end

                    % % This is a regular collision check with the all obstacles inside map.
                    %                 [obsx,obsy]=boundary(obstacle(kr).Object);
                    edgeCollision= isCollisionFreeEdge(saturNode,nodeList(1:2,currentIdx),obstacle(kr));
                    %                 end
                    if edgeCollision == 1 % if there is a conflict between the possible connection and obstacle
                        distNeighbMat(i,currentIdx)=inf;
                        break
                    end
                end
            end

            if edgeCollision== 0

                % Edge Cost Calculation
                nearDist=edgeCost(saturNode(:,1),nearSaturNode(:,k));

                % this is a cost value if the robot's edge connection is lost, this gives futureCollisionCost value to the directly edge
                if futureCollisinCost ==0
                    distNeighbMat(i,currentIdx)=nearDist;
                else
                    distNeighbMat(i,currentIdx)=futureCollisinCost; % Giving the weighted cost for the conflicted edge
                end
            else
                collisionEdge=true;
                collisionFreeEdgesMat(1,k)=1;
                distNeighbMat(i,currentIdx)=inf;
                %                 distNeighbMat(currentIdx,i)=inf;
            end
            % To find The optimum and collision free obstacle with this if
            % condition

            % finding the parent if neccesary conditions are satisfied
            try
                if (distNeighbMat(i,currentIdx) <= param.r+1e-10) && node(i).Lmc > (distNeighbMat(i,currentIdx) + node(currentIdx).Lmc)  && (~collisionEdge) %% && (~nearStartNodeFound)

                    parentFound= true;
                    node(i).parent = currentIdx;
                    node(i).Lmc = distNeighbMat(i,currentIdx) + node(currentIdx).Lmc;
                end
            catch ME
                rethrow(ME)
            end
        end

        newNode= saturNode;
        % Extend line 3 for parent checking and the rest of lines for extend
        %     collisionCheckAllEdges= isempty(find(collisionFreeEdgesMat==0,1));
        %     if ~collisionCheckAllEdges && parentFound
        if parentFound
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

                edgeCollision2 =false;
                for kr= 1:n_obs % obstacle avoidance checking if the new connection between saturated node and nearest node
                    collision=explicitEdgeCheck(kr,obstacle,nodeList(:,nearNode),newNode(:,1));
                    if collision == 1
                        edgeCollision2= true;
                        break
                    end
                end

                if ~edgeCollision2
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

    %Extend operation is finished here and if the node is added pass to
    %reduceInconsistency and the rest of the code
    if isNodeAdded && ~isempty(NearNodesIdx)
        % if isNodeAdded

        % First Rewire Operation

        [node,QMatrix,Q,graphTree]= rewire(i,param,nodeList,node,graphTree,Q,QMatrix,distNeighbMat);

        % Reduce Inconsistency

        [graphTree,node,Q,QMatrix]= reduceInconsistency(node,nodeList,graphTree,param,Q,QMatrix,distNeighbMat,R);

        i=i+1;
    end

    % shortest path and robot position drawings only if firstPlanning flag or
    % replanningFlag are true
    if (replanningFlag || firstPlanningFlag)
        % sensor Range drawings
        th = 0:pi/50:2*pi;
        xunit = param.sensorRange * cos(th) + R.robotPose(1);
        yunit = param.sensorRange * sin(th) + R.robotPose(2);
        numberCurrentObs=[obstacle.Used]==1; % Show the total number of Used Obstacles in the map
        clf
        hold on
        pedLegend=false;
        for obsi=1:numel(numberCurrentObs)
            if numberCurrentObs(obsi)==1
                if obstacle(obsi).Moving==1
                    plotPed=plot(obstacle(obsi).Object,'FaceColor','m','FaceAlpha',0.7);
                    pedLegend=true;
                    %                 obsPlot(obsi).FaceColor='b';
                else
                    plotSta=plot(obstacle(obsi).Object,'FaceColor','k','FaceAlpha',0.8);
                end
            end
        end
        pedFuture=true;
        for movingObs= n_st+wallPoints+1:n_obs+wallPoints
            if obstacle(movingObs).Used == 1 && obstacle(movingObs).ObsTimePoint <= numel(obstacle(movingObs).Orientation)
                %         plot(obstacle(movingObs).WeightArea{obsTimePoint},'FaceColor','white','FaceAlpha',0.1,'LineStyle','--','EdgeColor','red')
                if obstacle(movingObs).Detected && ~obstacle(movingObs).Waiting
                    plotPedestrinFuture=plot(obstacle(movingObs).FutureObject,'FaceColor','white','FaceAlpha',0.1,'LineStyle','--','EdgeColor','black','DisplayName','Predicted Future Pedestrian Area');
                    pedFuture=true;
                end
            end
        end

        show(map);
        plot(xunit, yunit,'LineStyle','--','Color',[1, 0.41, 0.16],'LineWidth',1.2,'DisplayName','Sensor Range'); % plotting the sensor range
        %The graph Plotting
        pmovie=plot(graphTree,'XData',nodeList(1,:),'YData',nodeList(2,:),'Marker','o', 'EdgeColor','b', 'MarkerSize',1,'NodeColor','c','EdgeAlpha',0.2,'DisplayName','Graph'); %The graph Plotting

        TRmovie = shortestpathtree(graphTree,{num2str(R.nextMoveTarget)},{'2'}); % find the shortest path from node nextMoveTarget to 2(goal node)
        currentEdgeMovie= shortestpathtree(graphTree,{num2str(R.robotNode)},{num2str(R.nextMoveTarget)});

        highlight(pmovie,TRmovie,'EdgeColor','r', 'LineWidth', 4,'NodeColor','g','MarkerSize' ,6) % Shortest Path highlight on the graph
        highlight(pmovie,currentEdgeMovie,'EdgeColor','g', 'LineWidth', 4,'NodeColor','g','MarkerSize' ,6) % Current Edge Highlight

        scatter(startLoc(1),startLoc(2),250,'filled','d','MarkerFaceColor','y','DisplayName','Robot Start Node') % Marking Start position  for plot
        scatter(endLoc(1),endLoc(2),250,'filled','d','MarkerFaceColor','g','DisplayName','Robot Goal Node') % Marking Goal position for plot
        %     head=scatter(R.robotMovePath(imove,1),R.robotMovePath(imove,2),70,'filled','o','MarkerFaceColor','r','DisplayName','Robot'); % plotting the robot position with star
        head=scatter(R.robotPose(1),R.robotPose(2),70,'filled','o','MarkerFaceColor','r','DisplayName','Robot'); % plotting the robot position with star
        hold off

        if(~isempty(TRmovie.Edges) && ~firstPlanningFlag)
            break
        end
    end
end
A=R.robotMovePath;
% pathdistance = ['The final shortest path distance is ', num2str(distances(graphTree,'1','2'))];
% display(pathdistance)

% Timing
path_graph = shortestpathtree(graphTree,{num2str(R.nextMoveTarget)},{'2'},'OutputForm','cell');
for i = 1:length(path_graph{1})
    path_nodes(i) = str2num(path_graph{1}{i});
end

sortedList = nodeList(:,path_nodes);
path_plot = TRmovie;
path_x = sortedList(1,:);
path_y = sortedList(2,:);

%Get unique values of x-y coordinates to get the correct number of
%waypoitns
path = unique([path_x',path_y'],'rows','stable');
path_x = path(:,1)';
path_y = path(:,2)';

graphParams.nodeList= nodeList;
graphParams.graphTree = graphTree;
graphParams.distNeighbMat = distNeighbMat;
graphParams.remObsSampleList = remObsSampleList;
graphParams.orphanNodeMat = orphanNodeMat;
graphParams.QMatrix = QMatrix;
graphParams.Q = Q;
graphParams.shortestPath = path_plot;

oldNode = node;
end
