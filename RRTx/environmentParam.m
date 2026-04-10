function [nodeList,graphTree,node,distNeighbMat, R, env, numSample, remObsSampleList, orphanNodeMat, obstacle, Q, QMatrix, param, TRmovie, map] = environmentParam(startLoc, endLoc, path_ped, n_obs, n_st, N, Ts, firstPlanningFlag, graphParams, oldNode, r_c, current_time, mapFile)

% Load map
map = load(strcat(mapFile,'.mat'));
map = map.map;

map_inflated = load(strcat(mapFile,'_inflated.mat'));
map_inflated = map_inflated.map;

motionTime = 0:0.2:4;
% environmentDimension=[min(startLoc(1), endLoc(1))+3,max(startLoc(1), endLoc(1))+3]; % It is the length of the environment area, the first element of the array gives the x-axis and and the second is y-axis.
environmentDimension=[map.GridSize(2) map.GridSize(1)-1]; % Dimension of the environment = dimension of the loaded map 
% env is the struct that contains the dimensions related environment
env.x=environmentDimension(1); %  env.x is the length of x-axis
env.y=environmentDimension(2); %  env.y is the length of y-axis


maxPathNodes= 400;
maxSample = 2000;
if firstPlanningFlag
    % The max number of Samples in the first graph construction graph
    numSample= 1000;
else
    % Number of nodes added at each replanning phase
    numSample = 10;
end
    
    
% Robot Class initializing 
robotMaxVelocity=0.55; % Robot Velocity
robotMinVelocity=0;

if firstPlanningFlag
    % nodeList includes the position information of the nodes in the map, first
    % node(first column) is the start node, and second is the goal node
    nodeList(1,1)= startLoc(1,1);
    nodeList(2,1)= startLoc(1,2);
    nodeList(1,2)= endLoc(1,1);
    nodeList(2,2)= endLoc(1,2);
    nodeList(3,2)= 0.0; % Time is set to 0 which is the time parameter of the goal node 
    maxSamples = 2000;
    
    %Max time to reach goal node. it is set to start node's time
    Max_time=40; 
    nodeList(3,1)=Max_time; % Max_time is set to time value of the start node.The maximum time to reach the goal node from start node, it is set to t
    
    % this is the matrix to keep the distance information between the nodes
    distNeighbMat=NaN(maxSample,maxSample);
    
    orphanNodeMat=zeros(2,maxSamples);
    
    % Q is a kind of data structure list. It is used to pop the nodes with min
    % Lmc
    Q = MinHeap(maxSample/2);
    % Making grapgh, and set the start and goal node
    graphTree= digraph;
    graphTree = addnode(graphTree, {'1'}); % start node adding in the tree
    graphTree = addnode(graphTree, {'2'}); % end node adding in the tree
    
    node(maxSamples).parent= [];
    node(2).Gv = 0.001;
    node(1).Gv= Inf;
    node(maxSamples).Gv=[];
    [node.Gv]= deal(Inf);
    node(maxSamples).Lmc=[];
    [node.Lmc]= deal(Inf);
    node(2).Lmc=0;
    node(maxSamples).child=[];
    QMatrix=[];
    node(maxSamples).MarkedQ=[]; 
    [node.MarkedQ]= deal(false); % initiliaze all the nodes of Q as false
    node(maxSamples).InitialOutConnect=[];
    node(maxSamples).RadiusInConnect=[];
    node(maxSamples).RadiusOutConnect=[];
    node(maxSamples).InitialInConnect=[];
    node(maxSamples).MarkedQOB=[];
    [node.MarkedQOB]= deal(false); % initiliaze all the nodes of obstacle intersected Q values as false
    
    
    % Rob= RobotData(nodeList(:,1),nodeList(:,1),maxPathNodes);
    intersectedNode = 0;
    remObsSampleList =[];
    
    R = RobotData(nodeList(:,1)', 1, maxPathNodes,robotMaxVelocity,robotMinVelocity);
    TRmovie = graphTree;
else
    nodeList = graphParams.nodeList;
    graphTree = graphParams.graphTree;
    node = oldNode;
    distNeighbMat = graphParams.distNeighbMat;
    remObsSampleList = graphParams.remObsSampleList; 
    orphanNodeMat = graphParams.orphanNodeMat;
    QMatrix = graphParams.QMatrix;
    Q = graphParams.Q;
    TRmovie = graphParams.shortestPath;

    [RnearIdx, dist]= knnsearch(nodeList(1:2,:)',startLoc(1:2)); 
    R = RobotData([nodeList(1:2, RnearIdx)' current_time], RnearIdx, maxPathNodes,robotMaxVelocity,robotMinVelocity);
    R.robotPose(3) = current_time;

end

iMoveIncrease = 0;

% parameters for the main operations in the algorithm

sensorRange= 10; % It sets the sensor range (meters)
eConstant = 1; % Epsilon constant
CollisionTolerance=1; %This is for the point collision test between the robot and future positions of the pedestrians

%  These three parameters are stored in param
% struct
param.sensorRange=sensorRange;
param.eConstant=eConstant;
param.colTolerance=CollisionTolerance;

imove=1; % iteration for robot move video

 % add other "times" to root of tree
% [nodeList,graphTree,node,addedNode,distNeighbMat]= addOtherTimesToRoot(nodeList,node,graphTree,R,distNeighbMat);


totalNumberOfStaticObs=n_st; % the number of static Obstacles
NumberOfMovingObs=n_obs-n_st; % the maximum number of moving Obstacles
% IMPORTANT- MAIN INITILIZATION FUNCTION OF OBSTACLES- 
[obstacle,totalNumberOfMovingObs] = loadObstacle(totalNumberOfStaticObs,NumberOfMovingObs,path_ped,N,Ts,firstPlanningFlag, r_c, map_inflated); % The obstacle is loaded in to map, the number inside shows the total number of obstacle are defined
totalNumberObs=totalNumberOfStaticObs + totalNumberOfMovingObs;

end
