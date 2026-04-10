function [newSample,priorityList]=randomNodeGenerate(R,nodeList,priorityList,env,parentEdgeDisconnected)
%%% Randomly sampling a point or sampling from a determined list
%%% if priority RemObsSAmpleList list is empty, sample randomly from free
%%% space

if isempty(priorityList)
    newSample= [env.x;env.y;nodeList(3,1)-nodeList(3,2)].*rand(3,1); % Random sampling, by considering the dimension length(env.x and env.y) of the environment 

    % minumum time that the robot reachs to new sampled node position, if
    % the time of this point is not feasible, choose a random time that
    % robot could possibly reach
    minTimeToReachNode = nodeList(3,2)+ norm(nodeList(1:2,2)-newSample(1:2,end))/ (R.robotMaxVelocity);
    
    %This if condition is applied to saturate the the sampling time parameter for the new node
    if newSample(3,1) < minTimeToReachNode || (newSample(3,1) > nodeList(3,R.robotNode) && R.robotNode ~= 1)
        %resample time in ok range
        newSample(3,1)= minTimeToReachNode + rand * (nodeList(3,R.robotNode)- minTimeToReachNode);
    end
    
else
    newSample= priorityList(:,end); 
    if parentEdgeDisconnected  ~=1 % Similarly,This if condtion is for the sampling time parameter for the new node for the case that the empty space behind a big obstacle to be filled with new sampled nodes.
        newSample(3,1)= (nodeList(3,1)-nodeList(3,2))*rand;
        
        % same role with minTimeToReachNode variable above 
        minTimeToReachNode = nodeList(3,2)+ norm(nodeList(1:2,2)-newSample(1:2,end))/ (R.robotMaxVelocity);
        
        if newSample(3,1) < minTimeToReachNode || (newSample(3,1) > nodeList(3,R.robotNode) && R.robotNode ~= 1)
            %resample time in ok range
            newSample(3,1)= minTimeToReachNode + rand * (nodeList(3,R.robotNode)- minTimeToReachNode);
        end
    end
    
    % Note: if the parent of the the the current Robot Node is empty. Then
    % no need to find a random time, directly take sampling at the robotPos
    % with defined time
    priorityList = priorityList(:,1:end-1); % Taking the sample from the area where removed Obstacles located before
end

end