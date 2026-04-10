function [collisionFreeNode]=nodeCollisionCheck(obstacle,saturNode)

% collisionFreeNode gives the result of collision check after checking the collision with all static obstacles 
% Checking whether new Node sampled in the free space of the environment by checking collision with with the static obstacles

totalNumberObs=numel(obstacle); % the total number of obstacle

collisionFreeNode=1;
for k= 1:totalNumberObs
    if obstacle(k).Used==1
        if obstacle(k).Moving ~= 1 % Check Only static obstacle(not obstacle(k).Moving=0) 
            checkobs=obstacle(k).Object;
            %checkobs=currentObsCellList{existedObsMat(k),1};
            intersectedNode = isStateFree(checkobs, saturNode);
            if intersectedNode
                collisionFreeNode=0;
                break
            end
        end
    end
end



end