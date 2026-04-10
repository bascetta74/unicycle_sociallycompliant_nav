function collisionResult = isStateFree(checkobs, checkNode)
% operation for checking that the checkNode is inside of the checkobs
% obstacle

%The checkobs is the obstacle for which the collision check is made
% checkNode is the query node

collisionResult = isinterior(checkobs, checkNode(1,1), checkNode(2,1));

end