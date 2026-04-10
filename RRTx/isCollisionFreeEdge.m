function intersectedEdge= isCollisionFreeEdge(edgeFirstNode,edgeSecondNode,queryobstacle)

% intersectedEdge variable indicates whether there is a collision or not. 
% if intersectedEdge=0, no collision
% if intersectedEdge=1, there is a collision between the edge 

% this is for finding the border of the polygonal shapes
[obsx,obsy]=boundary(queryobstacle.Object);

% Intersection points between the edge line and the polygonal shape
intersectX = polyxpoly([edgeFirstNode(1,1) edgeSecondNode(1,1)],[edgeFirstNode(2,1) edgeSecondNode(2,1)], obsx,obsy);

if isempty(intersectX) ~= 1 % if there is a conflict between the edge and obstacle
    
    intersectedEdge=1;
else
    intersectedEdge=0;
end
if isempty(queryobstacle.FuturePredictObject)
    intersectedEdge = 0;
    return
end
[obsx,obsy]=boundary(queryobstacle.FuturePredictObject);
intersectX1 = polyxpoly([edgeFirstNode(1,1) edgeSecondNode(1,1)],[edgeFirstNode(2,1) edgeSecondNode(2,1)], obsx,obsy);

if isempty(intersectX1) ~= 1 % if there is a conflict between the edge and obstacle
    
    intersectedEdge=1;
else
    intersectedEdge=0;
end

end