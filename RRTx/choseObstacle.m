function [nextObstacle,nextObstacleList,existedObsList]= choseObstacle(obsNumber,currentObsList,allObstacles,command)
% command positive 1 means we provide outputs for addObstacle operation
% command negative -1 means we provide outputs for removeObstacle operation
if command >0
    
    emptyList=find(cellfun(@isempty,currentObsList));
    if numel(emptyList)==0
        error('Error. \nNo extra obstacle can be definded. All obstacles are on the map')
    end
    if isempty(find(emptyList==obsNumber, 1))
        error('Error. \nThe intented to add Obstacle is on already on the map')
    else
        nextObstacle=allObstacles{obsNumber,1};
    end
%     emptyList(emptyList==obsNumber)=[];
    currentObsList(obsNumber,1)={nextObstacle};
    nextObstacleList=currentObsList;
elseif command < 0
    occupiedList=find(~cellfun(@isempty,currentObsList));
    if numel(occupiedList)==0
        error('Error. \nThere is no extra obstacle to delete on the map')
    end
    if isempty(find(occupiedList==obsNumber, 1))
        error('Error. \nThe intented to remove Obstacle is not on the map')
    else
        nextObstacle=allObstacles{obsNumber,1};
    end
    currentObsList(obsNumber,1)={[]};
    nextObstacleList=currentObsList;
end
existedObsList=find(~cellfun(@isempty,currentObsList));
end