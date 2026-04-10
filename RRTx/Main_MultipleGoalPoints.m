figure(7)
close(gcf)
% VideoData=0; %This saves the simulation frames and makes video later 
videoIterNumb=1; % This record the data The first video iteration number
startFrame=1;

%% Two Goals Example Start and Goal Points
firstStartPos=[2;9]; % The first parameter is the x pos, second one y pos
firstGoalPos=[6;6.5]; % This is the second startPos also
secondGoalPos=[11;12];

%% Three Goals Example Start and Goal Points
% firstStartPos=[2;9]; % The first parameter is the x pos, second one y pos
% firstGoalPos=[4;7]; % This is the second startPos also
% secondGoalPos=[13;9];
% thirdGoalPos=[10;12];

%% First Movement
simVisualTime=48-10; % Time spent to find the intial tree. The simulation doesn't show before this
simRobotMoveTime=49-10; % Robot start moving at this time
simObsMoveTime=48-10; % Obstacle's start time. It choose one second time difference with the robot move time.
[videoIterNumb,VideoData,secondStartFrame]=MainLoopFunction(firstStartPos,firstGoalPos,startFrame,videoIterNumb,simVisualTime,simRobotMoveTime,simObsMoveTime);

%% Second Movement
simVisualTime=48;
simRobotMoveTime=49;
simObsMoveTime=48;
[videoIterNumb,VideoData,thirdStartFrame]=MainLoopFunction(firstGoalPos,secondGoalPos,secondStartFrame,videoIterNumb,simVisualTime,simRobotMoveTime,simObsMoveTime,VideoData);

%% Third Movement
simVisualTime=31;
simRobotMoveTime=32;
simObsMoveTime=31;
[videoIterNumb,VideoData,lastFrame]=MainLoopFunction(secondGoalPos,thirdGoalPos,thirdStartFrame,videoIterNumb,simVisualTime,simRobotMoveTime,simObsMoveTime,VideoData);

%%
% Video Creating
video= VideoWriter('RRTXMultipleObstaclesTwoGoals.mp4','MPEG-4');
video.FrameRate = 5; % If this is taken 10, then real time video can be obtained
open(video)
writeVideo(video, VideoData)
close(video)
