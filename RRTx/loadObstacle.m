function [obstacle,movingObsNum]= loadObstacle(staticObsNum,movingObsNum,path_ped,N,Ts,firstPlanningFlag,r_c, map)

%I want to estimate te pedestrian future position within the current
%prediction horizon with N = 20 --> 4 seconds
motionTime = 0:0.2:(N-1)*Ts;
allObs=staticObsNum+movingObsNum;
% obstacle struct initiliziations
obstacle(allObs).Object=[];
[obstacle.Used]=deal(true);
obstacle(allObs).PosX=[];
obstacle(allObs).PosY=[];
[obstacle.StartTime]=deal(0.0);
[obstacle.FinishTime]=deal(0.0);
[obstacle.Radius]=deal(0.0);
obstacle(allObs).Path=[];
[obstacle.Sensing]=deal(false);
[obstacle.Moving]=deal(false);
% obstacle(4).Used=0;



%% Pixels are taken from the Cursor points
wallsMatrix = occupancyMatrix(map);
wallsNum = length(find(wallsMatrix == 1));
[w_idx, w_idy] = find(wallsMatrix == 1);
numStaticObs=staticObsNum+wallsNum;



%% Moving Obstacle Loading Part From Now on


allPedID=[1:2:numel(path_ped(1,:))];

movingObsNum=numel(allPedID);

pedID=allPedID(1:movingObsNum);  % Id of pedestrian that will be used. The first movingObsNum(required number of moving) of allPedId is taken 

allObs=staticObsNum+movingObsNum;
% obstacle struct initiliziations
obstacle(allObs).Object=[];
% [obstacle.Used]=deal(false);
% obstacle(allObs).PosX=[];
% obstacle(allObs).PosY=[];
[obstacle.StartTime]=deal(0.0);
[obstacle.FinishTime]=deal(0.0);
% [obstacle.Radius]=deal(0.0);
obstacle(allObs).Path=[];
[obstacle.Sensing]=deal(false);
[obstacle.Moving]=deal(false);
%%% Human shape initilization

% obPersonX=[-1 -1 1 1]/2; % the size of human x and y positions
% obPersonY=[-1 1 1 -1]/2;
% obPersonX=[-1 -1 1 1]*0.20; % the size of human x and y positions
% obPersonY=[-1 1 1 -1]*0.25;
th = 0:0.01:2*pi;
obPersonX = r_c*cos(th);
obPersonY = r_c*sin(th);
polyHuman=polyshape(obPersonX,obPersonY); % human shape is transformed to a poly shape
[centX,centY]=centroid(polyHuman); % centroid of the human
[~,ylim]=boundingbox(polyHuman);
humRadius=sqrt(2)*( (ylim(2)-ylim(1)) / 2 ); % radius of human to use for obstacle detection

%Add 1 static obstacle
if staticObsNum ~= 0
    obsPos = [obstacle(1:staticObsNum).PosX;obstacle(1:staticObsNum).PosY];
    ObsCoord(1,:) = obsPos(1,:)+r_c*cos(th);
    ObsCoord(2,:) = obsPos(2,:)+r_c*sin(th);
    
    for obi=1:staticObsNum
        simObsCoordX=ObsCoord(1,:);
        simObsCoordY=ObsCoord(2,:);
        simObsCoord=[simObsCoordX;simObsCoordY];
        obstacle(obi).Object=polyshape(simObsCoord(1,:),simObsCoord(2,:));
        obstacle(obi).Radius= r_c;
        obstacle(obi).Used= 1;
        [obstacle(obi).PosX,obstacle(obi).PosY]=centroid(obstacle(obi).Object);
    %     plot(obstacle(obi).Object)
    end

end

% Add walls
for obw = 1:numStaticObs-staticObsNum
    obsPos = [w_idy(obw);w_idx(obw)];
    ObsCoord(1,:) = obsPos(1) + [-1 -1 1 1];
    ObsCoord(2,:) = obsPos(2)-0.5 + 0.5*[-1 1 1 -1];
    
    simObsCoordX=ObsCoord(1,:);
    simObsCoordY=ObsCoord(2,:);
    simObsCoord=[simObsCoordX;simObsCoordY];
    obstacle(staticObsNum+obw).Object=polyshape(simObsCoord(1,:),simObsCoord(2,:));
    obstacle(staticObsNum+obw).Radius= r_c;
    obstacle(staticObsNum+obw).Used= 1;
    obstacle(staticObsNum+obw).Moving= 0;

    [obstacle(staticObsNum+obw).PosX,obstacle(staticObsNum+obw).PosY]=centroid(obstacle(staticObsNum+obw).Object);
%     plot(obstacle(obi).Object)
end

% figure
tempPedID=pedID;

obs_mat = path_ped(:,:);
for k=1:numel(pedID)

    obstacle(numStaticObs+k).Path=[obs_mat(:,k) obs_mat(:,k+numel(pedID)) motionTime'];



    obstacle(numStaticObs+k).Object=polyHuman;
    obstacle(numStaticObs+k).Moving=1;
    if(firstPlanningFlag)
        obstacle(numStaticObs+k).Moving=0;
    end
    %obstacle(numStaticObs+1).Object=rotate(obstacle(numStaticObs+1).Object,45,[centX,centY]);
    %points = [15 4; 18 4; 21 4; 24 4;27 4];
    %points=points(:,1:2)-[8,0];
    obstacle(numStaticObs+k).PosX=obstacle(numStaticObs+k).Path(1,1); % The first x coordinate of the human
    obstacle(numStaticObs+k).PosY=obstacle(numStaticObs+k).Path(1,2);
    obstacle(numStaticObs+k).Object= translate(obstacle(numStaticObs+k).Object,obstacle(numStaticObs+k).PosX-centX,obstacle(numStaticObs+k).PosY-centY);
    obstacle(numStaticObs+k).StartTime= 0;
    obstacle(numStaticObs+k).FinishTime= motionTime(end);
    obstacle(numStaticObs+k).Radius=humRadius;
    
    numStep=numel(motionTime);
    for timePoint=1:numStep-1
        xchange=obstacle(numStaticObs+k).Path(timePoint+1,1)-obstacle(numStaticObs+k).Path(timePoint,1);
        ychange=obstacle(numStaticObs+k).Path(timePoint+1,2)-obstacle(numStaticObs+k).Path(timePoint,2);
        xchange=round(xchange,5);
        ychange=round(ychange,5);
        % to get orientaion of the obstacle for each step
        rotDeg = rad2deg(atan2(ychange, xchange));
        obstacle(numStaticObs+k).Orientation(timePoint)=rotDeg;
        obstacle(numStaticObs+k).Used = 1;
        if firstPlanningFlag
            obstacle(numStaticObs+k).Used = 0;
        end
    end
end
pedID=tempPedID;

% Start of path prediction of pedestrians
kf=1;
PLong(kf)=0;
VelLong(kf)=0;
PLat(kf)=0;
VelLat(kf)=0;

tau= 0.2; % sampling time of the system
varX=0;   % variance in the lateral(x) direction
varY=0;  % variance in the longtitudinal(y) direction
numSteps=31; % total time for prediction 40*0.1= 4 seconds
numSample=1; % Number of test for prediction to see the stochastic beahaviour of the human motion trajectrory
XposGlob=zeros(1,numSteps+1);
YposGlob=zeros(1,numSteps+1);
theta=zeros(1,numSteps);
sampleXp= cell(numSample,1);
XposGlob(kf:kf+1)=0;
YposGlob(kf:kf+1)=0;
theta(kf:numSteps+1)=0;
% figure

Ap= [1 tau 0 0;0 1 0 0;0 0 1 tau;0 0 0 1];
Bp= [0 0;tau 0;0 0;0 tau];
% rand_seed=5;
% rng(rand_seed);
for it=1:numSample
    noiseLong= zeros(1,numSteps);
    noiseLong = sqrt(varY)*randn(size(noiseLong));
    noiseLat=zeros(1,numSteps);
    noiseLat= sqrt(varX)*randn(size(noiseLat));
    Xp=zeros(4,numSteps+1);
    k=1;

    Xp(:,k)=[PLong(k);VelLong(k);PLat(k);VelLat(k)];
    Xp(:,k+1)=Ap*Xp(:,k)+Bp*[noiseLong(k);noiseLat(k)];

    for k=2:numSteps

        Xp(:,k+1)=Ap*Xp(:,k)+Bp*[noiseLong(k);noiseLat(k)];
        
%         changeX=0.36*cos(theta(k))+[cos(theta(k)),-sin(theta(k))]*[(Xp(1,k+1)-Xp(1,k));(Xp(3,k+1)-Xp(3,k))];
        changeX= 0.12*cos(theta(k));
        changeY=0.12*sin(theta(k))+[sin(theta(k)),cos(theta(k))]*[(Xp(1,k+1)-Xp(1,k));(Xp(3,k+1)-Xp(3,k))];
%         changeY= 0.36*sin(theta(k));
%         theta(k+1)=atan(changeY/changeX);
%         theta(k+1)=pi/2;
        XposGlob(k+1)=XposGlob(k)+changeX;
        YposGlob(k+1)=YposGlob(k)+changeY;

    end

%     PLong=Xp(1,1:end);
%     VelLong=Xp(2,1:end);
%     PLat=Xp(3,1:end);
%     VelLat=Xp(4,1:end);
%     plot(PLat,PLong,'b')
%     hold on
%    plot(0:numSteps,PLat,'r')
    
    sampleXp{it,1}= Xp;
    sampleXp{it,2}= XposGlob;
    sampleXp{it,3}= YposGlob;    

end

pedXPos=cell2mat(sampleXp(1:end,2));
pedYPos=cell2mat(sampleXp(1:end,3));

% The finding standard Deviation of the each predicted 0.1 times starting
% from 0.4 to 3.2 seconds
allPredictedTime=1:21;
posStepX=zeros(numel(allPredictedTime),1);
standardDeviationStepY=zeros(numel(allPredictedTime),1);
meanValueStep=zeros(numel(allPredictedTime),1);
pedXPos=pedXPos(:,3:end);
pedYPos=pedYPos(:,3:end);

for i=1:numel(allPredictedTime)
    posStepX(i,1)=pedXPos(1,allPredictedTime(i));
    posStepY=pedYPos(1:end,allPredictedTime(i));
    standardDeviationStepY(i,1)=std(posStepY);
%     varianceStepY(i,1)=var(posStepY);
    meanValueStep(i,1)=mean(posStepY);
end
obstacle(numStaticObs+1).FuturePredictTimes=posStepX;

% finding a proper function for the standard deviation by usig 2 degree
% polynominal function
% polFunc=polyfit(1:numel(standardDeviationStepY),standardDeviationStepY',2);
polFunc=polyfit(posStepX,standardDeviationStepY,2);
obstacle(numStaticObs+1).DeviationFunction=polFunc;
% obstacle(numStaticObs+1).ProbabilityDistrubution(:,2)=standardDeviationStepY;
% checking values from sampling time 32 because it is higher than 3 meter
% distance of personal space
FirstPredictTime=allPredictedTime(1);
MidPredcitTime=(allPredictedTime(1)+allPredictedTime(end))/2;
EndPredictTime=allPredictedTime(end);
% checkSampleTime=[FirstPredictTime,MidPredcitTime,EndPredictTime]; % taking the sample first, mid and final of step of prediction
checkSampleTime=FirstPredictTime:EndPredictTime;
posX=zeros(numel(checkSampleTime),2);
posY=zeros(numel(checkSampleTime),numSample);
Xpoints=zeros(2,numel(checkSampleTime));
Ypoints=zeros(2,numel(checkSampleTime));
YpointsIn=zeros(2,numel(checkSampleTime));
YpointsOut=zeros(2,numel(checkSampleTime));

for i=1:numel(checkSampleTime)
    
    posX(i,:)=pedXPos(1,checkSampleTime(i));
    posY(i,:)=pedYPos(1:end,checkSampleTime(i));
%     meanValueX=mean(posX(i,:));
%     standarDeviationX=std(posX(i,:));
    Xpoints(:,i)=[posX(i,1);posX(i,1)];
    meanValueY=mean(posY(i,:));
    standarDeviationY=std(posY(i,:));
    Ypoints(:,i)=[meanValueY-2*standarDeviationY;meanValueY+2*standarDeviationY];
    YpointsIn(:,i)=[meanValueY-standarDeviationY;meanValueY+standarDeviationY];
    YpointsOut(:,i)=[meanValueY-3*standarDeviationY;meanValueY+3*standarDeviationY];
%     obstacle(6).futureYPos(i,:)=[meanValueY-standarDeviationY,meanValueY+standarDeviationY];
    
end

humanwidthsizeX=0.75; % half of the human width (approximated to half shoulder distance of a person)
humanwidthsizeY=0.75; % half of the human length in the longitutidinal side (approximated distance from chest to back of a person)

% since human trajectory is calculated as a point, here we add size of a
% person to projection
Xpoints(:,1)=Xpoints(:,1)-humanwidthsizeY;
Xpoints(:,end)=Xpoints(:,end)+humanwidthsizeY;
Ypoints(1,:)=Ypoints(1,:)-humanwidthsizeX;
Ypoints(2,:)=Ypoints(2,:)+humanwidthsizeX;
YpointsIn(1,:)=YpointsIn(1,:)-humanwidthsizeX;
YpointsIn(2,:)=YpointsIn(2,:)+humanwidthsizeX;
YpointsOut(1,:)=YpointsOut(1,:)-humanwidthsizeX;
YpointsOut(2,:)=YpointsOut(2,:)+humanwidthsizeX;
obstacle(numStaticObs+1).futureXPos=reshape(Xpoints,1,[]);
obstacle(numStaticObs+1).futureYPos=reshape(Ypoints,1,[]);
innerYpoints=reshape(YpointsIn,1,[]);
outerYpoints=reshape(YpointsOut,1,[]);


polygonXascend=sort(unique(obstacle(numStaticObs+1).futureXPos),'ascend');
polygonXdescend=sort(unique(obstacle(numStaticObs+1).futureXPos),'descend');
polygonYascend=sort(obstacle(numStaticObs+1).futureYPos,'ascend');
innerYassend=sort(innerYpoints,'ascend');
outerYassend=sort(outerYpoints,'ascend');
obstacle(numStaticObs+1).FutureObject=polyshape([polygonXdescend,polygonXascend],polygonYascend);
innerObject=polyshape([polygonXdescend,polygonXascend],innerYassend);
outerObject=polyshape([polygonXdescend,polygonXascend],outerYassend);

%% Plotting the future prediction region for my writing part, not included inside the 
% figure
% hold on
% % plot(outerObject)
% % plot(obstacle(numStaticObs+1).FutureObject)
% % plot(innerObject)
% % hold off
% 
% for im=1:numSample
% %     xLoc(im)=sampleXp{im,2};
% %     yLoc(im)=sampleXp{im,3};
% %     plot(sampleXp{im,2},sampleXp{im,3},'-r')
% %     axis([0 10 -10 10])
% %     scatter(sampleXp{im,2}(32),sampleXp{im,3}(32),20,'b','filled')
%     for idot=2:32
%         scatter(sampleXp{im,2}(idot),sampleXp{im,3}(idot),6,'b','filled')
%     end
% %     scatter(sampleXp{im,2}(20),sampleXp{im,3}(20),20,'g','filled')
% %     scatter(sampleXp{im,2}(42),sampleXp{im,3}(42),20,'y','filled')
% %     hold on
% end
% % plot(outerObject)
% title('Possible Positions of the Human Based on the Motion Model')
% xlabel('x(meter)')
% ylabel('y(meter)')
% % set(gca, 'XLim', [0 5],'YLim', [-2.5 2.5]);


%%

%0.1 distance grid
gridDist=0.1;
numOfXgrid=round(((Xpoints(1,end)+gridDist)-(Xpoints(1,1)-gridDist))/gridDist);
numOfYgrid=round(((YpointsOut(2,end)+gridDist)-(YpointsOut(1,end)-gridDist))/gridDist);
xminGridLim=Xpoints(1,1)-gridDist/2;
xmaxGridLim=Xpoints(1,end)+gridDist/2;
yminGridLim=YpointsOut(1,end)-gridDist/2;
ymaxGridLim=YpointsOut(2,end)+gridDist/2;
weightMatrix=NaN(numOfYgrid,numOfXgrid);
m=1;
n=1;

for xpos=xminGridLim:gridDist:xmaxGridLim
    
    for ypos=yminGridLim:gridDist:ymaxGridLim
        if ~isinterior(outerObject,xpos,ypos)            
            weightMatrix(n,m)=0.1;
            n=n+1;
            continue
        else
            if ~isinterior(obstacle(numStaticObs+1).FutureObject,xpos,ypos)
                weightMatrix(n,m)=0.25;
            else
                if ~isinterior(innerObject,xpos,ypos)
                    weightMatrix(n,m)=0.35;
                else
                    weightMatrix(n,m)=0.65;
                end                
            end
            n=n+1;
        end        
    end
    m=m+1;
    n=1;
end

obstacle(numStaticObs+1).WeightMatrix=weightMatrix;
% Limit Grids will hepl you find a node postion in the matrix of

% % % % % weightMatrix Figure codes

obstacle(numStaticObs+1).LimitGrids=[xminGridLim,xmaxGridLim,yminGridLim,ymaxGridLim,gridDist];

% ObsTimePoint To track the movement of the obstacles while moving  in the path
[obstacle(numStaticObs+1:numStaticObs+numel(pedID)).ObsTimePoint]=deal(1);
[obstacle(numStaticObs+1:numStaticObs+numel(pedID)).ElapseTime]=deal(false); % To verify sampling time is passed for the movement of each obstacle
[obstacle(numStaticObs+1:numStaticObs+numel(pedID)).Detected]=deal(false); % To check the obstacle is detected by the robot in the sensor range
[obstacle(numStaticObs+1:numStaticObs+numel(pedID)).Waiting]=deal(false); % This is the case, the obstacle is in the map but waiting at  a point
[obstacle(numStaticObs+1:numStaticObs+numel(pedID)).RotateDeg]=deal(0);
[obstacle(numStaticObs+1:numStaticObs+numel(pedID)).Active]=deal(0); % shows whether the moving obstacle is active or not. When the time of the moving obstacle comes, it becomes active 

% Set the  future predict area information for all the rest of
% moving obstacles which is same with the first moving obstacle
% Also other parameters
for k=2:numel(pedID)
    obstacle(numStaticObs+k).futureXPos=obstacle(numStaticObs+1).futureXPos;
    obstacle(numStaticObs+k).futureYPos=obstacle(numStaticObs+1).futureYPos;
    obstacle(numStaticObs+k).FutureObject=obstacle(numStaticObs+1).FutureObject;
    obstacle(numStaticObs+k).WeightMatrix=obstacle(numStaticObs+1).WeightMatrix;
    obstacle(numStaticObs+k).LimitGrids=obstacle(numStaticObs+1).LimitGrids;
    obstacle(numStaticObs+k).FuturePredictTimes=obstacle(numStaticObs+1).FuturePredictTimes;
    obstacle(numStaticObs+k).DeviationFunction=obstacle(numStaticObs+1).DeviationFunction;
end

% % % Plotting the future predition area of the Obstacle
% % figure
% % hold on
% % plot(outerObject,'FaceColor','red')
% % plot(obstacle(numStaticObs+1).FutureObject,'FaceColor','yellow')
% % plot(innerObject,'FaceColor','green')
% % 
% % % Finish Plotting the future predition area of the Obstacle
% % 
% % ax=gca;
% % ax.XTick=[xminGridLim:gridDist:xmaxGridLim];
% % ax.YTick=[yminGridLim:gridDist:ymaxGridLim];
% % ax.GridAlpha = 0.8;
% % axis([xminGridLim xmaxGridLim yminGridLim ymaxGridLim])
% % grid on

% % % % % weightMatrix Figure codes finish

%%%Plot figure of the points
% % % % xLoc=zeros(numSample,1);
% % % % yLoc=zeros(numSample,1);
% % % for im=1:numSample
% % % %     xLoc(im)=sampleXp{im,2};
% % % %     yLoc(im)=sampleXp{im,3};
% % %     plot(sampleXp{im,2},sampleXp{im,3},'-r')
% % % %     axis([0 10 -10 10])
% % %     scatter(sampleXp{im,2}(20),sampleXp{im,3}(20),20,'b','filled')
% % %     scatter(sampleXp{im,2}(10),sampleXp{im,3}(10),20,'m','filled')
% % %     scatter(sampleXp{im,2}(11),sampleXp{im,3}(11),20,'g','filled')
% % %     scatter(sampleXp{im,2}(21),sampleXp{im,3}(21),20,'y','filled')
% % %     hold on
% % % end
% % % axis([0 10 -10 10])







end