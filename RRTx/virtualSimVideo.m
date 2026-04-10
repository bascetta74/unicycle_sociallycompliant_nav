% virtual simulation video is made in this function

function virtualSimVideo(videoStr,videoNum,param,nodeList)

figure(2)
hold on
set(gcf,'position',[500,50,720,576]);
videoHotel=VideoReader('seq_hotel.avi');
H=[   1.1048200e-02   6.6958900e-04  -3.3295300e+00;
  -1.5966000e-03   1.1632400e-02  -5.3951400e+00;
   1.1190700e-04   1.3617400e-05   5.4276600e-01];
Hinv=inv(H);

for ivid=1:videoNum-1
    

    RobotPose=videoStr(ivid).Rob;
    thisObstacle=videoStr(ivid).Obs;
    shortestPathNodes=videoStr(ivid).PathNodes;
    videoFrame=videoStr(ivid).FrameNum;
%     thisRobotPos=videoStr(ivid).Graph;

    clf(2)
    pause(0.05)
    figure(2)
    myread=read(videoHotel,videoFrame);
    imshow(myread)
    pause(0.05)
    hold on
    
    % sensor range drawing in the real map
    th = 0:pi/50:2*pi;
    xunit = param.sensorRange * cos(th) + RobotPose(1);
    yunit = param.sensorRange * sin(th) + RobotPose(2);
    sensorRangeUnit=[yunit;xunit;ones(1,numel(xunit))];
    sensReal=(sensorRangeUnit-[4;10;0]).*[-1;1;1];
    sensscaledRes=Hinv*sensReal;
    SensPixMat=sensscaledRes(:,:)./sensscaledRes(3,:);
    plot(SensPixMat(2,:),SensPixMat(1,:),'LineStyle','--','Color',[1, 0.41, 0.16],'LineWidth',1.5)
    
    % Ploting the position of the obstacles in the video
    obstacle=thisObstacle;
    numberCurrentObs=[obstacle.Used]==1;
    for obsi=1:numel(numberCurrentObs)
        if numberCurrentObs(obsi)==1
            if obstacle(obsi).Moving==1
                [obsx,obsy]=boundary(obstacle(obsi).Object); % taking the vertice positions of the obstacles
                obsx=obsx(1:end-1)'; % the last element of the array is repeated so it is deleted
                obsy=obsy(1:end-1)';
                VideoRealobs=[-1*(obsy-4);obsx-10;ones(1,numel(obsx))];
                
                scaledObsPos=Hinv*VideoRealobs;
                PixObsMat=scaledObsPos(:,:)./scaledObsPos(3,:);
                
                PixObsCoord=[PixObsMat(2,:);PixObsMat(1,:)];
                plotPedVideo=polyshape(PixObsCoord(1,:),PixObsCoord(2,:));
                
                plot(plotPedVideo,'FaceColor','m','FaceAlpha',0.4);
%                 pedLegend=true;
%                 obsPlot(obsi).FaceColor='b';
            else
%                 plotSta=plot(obstacle(obsi).Object,'FaceColor','k','FaceAlpha',0.8);
            end
        end    
    end
    
    % In the real map, drawing the green line which highlights the edge on where the robot% is at the time drawing is made
    firtsNode=str2double(shortestPathNodes{1});
    secondNode=str2double(shortestPathNodes{2});
    
    simPos1=nodeList(1:2,firtsNode);
    simPos2=nodeList(1:2,secondNode);
    
    VideoRealPos1=[-1*(simPos1(2)-4);simPos1(1)-10;1];
    VideoRealPos2=[-1*(simPos2(2)-4);simPos2(1)-10;1];

    scaledResStart=Hinv*[VideoRealPos1,VideoRealPos2];
    PixPositions=scaledResStart(:,:)./scaledResStart(3,:);
    
    scatter(PixPositions(2,:),PixPositions(1,:),40,'o','filled','g')
    plot(PixPositions(2,:),PixPositions(1,:),'Color','g','LineWidth',2)
    
    % Drawing the red line which highlights the optimal trajectory on the  real map(video)
    
    for im=1:numel(shortestPathNodes)-1
        firtsNode=str2double(shortestPathNodes{im});
        secondNode=str2double(shortestPathNodes{im+1});
        simPos1=nodeList(1:2,firtsNode);
        simPos2=nodeList(1:2,secondNode);

        VideoRealPos1=[-1*(simPos1(2)-4);simPos1(1)-10;1];
        VideoRealPos2=[-1*(simPos2(2)-4);simPos2(1)-10;1];

        scaledResStart=Hinv*[VideoRealPos1,VideoRealPos2];
        PixPositions=scaledResStart(:,:)./scaledResStart(3,:);        

        scatter(PixPositions(2,:),PixPositions(1,:),40,'o','filled','g')
        plot(PixPositions(2,:),PixPositions(1,:),'Color','r','LineWidth',2)

    end
    
    %Drawing the start node, goal node and robot position in the real map
    VideoRealStartPos=[-1*(nodeList(2,1)-4);nodeList(1,1)-10;1];
    VideoRealGoalPos=[-1*(nodeList(2,2)-4);nodeList(1,2)-10;1];
    VideoRobotPos=[-1*(RobotPose(2)-4);RobotPose(1)-10;1];

    scaledResStart=Hinv*[VideoRealStartPos,VideoRealGoalPos,VideoRobotPos];
    PixMat=scaledResStart(:,:)./scaledResStart(3,:);


    scatter(PixMat(2,1),PixMat(1,1),150,'d','filled','y')
    scatter(PixMat(2,2),PixMat(1,2),120,'d','filled','g')
    scatter(PixMat(2,3),PixMat(1,3),80,'o','filled','r')
    
    pause(0.2)
    set(figure(2),'position',[500,50,720,576]);
    pause(0.2)
    hold off
    drawnow
    FRealVir(ivid)= getframe(figure(2),[0,0,720,576]); % saving the frames into freal video   

    
end

% Real video making
% Here there could be a frame proablem, I delete the frames which are not
% corresponds to the suitable frames
videoReal= VideoWriter('HotelVideoVirtualTry.mp4','MPEG-4');
videoReal.FrameRate = 5;
open(videoReal)
writeVideo(videoReal, FRealVir)
close(videoReal)

%}

end