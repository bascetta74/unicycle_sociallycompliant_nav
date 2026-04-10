figure
I = imread('mapHotel.png');
imshow(I)
%%
% figure(13)
hold on
%
I = imread('mapHotel.png');
imshow(I)
xx=size(I,2)
yy=size(I,1)

%% bozza

video=VideoReader('seq_hotel.avi');
frameNum=1;
myread=read(video,frameNum);

figure(15)
% imshow(myread)
hold on
%plot (xRealPos , yRealPos)
viscircles([xRealPos(1),yRealPos(1)],0.1,'Color','b')
%%
%imcontour(I,3)
%
%[a,b]=imcontour(I,3);
%
boundaries = bwboundaries(I);
numberOfBoundaries = size(boundaries, 1);
%
%OK=boundaries(1)
%
thisBoundary = boundaries{1}; % Pull N by 2 array from k'th cell.
      % Get x and y from the N-by-2 array.
      x = thisBoundary(:,2);
      y = thisBoundary(:,1);
     
plot(x, y, 'r', 'LineWidth', 2); % Plot outline over blob.
%set(gca, 'XAxisLocation','top','YAxisLocation','left','ydir','reverse','XLim', [0 xx],'YLim', [0 yy]);
%%
max_x=max(x);
max_y=max(y);
min_x=min(x);
min_y=min(y);
% (x,y,width,height) 
wi=max_x-min_x;
hei=max_y-min_y;
%
% a{i,1}=polyshape([],[]);
b=rectangle('Position',[min_x, min_y, wi, hei],'FaceColor','r');

matCoord=[min_x,min_y;min_x,max_y;max_x,max_y;max_x,min_y];
centers=[max_x,max_y];
% for k=1:size(matCoord(:,1))
% viscircles([matCoord(k,1),matCoord(k,2)],3,'Color','b')
% end


%%
hold on
%imcontour(I,3)
%
%[a,b]=imcontour(I,3);
%
boundaries = bwboundaries(I);
numberOfBoundaries = size(boundaries, 1);
%
%OK=boundaries(1)
for i=1:numberOfBoundaries
thisBoundary = boundaries{1}; % Pull N by 2 array from k'th cell.
      % Get x and y from the N-by-2 array.
      x = thisBoundary(:,2);
      y = thisBoundary(:,1);

% plot(x, y, 'r', 'LineWidth', 2); % Plot outline over blob.
%
max_x=max(x);
max_y=max(y);
min_x=min(x);
min_y=min(y);
% (x,y,width,height) 
wi=max_x-min_x;
hei=max_y-min_y;
%
% a{i,1}=polyshape([],[]);
b=rectangle('Position',[min_x, min_y, wi, hei],'FaceColor','r');
plot (b)
matCoord=[min_x,min_y;min_x,max_y;max_x,max_y;max_x,min_y];
centers=[max_x,max_y];
% for k=1:size(matCoord(:,1))
% viscircles([matCoord(k,1),matCoord(k,2)],3,'Color','b')
% end
end
H=[   1.1048200e-02   6.6958900e-04  -3.3295300e+00;
  -1.5966000e-03   1.1632400e-02  -5.3951400e+00;
   1.1190700e-04   1.3617400e-05   5.4276600e-01];
%%
matz=[matCoord';1  1 1 1];
Rxyz=H*1.7*matz;
Rxyz=Rxyz(:,:)./Rxyz(3,:);
yRealPos= -1*(Rxyz(2,:))+10
xRealPos= Rxyz(1,:)+10
%%
matz=[matCoord';1  1 1 1];

%%
H=[   1.1048200e-02   6.6958900e-04  -3.3295300e+00;
  -1.5966000e-03   1.1632400e-02  -5.3951400e+00;
   1.1190700e-04   1.3617400e-05   5.4276600e-01];
real1=H*[matCoord(:,1)';matCoord(:,2)';ones(1,4)];
real1(:,:)./real1(3,:)
%set(b,'FaceColor','none','EdgeColor','g','LineWidth',1);
% title(b,'a')

%%
[obstacle,movingObsNum]=loadObstacle;
[xTrans,yTrans]=centroid(obstacle(5).Object);
centObject=translate(obstacle(5).Object,-xTrans,-yTrans);
%%
figure
hold on
plot(centObject)
plot(obstacle(5).FutureObject,'FaceColor','white','FaceAlpha',0.1,'LineStyle','--','EdgeColor','black')
plot(3, 0.5, 'go', 'MarkerSize', 8);
plot(2.5, 0.5,'r*', 'MarkerSize', 5);
plot(3.5, 0.5,'r*', 'MarkerSize', 5);
xlabel('X Longitudinal direction of Pedestrian')
ylabel('Y Lateral Direction')
title('Collision Check Point-Obstacle and Future Prediction Area')
text(2.5, 0.4,'lower Limit -0.5 m','Color','r','HorizontalAlignment','center','FontSize',7)
text(3.5, 0.4,'upper Limit +0.5 m','Color','r','HorizontalAlignment','center','FontSize',7)
text(3.0, 0.7,'Collision Check Point','Color','g','HorizontalAlignment','center','FontSize',6)