%          [10, 20, 30];
%          magic(5)};
% plot(x,y,'.')

% map = robotics.BinaryOccupancyGrid(100,100,100);
% x = [1.2; 2.3; 3.4; 4.5; 5.6]*10;
% y = [5.0; 4.0; 3.0; 2.0; 1.0]*10;
% 
% setOccupancy(map, [x y], ones(5,1))
% inflate(map, 2)
% figure
% show(map)
figure
p= plot(graphTree,'XData',nodeList(1,:),'YData',nodeList(2,:),'Marker','o', 'MarkerSize',2);
TR = shortestpathtree(graphTree,{'1'},{'2'}); % find the shortest path from node 1 to 2
% highlight(p,TR,'EdgeColor','r', 'LineWidth', 3) % Shortest Path highlight on the graph
hold on;
cellfun(@plot,currentObsCellList);
pathNodes = shortestpath(graphTree, {'1'},{'2'});
pathNodesMat= str2double(pathNodes);

XpathNodes= nodeList(1,pathNodesMat);
YpathNodes= nodeList(2,pathNodesMat);

curve = animatedline('Color','r','Marker','p','MarkerSize',6,'LineWidth',2);
set(gca, 'XLim', [0 100],'YLim', [0 100]);
grid on;
for i = 1:length(XpathNodes)
addpoints(curve,XpathNodes(i),YpathNodes(i));
head = scatter(XpathNodes(i),YpathNodes(i),'filled','p','MarkerFaceColor','y');
drawnow
pause(0.5);
F(i)= getframe(gcf);

delete(head);
end

video= VideoWriter('RRTXStatic13.avi','uncompressed AVI');
video.FrameRate = 5;
open(video)
writeVideo(video, F)
close(video)


