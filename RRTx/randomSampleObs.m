% function [sampleOb]=randomSampleObs(removedObs)
% Sampling for new nodes in the empty space of Removed Obstacle
% [borderx,bordery]= boundingbox(removedObs);
% numObsSample= 20;
% iobs=0;
% sampleOb=zeros(2,numObsSample);
% while iobs<20
%     sampleTryOb= [borderx(1)+randn()*(borderx(2)-borderx(1));
%     bordery(1)+randn()*(bordery(2)-bordery(1))];
%     if isinterior(removedObs, sampleTryOb(1,1), sampleTryOb(2,1))
%         iobs=iobs+1;
%         sampleOb(:,iobs)=sampleTryOb;
%     end
% end
% end
% [xcen,ycen]=centroid(a);
% xycen=[xcen,ycen];
% rbiggest= 0;
% for i= 1:numel(xver)
%     rtemp=sqrt((xvert(i)-xcen)^2+(yvert(i)-ycen)^2);
%     if rtemp > rbiggest
%         rbiggest= rtemp;
%     end    
% end
% 
% h= viscircles(xycen,rbiggest);
% plot(a)
% hold on
% viscircles(xycen,rbiggest)


% map = robotics.BinaryOccupancyGrid(100,100,1);
% %xy = [5 5; 4.3 4.4; 5.6 5.3];
% setOccupancy(map,xy1,1);
% show(map);
% hold on
% plot(xy1(:,1),xy1(:,2),'xr','MarkerSize', 20)
% grid on
% set(gca,'XTick',0:1:100,'YTick',0:1:100)


