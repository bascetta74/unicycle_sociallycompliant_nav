function collision=explicitEdgeCheck(ObsNum,obstacle,startNode,endNode)
% must check all edges of obstacle path that overlap with the robot edge in time
if obstacle(ObsNum).Used == 0
    collision=false;
    return
end
% if obstacle(ObsNum).Moving ~= 1
%         for kr= 1:totalNumberObs % obstacle avoidance checking if the new connection between saturated node and nearest node
%             if obstacle(kr).Moving==1
        [obsx,obsy]=boundary(obstacle(ObsNum).Object);    
        intersectX = polyxpoly([startNode(1,1) endNode(1,1)],[startNode(2,1) endNode(2,1)], obsx,obsy);
        if isempty(intersectX) ~= 1
            collision = true;
            return
        end        
%             end
%         end
% else 
%     if startNode(3,1) < endNode(3,1)
%         earlyNode = startNode;
%         lateNode = endNode;
%     else
%         lateNode= startNode;
%         earlyNode= endNode;
%     end
% 
%     totalInd=numel(obstacle(ObsNum).FuturePoint(1,:));
%     % find Indx before time
%     iPathfirst= 0;
%     iPathsecond= 0;
% 
%     while iPathfirst+1 <= totalInd && obstacle(ObsNum).FuturePoint(3,iPathfirst+1) < earlyNode(3)
%         iPathfirst = iPathfirst+1;
%     end
%     while iPathsecond+1 <= totalInd && obstacle(ObsNum).FuturePoint(3,iPathsecond+1) < lateNode(3)
%         iPathsecond = iPathsecond+1;
%     end
% 
%     firstObsInd= max(iPathfirst,1);
%     lastObsInd= min(1+iPathsecond,totalInd);
% 
%     for i_start = firstObsInd:(lastObsInd-1)
%         i_end= i_start +1;
% 
% 
%         x_1 = earlyNode(1); % robot start x, y and start time values
%         y_1 = earlyNode(2);
%         t_1 = earlyNode(3); 
% 
%         %calculate the time of minimum approach of the centers of obstacle and robot
%         x_2 = obstacle(ObsNum).Path(1,i_start) + obstacle(ObsNum).PosX;
%         y_2 = obstacle(ObsNum).Path(2,i_start) + obstacle(ObsNum).PosY;
%         t_2 = obstacle(ObsNum).Path(3,i_start);
% 
%         % calculate intermediate quantities (parametric slopes)
%         m_x1 = (lateNode(1)- x_1)/(lateNode(3)-t_1);
%         m_y1 = (lateNode(2)- y_1)/(lateNode(3)-t_1);
%         m_x2 = ( obstacle(ObsNum).Path(1,i_end) + obstacle(ObsNum).PosX - x_2 )/( obstacle(ObsNum).Path(3,i_end) - t_2 );
%         m_y2 = ( obstacle(ObsNum).Path(2,i_end) + obstacle(ObsNum).PosY - y_2 )/( obstacle(ObsNum).Path(3,i_end) - t_2 );
% 
%         % solve for time of closest pass of centers
%         t_c = ((m_x1^2 * t_1 + m_x2 * (m_x2 * t_2 + x_1 - x_2) -  m_x1 * (m_x2 * (t_1 + t_2) + x_1 - x_2)...
%         + (m_y1 - m_y2) * (m_y1 * t_1 - m_y2 * t_2 - y_1 + y_2)) / ((m_x1 - m_x2)^2 + (m_y1 - m_y2)^2));
% 
%         % now bound T_c by the allowable times of the robot and the obstacle
%         if t_c  < max(t_1,t_2)
%             t_c = max(t_1,t_2);
%         elseif t_c > min(lateNode(3),obstacle(ObsNum).Path(3,i_end))
%             t_c = min(lateNode(3),obstacle(ObsNum).Path(3,i_end));        
%         end
% 
%         % finally see if the distance between the robot and the obstacle at T_c 
%         % is close enough to cause a conflict
%         r_x = m_x1*(t_c - t_1) + x_1;  % robot x at T_c
%         r_y = m_y1*(t_c - t_1) + y_1;  % robot y at T_c
%         o_x = m_x2*(t_c - t_2) + x_2;  % obstacle x at T_c
%         o_y = m_y2*(t_c - t_2) + y_2;  % obstacle y at T_c
% 
%         distCur=(r_x - o_x)^2 + (r_y - o_y)^2;
%         obstacleRad= 7;
% 
%         if distCur < obstacleRad^2
%             collision = true;
%             return
%         end
%     end
% end
collision= false;
end