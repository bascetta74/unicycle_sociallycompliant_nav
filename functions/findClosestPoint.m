function P_int = findClosestPoint(x_sol, y_sol, robot_position)

% The function gets as input the solutions x_sol and y_sol (coordinates
% of the intersection points line/virtualbox or line/psf) and the
% wheelchair position, giving as output the intersection point closer
% to the wheelchair

% INPUT: x_sol - x coordinate of the two intersection points
%        y_sol - y coordinate of the two intersection points
%        robot_position - wheelchair position

% OUTPUT: P_int - intersection point closer to the wheelchair

%if more than one intersection point is the detected, the point closer
%to the robot position is extracted

d1=norm([robot_position(1) robot_position(2)]-[x_sol(1) y_sol(1)]);
d2=norm([robot_position(1) robot_position(2)]-[x_sol(2) y_sol(2)]);
x_int = x_sol(1); y_int = y_sol(1);
if d2<d1
    x_int = x_sol(2); y_int = y_sol(2);
end
P_int = double([x_int; y_int]);
end

