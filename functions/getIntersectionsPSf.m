function [x_sol, y_sol] = getIntersectionsPSf(x_ellipse, y_ellipse, x_circle, y_circle, ped_pos, v_ped)

% The function gets as input the intersection points on the circle and
% the ellipse composing the personal space function, the pedestrian
% position and velocity. The outputs are the x and y coordinates of the
% intersection point found on the pedestrian personal space function
%
% INPUTS:
%       x_ellipse - x coordinate of the intersection point on the
%                   ellipse
%       y_ellipse - y coordinate of the intersection point on the
%                   ellipse
%       x_circle  - x coordinate of the intersection point on the
%                   cirlce
%       y_circle  - y coordinate of the intersection point on the
%                   circle
%       ped_pos   - pedestrian position
%       v_ped     - pedestrian velocity
%
% OUTPUTS:
%       x_sol - x coordinate of the intersection point on the PSf
%       y_sol - y coordinate of the intersection point on the PSf

x_sol = x_ellipse;y_sol = y_ellipse;

%here i check the two intersection points on the ellipse -> if the
%intersection point is "behind" wrt the pedestrian position, it is
%switched with the intersection point computed on the circle
for i = 1:length(x_sol)
    if (dot([x_sol(i); y_sol(i)]-ped_pos,v_ped')<=0)
        x_sol(i) = x_circle(i);
        y_sol(i) = y_circle(i);
    end
end
end
