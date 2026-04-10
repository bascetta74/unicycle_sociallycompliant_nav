function [x_sol, y_sol] = findIntersection(line, ped_pos, r_c, PSf,  flag)

% The function computes the intersection points between a line and a
% circle or an ellipse, depending on the value of flag parameter. For
% the computation of the intersection points on a circle, the PSf input
% is given to the function as an empty vector, while r_c parameter is
% the radius of the circle (the obstacle virtual box)

% INPUT:
%        line - struct containing the line coefficients
%        ped_pos - pedestrian position (center of its virtual box)
%        r_c - circle radius
%        PSf - vector containing the ellipse axis values and its angle
%              wrt the global reference x axis, corresponding to the
%              pedestrian heading direction (PSf = [a_e b_e alfa])
%        flag - parameter used to distinguish the two cases of
%               intersection computation

% OUTPUT:
%        x_sol - x coordinate of the intersection point
%        y_sol - y coordinate of the intersection point

a = line.a; b = line.b; c = line.c;
x_c = ped_pos(1); y_c = ped_pos(2);

if flag == false
    %Intersections on the circle
    %line parallel to x axis case
    if (line.a == 0)
        x_sol = [((-(c + b*r_c + b*y_c)*(c - b*r_c + b*y_c))^(1/2) + b*x_c)/b;
            -((-(c + b*r_c + b*y_c)*(c - b*r_c + b*y_c))^(1/2) - b*x_c)/b];
        y_sol = [ped_pos(2);
            ped_pos(2)];
    else
        x_sol  =  [-(c - (b*(b*c - a^2*y_c + a*(a^2*r_c^2 - a^2*x_c^2 - 2*a*b*x_c*y_c - 2*a*c*x_c + b^2*r_c^2 - b^2*y_c^2 - 2*b*c*y_c - c^2)^(1/2) + a*b*x_c))/(a^2 + b^2))/a;
            -(c - (b*(b*c - a^2*y_c - a*(a^2*r_c^2 - a^2*x_c^2 - 2*a*b*x_c*y_c - 2*a*c*x_c + b^2*r_c^2 - b^2*y_c^2 - 2*b*c*y_c - c^2)^(1/2) + a*b*x_c))/(a^2 + b^2))/a];

        y_sol  = [-(b*c - a^2*y_c + a*(a^2*r_c^2 - a^2*x_c^2 - 2*a*b*x_c*y_c - 2*a*c*x_c + b^2*r_c^2 - b^2*y_c^2 - 2*b*c*y_c - c^2)^(1/2) + a*b*x_c)/(a^2 + b^2);
            -(b*c - a^2*y_c - a*(a^2*r_c^2 - a^2*x_c^2 - 2*a*b*x_c*y_c - 2*a*c*x_c + b^2*r_c^2 - b^2*y_c^2 - 2*b*c*y_c - c^2)^(1/2) + a*b*x_c)/(a^2 + b^2)] ;
    end
else
    %Intersections on ellipse
    a_e = PSf(1); b_e = PSf(2);
    alfa = PSf(3);
    %line parallel to x axis case
    if (line.a == 0 )
        x_sol = [(a_e*b_e*(b^2*b_e^2*cos(alfa)^2 - c^2*sin(alfa)^4 - c^2*cos(alfa)^4 + a_e^2*b^2*sin(alfa)^2 - b^2*y_c^2*cos(alfa)^4 - b^2*y_c^2*sin(alfa)^4 - 2*c^2*cos(alfa)^2*sin(alfa)^2 - 2*b^2*y_c^2*cos(alfa)^2*sin(alfa)^2 - 2*b*c*y_c*cos(alfa)^4 - 2*b*c*y_c*sin(alfa)^4 - 4*b*c*y_c*cos(alfa)^2*sin(alfa)^2)^(1/2) - a_e^2*c*cos(alfa)*sin(alfa) + b_e^2*c*cos(alfa)*sin(alfa) + b*b_e^2*x_c*cos(alfa)^2 + a_e^2*b*x_c*sin(alfa)^2 - a_e^2*b*y_c*cos(alfa)*sin(alfa) + b*b_e^2*y_c*cos(alfa)*sin(alfa))/(b*a_e^2*sin(alfa)^2 + b*b_e^2*cos(alfa)^2);
            (b_e^2*c*cos(alfa)*sin(alfa) - a_e^2*c*cos(alfa)*sin(alfa) - a_e*b_e*(b^2*b_e^2*cos(alfa)^2 - c^2*sin(alfa)^4 - c^2*cos(alfa)^4 + a_e^2*b^2*sin(alfa)^2 - b^2*y_c^2*cos(alfa)^4 - b^2*y_c^2*sin(alfa)^4 - 2*c^2*cos(alfa)^2*sin(alfa)^2 - 2*b^2*y_c^2*cos(alfa)^2*sin(alfa)^2 - 2*b*c*y_c*cos(alfa)^4 - 2*b*c*y_c*sin(alfa)^4 - 4*b*c*y_c*cos(alfa)^2*sin(alfa)^2)^(1/2) + b*b_e^2*x_c*cos(alfa)^2 + a_e^2*b*x_c*sin(alfa)^2 - a_e^2*b*y_c*cos(alfa)*sin(alfa) + b*b_e^2*y_c*cos(alfa)*sin(alfa))/(b*a_e^2*sin(alfa)^2 + b*b_e^2*cos(alfa)^2)];
        y_sol = [ped_pos(2);
            ped_pos(2)];
    else
        x_sol = [-(c - (b*(b*b_e^2*c*cos(alfa)^2 + a_e^2*b*c*sin(alfa)^2 - a^2*a_e^2*y_c*cos(alfa)^2 - a^2*b_e^2*y_c*sin(alfa)^2 + a*a_e*b_e*(a^2*a_e^2*cos(alfa)^2 - c^2*sin(alfa)^4 - c^2*cos(alfa)^4 + b^2*b_e^2*cos(alfa)^2 + a^2*b_e^2*sin(alfa)^2 + a_e^2*b^2*sin(alfa)^2 - a^2*x_c^2*cos(alfa)^4 - b^2*y_c^2*cos(alfa)^4 - a^2*x_c^2*sin(alfa)^4 - b^2*y_c^2*sin(alfa)^4 - 2*c^2*cos(alfa)^2*sin(alfa)^2 - 2*a^2*x_c^2*cos(alfa)^2*sin(alfa)^2 - 2*b^2*y_c^2*cos(alfa)^2*sin(alfa)^2 - 2*a*c*x_c*cos(alfa)^4 - 2*b*c*y_c*cos(alfa)^4 - 2*a*c*x_c*sin(alfa)^4 - 2*b*c*y_c*sin(alfa)^4 - 2*a*b*x_c*y_c*cos(alfa)^4 - 2*a*b*x_c*y_c*sin(alfa)^4 + 2*a*a_e^2*b*cos(alfa)*sin(alfa) - 2*a*b*b_e^2*cos(alfa)*sin(alfa) - 4*a*c*x_c*cos(alfa)^2*sin(alfa)^2 - 4*b*c*y_c*cos(alfa)^2*sin(alfa)^2 - 4*a*b*x_c*y_c*cos(alfa)^2*sin(alfa)^2)^(1/2) + a*a_e^2*c*cos(alfa)*sin(alfa) - a*b_e^2*c*cos(alfa)*sin(alfa) + a*b*b_e^2*x_c*cos(alfa)^2 + a*a_e^2*b*x_c*sin(alfa)^2 + a^2*a_e^2*x_c*cos(alfa)*sin(alfa) - a^2*b_e^2*x_c*cos(alfa)*sin(alfa) - a*a_e^2*b*y_c*cos(alfa)*sin(alfa) + a*b*b_e^2*y_c*cos(alfa)*sin(alfa)))/(a^2*a_e^2*cos(alfa)^2 + b^2*b_e^2*cos(alfa)^2 + a^2*b_e^2*sin(alfa)^2 + a_e^2*b^2*sin(alfa)^2 + 2*a*a_e^2*b*cos(alfa)*sin(alfa) - 2*a*b*b_e^2*cos(alfa)*sin(alfa)))/a;
            -(c - (b*(b*b_e^2*c*cos(alfa)^2 + a_e^2*b*c*sin(alfa)^2 - a^2*a_e^2*y_c*cos(alfa)^2 - a^2*b_e^2*y_c*sin(alfa)^2 - a*a_e*b_e*(a^2*a_e^2*cos(alfa)^2 - c^2*sin(alfa)^4 - c^2*cos(alfa)^4 + b^2*b_e^2*cos(alfa)^2 + a^2*b_e^2*sin(alfa)^2 + a_e^2*b^2*sin(alfa)^2 - a^2*x_c^2*cos(alfa)^4 - b^2*y_c^2*cos(alfa)^4 - a^2*x_c^2*sin(alfa)^4 - b^2*y_c^2*sin(alfa)^4 - 2*c^2*cos(alfa)^2*sin(alfa)^2 - 2*a^2*x_c^2*cos(alfa)^2*sin(alfa)^2 - 2*b^2*y_c^2*cos(alfa)^2*sin(alfa)^2 - 2*a*c*x_c*cos(alfa)^4 - 2*b*c*y_c*cos(alfa)^4 - 2*a*c*x_c*sin(alfa)^4 - 2*b*c*y_c*sin(alfa)^4 - 2*a*b*x_c*y_c*cos(alfa)^4 - 2*a*b*x_c*y_c*sin(alfa)^4 + 2*a*a_e^2*b*cos(alfa)*sin(alfa) - 2*a*b*b_e^2*cos(alfa)*sin(alfa) - 4*a*c*x_c*cos(alfa)^2*sin(alfa)^2 - 4*b*c*y_c*cos(alfa)^2*sin(alfa)^2 - 4*a*b*x_c*y_c*cos(alfa)^2*sin(alfa)^2)^(1/2) + a*a_e^2*c*cos(alfa)*sin(alfa) - a*b_e^2*c*cos(alfa)*sin(alfa) + a*b*b_e^2*x_c*cos(alfa)^2 + a*a_e^2*b*x_c*sin(alfa)^2 + a^2*a_e^2*x_c*cos(alfa)*sin(alfa) - a^2*b_e^2*x_c*cos(alfa)*sin(alfa) - a*a_e^2*b*y_c*cos(alfa)*sin(alfa) + a*b*b_e^2*y_c*cos(alfa)*sin(alfa)))/(a^2*a_e^2*cos(alfa)^2 + b^2*b_e^2*cos(alfa)^2 + a^2*b_e^2*sin(alfa)^2 + a_e^2*b^2*sin(alfa)^2 + 2*a*a_e^2*b*cos(alfa)*sin(alfa) - 2*a*b*b_e^2*cos(alfa)*sin(alfa)))/a];

        y_sol = [-(b*b_e^2*c*cos(alfa)^2 + a_e^2*b*c*sin(alfa)^2 - a^2*a_e^2*y_c*cos(alfa)^2 - a^2*b_e^2*y_c*sin(alfa)^2 + a*a_e*b_e*(a^2*a_e^2*cos(alfa)^2 - c^2*sin(alfa)^4 - c^2*cos(alfa)^4 + b^2*b_e^2*cos(alfa)^2 + a^2*b_e^2*sin(alfa)^2 + a_e^2*b^2*sin(alfa)^2 - a^2*x_c^2*cos(alfa)^4 - b^2*y_c^2*cos(alfa)^4 - a^2*x_c^2*sin(alfa)^4 - b^2*y_c^2*sin(alfa)^4 - 2*c^2*cos(alfa)^2*sin(alfa)^2 - 2*a^2*x_c^2*cos(alfa)^2*sin(alfa)^2 - 2*b^2*y_c^2*cos(alfa)^2*sin(alfa)^2 - 2*a*c*x_c*cos(alfa)^4 - 2*b*c*y_c*cos(alfa)^4 - 2*a*c*x_c*sin(alfa)^4 - 2*b*c*y_c*sin(alfa)^4 - 2*a*b*x_c*y_c*cos(alfa)^4 - 2*a*b*x_c*y_c*sin(alfa)^4 + 2*a*a_e^2*b*cos(alfa)*sin(alfa) - 2*a*b*b_e^2*cos(alfa)*sin(alfa) - 4*a*c*x_c*cos(alfa)^2*sin(alfa)^2 - 4*b*c*y_c*cos(alfa)^2*sin(alfa)^2 - 4*a*b*x_c*y_c*cos(alfa)^2*sin(alfa)^2)^(1/2) + a*a_e^2*c*cos(alfa)*sin(alfa) - a*b_e^2*c*cos(alfa)*sin(alfa) + a*b*b_e^2*x_c*cos(alfa)^2 + a*a_e^2*b*x_c*sin(alfa)^2 + a^2*a_e^2*x_c*cos(alfa)*sin(alfa) - a^2*b_e^2*x_c*cos(alfa)*sin(alfa) - a*a_e^2*b*y_c*cos(alfa)*sin(alfa) + a*b*b_e^2*y_c*cos(alfa)*sin(alfa))/(a^2*a_e^2*cos(alfa)^2 + b^2*b_e^2*cos(alfa)^2 + a^2*b_e^2*sin(alfa)^2 + a_e^2*b^2*sin(alfa)^2 + 2*a*a_e^2*b*cos(alfa)*sin(alfa) - 2*a*b*b_e^2*cos(alfa)*sin(alfa));
            -(b*b_e^2*c*cos(alfa)^2 + a_e^2*b*c*sin(alfa)^2 - a^2*a_e^2*y_c*cos(alfa)^2 - a^2*b_e^2*y_c*sin(alfa)^2 - a*a_e*b_e*(a^2*a_e^2*cos(alfa)^2 - c^2*sin(alfa)^4 - c^2*cos(alfa)^4 + b^2*b_e^2*cos(alfa)^2 + a^2*b_e^2*sin(alfa)^2 + a_e^2*b^2*sin(alfa)^2 - a^2*x_c^2*cos(alfa)^4 - b^2*y_c^2*cos(alfa)^4 - a^2*x_c^2*sin(alfa)^4 - b^2*y_c^2*sin(alfa)^4 - 2*c^2*cos(alfa)^2*sin(alfa)^2 - 2*a^2*x_c^2*cos(alfa)^2*sin(alfa)^2 - 2*b^2*y_c^2*cos(alfa)^2*sin(alfa)^2 - 2*a*c*x_c*cos(alfa)^4 - 2*b*c*y_c*cos(alfa)^4 - 2*a*c*x_c*sin(alfa)^4 - 2*b*c*y_c*sin(alfa)^4 - 2*a*b*x_c*y_c*cos(alfa)^4 - 2*a*b*x_c*y_c*sin(alfa)^4 + 2*a*a_e^2*b*cos(alfa)*sin(alfa) - 2*a*b*b_e^2*cos(alfa)*sin(alfa) - 4*a*c*x_c*cos(alfa)^2*sin(alfa)^2 - 4*b*c*y_c*cos(alfa)^2*sin(alfa)^2 - 4*a*b*x_c*y_c*cos(alfa)^2*sin(alfa)^2)^(1/2) + a*a_e^2*c*cos(alfa)*sin(alfa) - a*b_e^2*c*cos(alfa)*sin(alfa) + a*b*b_e^2*x_c*cos(alfa)^2 + a*a_e^2*b*x_c*sin(alfa)^2 + a^2*a_e^2*x_c*cos(alfa)*sin(alfa) - a^2*b_e^2*x_c*cos(alfa)*sin(alfa) - a*a_e^2*b*y_c*cos(alfa)*sin(alfa) + a*b*b_e^2*y_c*cos(alfa)*sin(alfa))/(a^2*a_e^2*cos(alfa)^2 + b^2*b_e^2*cos(alfa)^2 + a^2*b_e^2*sin(alfa)^2 + a_e^2*b^2*sin(alfa)^2 + 2*a*a_e^2*b*cos(alfa)*sin(alfa) - 2*a*b*b_e^2*cos(alfa)*sin(alfa))];
    end
end
end
