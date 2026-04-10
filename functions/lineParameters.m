function line = lineParameters(relative_velocity, P1, P2)

% The function computes the line coefficients, in the implicit form
% a*x+b*y+c=0, for two different cases. For the computation of the line
% obtained form the relative velocity between the pedestrian and the
% wheelchair, the relative velocity and the wheelchair positon P2 are
% used, while P1 given as and empty vector. For the computation of the
% line that intersect the reduced virtual box and the PSf, the
% intersection point on the virtual box P1 and the pedestrian poisition
% P2 are used. In this case the relative_velocity input is provided as
% an empty vector.

% INPUTS:
%       relative_velocity - relative velocity
%       P1 - intersection point on the pedestrian virtual box
%       P2 - pwheelchair position (first case) or pedestrian position
%            (second case)

% OUTPUT:
%       line - struct containing the line coefficients


if (~isempty(relative_velocity))
    %line paramenters for the first intersection point
    line.a = relative_velocity(2);
    line.b = -relative_velocity(1);
    line.c = (relative_velocity(1)*P2(2)-relative_velocity(2)*P2(1));
else
    %line paramenters for the second and third intersection points
    line.a = P2(2) - P1(2);
    line.b = P1(1) - P2(1);
    line.c = P2(1)*P1(2) - P2(2)*P1(1);
end
end
