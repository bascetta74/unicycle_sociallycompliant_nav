function [Q1,Q2,robot_inside_obs]= findTangentPoints(robot_pos,ped_pos,PSf,theta_heading,r_red,r_c)

% The function gets as input the position of the wheelchair and
% pedestrian, with respect to the global reference, giving as output
% the two tangent points on the virtual box, defining the bounds of the
% collision cone for the relative velocity
%
% INPUTS:
%       robot_pos - wheelchair position
%       ped_pos   - pedestrian position
%
% OUTPUTS:
%       Q1 - first tangent point (+alfa)
%       Q2 - second tangent point (-alfa)
%       robot_inside_obs - varaible to check the postion of the robot

Q1=zeros(2,1);
Q2=zeros(2,1);

robot_inside_obs=0;

%Tangent points computation
%pedestrian-robot distance
robot_pos_local = [cos(theta_heading), sin(theta_heading); -sin(theta_heading), cos(theta_heading)]*(robot_pos - ped_pos);

delta = (r_c - r_red);

a = PSf(2) + delta;
b = PSf(1) + delta;

if (robot_pos_local(1) >= 0 && ((robot_pos_local(1)^2)/((a)^2) + (robot_pos_local(2)^2)/((b)^2) > 1)) || (robot_pos_local(1) < 0 && ((robot_pos_local(1)^2)/((b)^2) + (robot_pos_local(2)^2)/((b)^2) > 1))

    [Q1_circle_local, Q2_circle_local] = findTangentPoints_local(robot_pos_local, b, b);

    if Q1_circle_local(1) < 0 && Q2_circle_local(1) < 0
        Q1 = [cos(theta_heading), -sin(theta_heading); sin(theta_heading), cos(theta_heading)]*Q1_circle_local + ped_pos;
        Q2 = [cos(theta_heading), -sin(theta_heading); sin(theta_heading), cos(theta_heading)]*Q2_circle_local + ped_pos;
    else
        [Q1_ellips_local, Q2_ellips_local] = findTangentPoints_local(robot_pos_local, a, b);
        if  Q1_circle_local(1) < 0 && Q2_circle_local(1) >=0
            Q1 = [cos(theta_heading), -sin(theta_heading); sin(theta_heading), cos(theta_heading)]*Q1_circle_local + ped_pos;
            Q2 = [cos(theta_heading), -sin(theta_heading); sin(theta_heading), cos(theta_heading)]*Q2_ellips_local + ped_pos;
        elseif Q1_circle_local(1) >= 0 && Q2_circle_local(1) < 0
            Q1 = [cos(theta_heading), -sin(theta_heading); sin(theta_heading), cos(theta_heading)]*Q1_ellips_local + ped_pos;
            Q2 = [cos(theta_heading), -sin(theta_heading); sin(theta_heading), cos(theta_heading)]*Q2_circle_local + ped_pos;
        else
            Q1 = [cos(theta_heading), -sin(theta_heading); sin(theta_heading), cos(theta_heading)]*Q1_ellips_local + ped_pos;
            Q2 = [cos(theta_heading), -sin(theta_heading); sin(theta_heading), cos(theta_heading)]*Q2_ellips_local + ped_pos;
        end
    end
else
    %estimated robot position inside the virtual box
    robot_inside_obs=1;
end
end
