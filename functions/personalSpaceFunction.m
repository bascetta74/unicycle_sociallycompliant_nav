function [PSf, theta_heading, scenario] = personalSpaceFunction(v_ped,relative_velocity, robot_pos, ped_pos, r_c)

relative_velocity_wrt_robot = - relative_velocity;

%Here i compute the direction of the wheelchair on the 2D plane and the
%pedestrian velocity wrt the wheelchair and the direction of its
%velocity vector, to correct the sign of the relative velocity

c = - dot(ped_pos-robot_pos,relative_velocity_wrt_robot);

PSf=zeros(2,1);

%Now if v_rel>0, the pedestrian and the robot are approaching and may
%collide, so the PSf is computed according to the relative velocity
%between them. If v_rel<0 the two are not approaching, so the PSf
%parameters are computed considerng only the pedestrian velocity
%components
if sign(c)>0
    %If the pedestrian is moving the personal space function is
    %obtained as the composition of a circle and an ellipse. Otherwise
    %it is reduced to a circle
    if norm(v_ped) ~= 0
        PSf(1)=min(1.2, 0.3+3*norm(relative_velocity));
        PSf(2)=1.5*PSf(1);
        scenario = 0;
    else
        PSf(1)=min(1.2, 0.3+3*norm(relative_velocity));
        PSf(2)=PSf(1);
        scenario = 1;
    end
else
    %Same construction as before
    if norm(v_ped) ~= 0
        PSf(1)=min(1.2, 0.3+3*norm(v_ped));
        PSf(2)= 1.5*PSf(1);
        scenario = 2;
    else
        PSf(1)=0.3;
        PSf(2)=PSf(1);
        scenario = 3;
    end
end

%The third component determines the direction of the pedestrian
theta_heading = atan2(v_ped(1,2), v_ped(1,1));
end
