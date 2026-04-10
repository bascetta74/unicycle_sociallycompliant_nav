function [PSf,VB,reducedVB] = PsfData(ped_pos,robot_pos,VR,v_ped,r_c,r_red)

% The function gets as input the pedestrian position, the relative
% velocity between the pedestrian and the robot and the pedestrian
% velcity. It gived as output the pedestrian virtual box as a struct
% of points, the reduced virtual box (not considering the robot
% dimension) and the personal space function, computed for the whole
% prediction horizon
%
% INPUTS:
%       P2 - pedestrian position
%       VR - relative velocity between the wheelchair and the
%            pedestrian
%       v_ped - pedestrian velocity
%       r_c - radius of the virtual box
%       r_red - radius of the reduced virtual box
%
% OUTPUTS:
%       PSf - personal space function struct
%       VB - virtual box struct
%       reducedVB - reduced virtual box struct


%% Pedestrian virtual box
%Here i define a struct of points x,y starting from a vector of angles theta.
%For each value in the th vector, i compute the points of the cricle
%with center in the current position of the pedestrian, considered
%inside the current prediction horizon

%x->r*cos(theta)+x_ped
%y->r*sin(theta)+y_ped

th=0:0.001:2*pi;

reducedVB.x=zeros(1,length(th));
reducedVB.y=zeros(1,length(th));

reducedVB.x(:)=r_red*cos(th)+ped_pos(1);
reducedVB.y(:)=r_red*sin(th)+ped_pos(2);

reducedVB=[reducedVB.x;reducedVB.y];

virtual_box.x=zeros(1,length(th));
virtual_box.y=zeros(1,length(th),1);

virtual_box.x=zeros(1,length(th));
virtual_box.y=zeros(1,length(th));

virtual_box.x(:)=r_c*cos(th)+ped_pos(1);
virtual_box.y(:)=r_c*sin(th)+ped_pos(2);

VB=[virtual_box.x;virtual_box.y];

%% Personal space function
%Slope used for the heading of the pedestrian, and of its PSf
slope=-pi/2+atan2(v_ped(2),v_ped(1));

%Struct of points x,y starting from a vector of angles th.
PS.x=zeros(1,length(th));
PS.y=zeros(1,length(th));

%Here i modify the sign of the relative velocity vector as in function
%personaSpaceFunction.m so that the PSf parameters depend on the
%relative velocity vector VR only if the robot and the human are in
%approaching relative motion

relative_velocity_wrt_robot = - VR;

c = - dot(ped_pos-robot_pos,relative_velocity_wrt_robot);

if sign(c)>0
    sigmax=min(1.2, 0.3+3*norm(VR));
    sigmay_re=sigmax;
    sigmay_fr=1.5*sigmax;
else
    sigmax=min(1.2, 0.3+3*norm(v_ped));
    sigmay_re=sigmax;
    sigmay_fr=1.5*sigmax;
end

%if the pedestrian is in motion, i compute the PSf as the composition
%of an ellipse and a circle
if norm(v_ped)~=0
    PS.x(:)=sigmax*cos(th)+ped_pos(1,1);
    PS.y(1,1:3141)=sigmay_fr*sin(th(1:3141))+ped_pos(2,1);
    PS.y(1,3142:length(th))=sigmay_re*sin(th(3142:length(th)))+ped_pos(2,1);

    Rz=[cos(slope) -sin(slope);
        sin(slope) cos(slope)];

    center=repmat(ped_pos,1,length(th));

    PSf=Rz*([PS.x;PS.y]-center)+center;
else
    %if the pedestrian is not moving, the personal space function is
    %reduced to a circle having radius sigmax
    PS.x(:)=sigmax*cos(th)+ped_pos(1);
    PS.y(:)=sigmax*sin(th)+ped_pos(2);
    PSf=[PS.x;PS.y];
end
end
