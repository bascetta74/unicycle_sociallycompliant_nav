function [pose, linear_position,v_long,U_pred]=prediction(curr_pose,Usl_prev,csi_prev,A_h,B_h,N,Ts,epsilon)

%The function gets as input the current robot position, the previous
%optimal control sequence and the previous linear position of the
%robot. The outputs of the function are the robot estimated linear
%position, geometric center position, longitudinal velocity and linear
%velocities.

%Estimate of the robot heading using the optimal control sequence
%computed at the prev prediction horizon
theta=zeros(N,1);

%The orientation of the robot at the first instant of the prediction
%horizon is the real orientation measured and given as input for the
%MPC
theta(1,1)=curr_pose(3);

v_long=zeros(N,1);

%Vector of previous prediction horizon control varialbes v_px, v_py
U_prev=Usl_prev(1:2*N,1);

%Estimation of the robot headin and longitudinal vector along the
%current prediction horizon, using the vector of linear velocities
%U_prev computed in the previous prediction horizon
%The first component of U_prev is the pair of velocities that moved the
%robot to its current position, so i discard it
j=3;
for i=2:N
    omega=(U_prev(j+1,1)*cos(theta(i-1,1))-U_prev(j,1)*sin(theta(i-1,1)))/epsilon;
    theta(i,1)=theta(i-1,1)+omega*Ts;
    v_long(i-1,1)=U_prev(j,1)*cos(theta(i-1,1))+U_prev(j+1,1)*sin(theta(i-1,1));
    j=j+2;
end

%I assume that the longitudinal velocity at the last prediction step is
%the same as the velocity at instant N-1
v_long(N,1)=v_long(N-1,1);

%Estimation of the linear positions of the wheelchair, using the
%optimal control sequence computed at the previous prediction horizon
xp_est=zeros(N,1);
yp_est=zeros(N,1);
Csi_prev=zeros(2*N,1);

Csi_prev(1:2*N,1)=A_h*csi_prev+B_h*U_prev;
xp_est(1:N,1)=Csi_prev(1:2:end-1,1);
yp_est(1:N,1)=Csi_prev(2:2:end,1);

%Estimation of the position of the robot geometric center, used for the
%computation of the position constraints
x=zeros(N,1);
y=zeros(N,1);

x(1,1)=curr_pose(1); y(1,1)=curr_pose(2);

x(2:N,1)=xp_est(2:N,1)-epsilon*cos(theta(2:N,1));
y(2:N,1)=yp_est(2:N,1)-epsilon*sin(theta(2:N,1));

%I add the last pair of velocity components, assuming the robot will
%keep the same velocity in the last prediction step
U_pred=[U_prev(3:2*N,1);
    U_prev(2*N-1:2*N,1)];

%Definition of the pose and linear position vectors
pose = [x, y, theta];
linear_position = [xp_est, yp_est];
end
