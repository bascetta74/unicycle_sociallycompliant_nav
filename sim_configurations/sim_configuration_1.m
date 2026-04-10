function [mapFile,wheelchair_initial_pose,point_P_initial_pos,point_P_final_pos,...
    num_pedestrian,pedestrian_initial_pos,pedestrian_initial_vel,pedestrian_vel]=sim_configuration_1(epsilon)

% Environment map
mapFile = 'hallwayMap1';

% Wheelchair initial pose
xi  = 0;
yi  = 5;
thi = 0;
wheelchair_initial_pose = [xi,yi,thi];

% Point P initial and final position
x_pi = xi+epsilon*cos(thi);
y_pi = yi+epsilon*sin(thi);
point_P_initial_pos = [x_pi, y_pi];

x_pf = 8.5;
y_pf = 5.0;
point_P_final_pos = [x_pf, y_pf];

% Pedestrian initial position
x0_1 = 8.5; y0_1 = 5.0;
pedestrian_initial_pos = [x0_1, y0_1];

% Pedestrian initial velocity
vx0_1 = -0.1;   vy0_1 =  0.0;
pedestrian_initial_vel = [ vx0_1, vy0_1];

% Pedestrian velocity
pedestrian_vel{1} = @(t) [vx0_1, vy0_1];

% Number of pedestrians
num_pedestrian = size(pedestrian_initial_pos,1);
