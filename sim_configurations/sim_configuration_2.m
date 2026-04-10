function [mapFile,wheelchair_initial_pose,point_P_initial_pos,point_P_final_pos,...
    num_pedestrian,pedestrian_initial_pos,pedestrian_initial_vel,pedestrian_vel]=sim_configuration_2(epsilon)

% Environment map
mapFile = 'hallwayMap1';

% Wheelchair initial pose
xi  = 0;
yi  = 6;
thi = 0;
wheelchair_initial_pose = [xi,yi,thi];

% Point P initial and final position
x_pi = xi+epsilon*cos(thi);
y_pi = yi+epsilon*sin(thi);
point_P_initial_pos = [x_pi, y_pi];

x_pf = 8;
y_pf = 6;
point_P_final_pos = [x_pf, y_pf];

% Pedestrian initial position
x0_1 = 2;   y0_1 = 2.5;
x0_2 = 2;   y0_2 = 3.5;
x0_3 = 7.5; y0_3 = 5.5;
x0_4 = 7.5; y0_4 = 6.5;
pedestrian_initial_pos = [x0_1, y0_1;
    x0_2, y0_2;
    x0_3, y0_3;
    x0_4, y0_4];

% Pedestrian initial velocity
vx0_1 = 0.1;  vy0_1 = 0;
vx0_2 = 0.1;  vy0_2 = 0;
vx0_3 = -0.2; vy0_3 = 0;
vx0_4 = -0.2; vy0_4 = 0;

pedestrian_initial_vel = [ vx0_1, vy0_1;
    vx0_2, vy0_2;
    vx0_3, vy0_3;
    vx0_4, vy0_4];

% Pedestrian velocity
pedestrian_vel{1} = @(t) [vx0_1, vy0_1];
pedestrian_vel{2} = @(t) [vx0_2, vy0_2];
pedestrian_vel{3} = @(t) [vx0_3, vy0_3];
pedestrian_vel{4} = @(t) [vx0_4, vy0_4];

% Number of pedestrians
num_pedestrian = size(pedestrian_initial_pos,1);
