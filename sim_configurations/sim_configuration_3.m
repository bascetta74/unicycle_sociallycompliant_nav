function [mapFile,wheelchair_initial_pose,point_P_initial_pos,point_P_final_pos,...
    num_pedestrian,pedestrian_initial_pos,pedestrian_initial_vel,pedestrian_vel]=sim_configuration_3(epsilon)

% Environment map
mapFile = 'hallwayMap2';

% Wheelchair initial pose
xi  = 0;
yi  = 5.5;
thi = 0;
wheelchair_initial_pose = [xi,yi,thi];

% Point P initial and final position
x_pi = xi+epsilon*cos(thi);
y_pi = yi+epsilon*sin(thi);
point_P_initial_pos = [x_pi, y_pi];

x_pf = 10.0;
y_pf = 5.5;
point_P_final_pos = [x_pf, y_pf];

% Pedestrian initial position
x0_1 = -1.0;  y0_1 = 6.5;
x0_2 = 2.5;   y0_2 = 8.5;
x0_3 = 4.5;   y0_3 = 8.0;
x0_4 = 10.0;  y0_4 = 2.5;
x0_5 = 8.0;   y0_5 = 5.0;
x0_6 = 0.0;   y0_6 = 8.5;
pedestrian_initial_pos = [x0_1, y0_1;
    x0_2, y0_2;
    x0_3, y0_3;
    x0_4, y0_4;
    x0_5, y0_5;
    x0_6, y0_6];

% Pedestrian initial velocity
vx0_1 = 0.15;  vy0_1 =  0.0;
vx0_2 = 0.2;   vy0_2 =  0.0;
vx0_3 = 0.0;   vy0_3 = -0.2;
vx0_4 = -0.7;  vy0_4 =  0.0;
vx0_5 = -0.15; vy0_5 =  0.0;
vx0_6 = 0.3;   vy0_6 = -0.3;
pedestrian_initial_vel = [ vx0_1, vy0_1;
    vx0_2, vy0_2;
    vx0_3, vy0_3;
    vx0_4, vy0_4;
    vx0_5, vy0_5;
    vx0_6, vy0_6];

% Pedestrian velocity
pedestrian_vel{1} = @(t) (t<=5)*[vx0_1, vy0_1]+(t>5)*[0.3, 0.0];
pedestrian_vel{2} = @(t) [vx0_2, vy0_2];
pedestrian_vel{3} = @(t) (t<=20)*[vx0_3, vy0_3]+(t>20)*[0.2, 0.0];
pedestrian_vel{4} = @(t) (t<=12)*[vx0_4, vy0_4]+(t>12)*[0, 0.2];
pedestrian_vel{5} = @(t) (t<=13)*[vx0_5, vy0_5]+(t>13)*[0, 0.2];
pedestrian_vel{6} = @(t) (t<=12)*[vx0_6, vy0_6]+((t>12) && (t<=20))*[0.35, -0.35]+(t>20)*[0, -0.3];

% Number of pedestrians
num_pedestrian = size(pedestrian_initial_pos,1);
