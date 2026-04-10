close all
clear all
clc

addpath('./functions');
addpath('./maps');
addpath('./RRTx');
addpath('./sim_configurations');

%% Parameters

% Controller parameters
Ts = 0.2;      % Sampling time MPC
N = 20;        % MPC prediction horizon
epsilon = 0.5; % Distance of point P form wheel axel

% Wheelchair parameters
d = 0.650;  % Wheel contact point distance [m]
Rw = 0.36;  % Wheel radius [m]

v_max = 0.55;        % Maximum linear velocity [m/s]
a_max = 0.20;        % Maximum linear acceleration [m/s^2]
omega_M = v_max/Rw;  % Maximum wheel velocity [rad/s]
omega_m = -v_max/Rw; % Minimum wheel velocity [rad/s]

r_red=0.3;
r_c=0.75;

%% AMCL parameters
rho            = 0.90;
m_pos          = 0.03;
q_pos          = 0.01;
m_theta        = 0.02;
q_theta        = 0.01;
sigma_min_fact = 0.5;
sigma_max_fact = 3;
p_fail         = 5e-4;
lost_duration  = 10;
qt_lost_thd    = 0.1;
max_jump_dist  = 0.25;
AMCL_param = [rho,m_pos,q_pos,m_theta,q_theta,sigma_min_fact,sigma_max_fact,...
    p_fail,lost_duration,qt_lost_thd,max_jump_dist];

%% Pedestrian position estimator parameters
Ts_pedpos_est = 0.1;
sigma_q = 0.01;
sigma_m = 0.005;
pos_estim_param = [sigma_q,sigma_m];

%% Simulation configuration
[mapFile,initial_cond,startLoc,endLoc,n_obs,Ped_init,v_ped,v_ped_fun] = sim_configuration_3(epsilon);
x_p0 = startLoc(1);
y_p0 = startLoc(2);

%% MPC controller tuning parameters
q = 1;
r = 1;
sp = 10^9;
sa = 10^3;
%
% % k should be k>-2/Ts, and to avoid excessive oscillations k>-1/Ts
k = -1/(2*Ts);
s = (q+k^2*r)/(1-(1+Ts*k)^2);

e = 0.1;
N_samples = 40;

%% Planner tuning parameters
%Global planner trigger intrval
T_gp = 4;

% At initialization phase, the pedestrian path is simply its initial
% position. The vector is reshaped to keep consistency with dimensions in
% the rest of the code
path_ped = reshape(Ped_init,[1 n_obs*2]);
path_ped = repmat(path_ped,[N 1]);

% Find first path for the wheelchair
[graphTree, node, path_x, path_y, path_plot, path_graph, param, obstacle, map] = RRTXmain(path_ped, r_c, Ts, startLoc, endLoc, v_ped, n_obs, N, mapFile, true, 0, false);
[x_pri, y_pri, qd, qdd, tvec] = trajectoryGeneration(path_x, path_y, v_max, a_max,Ts, 0);

rng(0,'twister');
