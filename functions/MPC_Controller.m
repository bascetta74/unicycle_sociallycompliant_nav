function [v_px,v_py,pose,linear_position,U_sl,H_obs,L_obs,dl, opt_stat, csi, error, runtime] = MPC_Controller(x_p, y_p, Ts, N, v_max, q, r, s, a_max, d, Rw, omega_m,...
    omega_M, Usl_prev, csi_prev, curr_pose, curr_ped_pos, v_ped, sp, x_pr, y_pr, epsilon, n_obs, sa, r_c, r_red, T_gp, replanTrigger, param, e, N_samples)
%#codegen

sim_time = get_param('simulatore','SimulationTime');
count = round(sim_time/Ts) + 1;

persistent model;
persistent is_started;
persistent p_ped_samples;
persistent v_ped_samples;
persistent p_robot_samples;
persistent v_robot_samples;

if isempty(is_started)
    is_started = 0;
end

if isempty(p_ped_samples)
    p_ped_samples = zeros(N_samples, 2, n_obs);
end

if isempty(v_ped_samples)
    v_ped_samples = zeros(N_samples, 2, n_obs);
end

if isempty(p_robot_samples)
    p_robot_samples = zeros(N_samples, 2);
end

if isempty(v_robot_samples)
    v_robot_samples = zeros(N_samples, 2);
end

if is_started == 1
    for i_obstacles = 1:n_obs
        p_ped_samples(2:N_samples, 1, i_obstacles) = p_ped_samples(1:(N_samples-1), 1, i_obstacles);
        p_ped_samples(1, 1, i_obstacles) = curr_ped_pos(i_obstacles);
        p_ped_samples(2:N_samples, 2, i_obstacles) = p_ped_samples(1:(N_samples-1),2, i_obstacles);
        p_ped_samples(1, 2, i_obstacles) = curr_ped_pos(i_obstacles + n_obs);

        v_ped_samples(2:N_samples, 1, i_obstacles) = v_ped_samples(1:(N_samples-1), 1, i_obstacles);
        v_ped_samples(1, 1, i_obstacles) = v_ped(i_obstacles, 1);
        v_ped_samples(2:N_samples, 2, i_obstacles) = v_ped_samples(1:(N_samples-1), 2, i_obstacles);
        v_ped_samples(1, 2, i_obstacles) = v_ped(i_obstacles, 2);
    end

    p_robot_samples(2:N_samples,1) = p_robot_samples(1:(N_samples-1),1);
    p_robot_samples(1,1) = curr_pose(1);
    p_robot_samples(2:N_samples,2) = p_robot_samples(1:(N_samples-1),2);
    p_robot_samples(1,2) = curr_pose(2);

    curr_v_long = Usl_prev(1, 1)*cos(curr_pose(3)) + Usl_prev(2, 1)*sin(curr_pose(3));

    v_robot_samples(2:N_samples,1) = v_robot_samples(1:(N_samples-1),1);
    v_robot_samples(1,1) = curr_v_long*cos(curr_pose(3));
    v_robot_samples(2:N_samples,2) = v_robot_samples(1:(N_samples-1),2);
    v_robot_samples(1,2) = curr_v_long*sin(curr_pose(3));
end

if is_started == 0
    for i_obstacles = 1:n_obs
        p_ped_samples(1:N_samples, 1, i_obstacles) = curr_ped_pos(i_obstacles);
        p_ped_samples(1:N_samples, 2, i_obstacles) = curr_ped_pos(i_obstacles + n_obs);

        v_ped_samples(1:N_samples, 1, i_obstacles) = 0;
        v_ped_samples(1:N_samples, 2, i_obstacles) = 0;
    end

    p_robot_samples(1:N_samples, 1) = curr_pose(1);
    p_robot_samples(1:N_samples, 2) = curr_pose(2);

    v_robot_samples(1:N_samples, 1) = 0;
    v_robot_samples(1:N_samples, 2) = 0;

    is_started = 1;
end

if sim_time==60
    is_started = 0;
end

%% Current linear position and reference extraction
csi=[x_p;y_p];

Csi_r = extractTraj([x_pr; y_pr], N, count, true, T_gp, replanTrigger);

%% Clustering of obstacles

%Initialization of vectors containing pedestrians predicted motion
current_x_ped = zeros(n_obs,1);
current_y_ped = zeros(n_obs,1);

%Position of pedestrians (obstacle) within the prediction horizon
current_x_ped(1:n_obs,1)=curr_ped_pos(1:n_obs);
current_y_ped(1:n_obs,1)=curr_ped_pos(n_obs+1:end);

X_ped = [current_x_ped, current_y_ped];

idx = 1:1:n_obs;

n_groups = max(idx);

x_near = zeros(n_groups, 1);
y_near = zeros(n_groups, 1);

for i=1:n_groups
    X_group = X_ped(idx==i, :);
    distance = norm(X_group - curr_pose(1:2)');
    [~, I_ped] = min(distance);
    x_near(i) = X_group(I_ped, 1);
    y_near(i) = X_group(I_ped, 2);
end

%% Control variables vector initialization
U_sl=zeros(2*N+n_groups+2,1);
U=zeros(2*N,1);

%% Weight matrices
Q=[q 0;
    0 q];

R=[r 0;
    0 r];

S=[s 0;
    0 s];

%% State space system matrices
A=[1 0;
    0 1];
B=[Ts 0;
    0  Ts];

%% Computation of hessian matrix and gradient vector
A_h=zeros(2*N,2);
B_h=zeros(2*N,2*N);
H_sl=zeros(2*N+n_groups+2,2*N+n_groups+2);
f_sl=zeros(1,2*N+n_groups+2);

%State matrices for the open-loop prediction (HO CORRETTO UN ERRORE!)
x=1;
for i=1:2:2*N
    y=1;
    A_h(i:i+1,:)=A^(x);
    for j=1:2:i
        B_h(i:i+1,j:j+1)=A^(x-y)*B;
        y = y+1;
    end
    x = x+1;
end

%Computation of weigth matrices Q and R
Q_h=zeros(2*N);
R_h=zeros(2*N);

for i=1:2:2*N
    Q_h(i:i+1,i:i+1)=Q;
    R_h(i:i+1,i:i+1)=R;
end
Q_h(2*N-1:2*N,2*N-1:2*N)=S;

%Hessian matric
H=B_h'*Q_h*B_h+R_h;
H_sl(1:2*N,1:2*N)=H;
H_sl(2*N+1:2*N+n_groups,2*N+1:2*N+n_groups)=sp*eye(n_groups);
H_sl(2*N+n_groups+1:2*N+n_groups+2,2*N+n_groups+1:2*N+n_groups+2)=sa*eye(2);

%Gradient vector
f=(A_h*csi-Csi_r)'*Q_h*B_h;
f_sl(1,1:2*N)=f;
f_sl(1,2*N+1:2*N+n_groups+2)=0;

%% Predicted positions from previous prediction horizon
[pose,linear_position,v_long,~]=prediction(curr_pose,Usl_prev,csi_prev,A_h,B_h,N,Ts,epsilon);

%% Linear velocity constraints
A_vel1=zeros(2*N,2*N+n_groups+2);
b_vel=zeros(4*N,1);
A_vel2=zeros(2*N,2*N+n_groups+2);

b_vel(1:2*N,1)=omega_M;
b_vel(2*N+1:4*N,1)=-omega_m;
theta=pose(:,3);

j=1;
for i=1:2:2*N-1
    A_bar=(1/(2*Rw))*[2*cos(theta(j,1))-(d/epsilon)*sin(theta(j,1)), 2*sin(theta(j,1))+(d/epsilon)*cos(theta(j,1));
        2*cos(theta(j,1))+(d/epsilon)*sin(theta(j,1)), 2*sin(theta(j,1))-(d/epsilon)*cos(theta(j,1))];

    A_vel1(i:i+1,i:i+1)=A_bar;
    A_vel2(i:i+1,i:i+1)=-A_bar;

    j=j+1;
end

A_vel=[A_vel1;
    A_vel2];

%% Velocity variation constraints
A_dvel=zeros(4*N,2*N+n_groups+2);
deltaV_max=a_max*Ts;

deltaV=deltaV_max*ones(4*N,1);

A_var=zeros(4*N,2*N);
A_var(1:2*N,1:2*N)=eye(2*N);
A_var(2*N+1:4*N,1:2*N)=-eye(2*N);

V=zeros(2*N,2*N);

%max accelearion increment
dsl_a=0.04;

for i=1:2:2*N-1
    V(i:i+1,i:i+1)=eye(2);
    %last two columns of matrix A_dvel are related to slack variables
    A_dvel(i:i+1,end-1:end)=-eye(2);
    A_dvel(2*N+i:2*N+i+1,2*N+n_groups+1:end)=-eye(2);
    if i<2*N-2
        V(i+2:i+3,i:i+1)=-eye(2);
    end
end

v0=zeros(2*N,1);
v0(1,1)=Usl_prev(1);
v0(2,1)=Usl_prev(2);

A_dvel(:,1:2*N)=A_var*V;

b_dvel=deltaV+A_var*v0;

%% Lower/Upper bounds
lb=zeros(2*N+n_groups+2,1);
ub=zeros(2*N+n_groups+2,1);

lb(1:2*N,1)=-100;
ub(1:2*N,1)=100;

%Upper bound per le variabili di slack
ub(2*N+1:2*N+n_groups,1)=1;
ub(2*N+n_groups+1:end,1)=dsl_a;

%% Obstacle avoidance constraints
%Initialization of matrices/vectors for state constraints
%H_obs: for each obstacle H_obs is a marix (N,2*N) --> definition of a 3D
%matrix where dimension 3 is the number of obstacles considered
H_obs=zeros(N,2*N,n_groups);

%L_obs,dl: vectors of dimension N for each obstacle --> definition of
%2 matrices with dimension (N,n_groups) so to have one column for each
%considered pedestrian
L_obs=zeros(N,n_groups);
dl=zeros(N,n_groups);
dl_min=zeros(N,n_groups);

%Final matrix and vector for obstacle avoidance linear constraints
A_obs=zeros(n_groups*N,2*N+n_groups+2);
b_obs=zeros(N*n_groups,1);

%Initialization of vectors containing pedestrians predicted motion
x_ped=zeros(n_obs,N);
y_ped=zeros(n_obs,N);

%Position of pedestrians (obstacle) within the prediction horizon
x_ped(1:n_obs,1)=curr_ped_pos(1:n_obs);
y_ped(1:n_obs,1)=curr_ped_pos(n_obs+1:end);

for i=2:N
    x_ped(:,i)=x_ped(:,i-1)+Ts*v_ped(:,1);
    y_ped(:,i)=y_ped(:,i-1)+Ts*v_ped(:,2);
end

Na = 50;

for i_obs=1:n_groups

    %E_sl: defined as a matrix (N,n_groups)
    E_sl=zeros(N,n_groups);

    j=1;
    if norm([x_near(i_obs); y_near(i_obs)]-curr_pose(1:2))<=param.sensorRange
        for i=1:N

            relative_velocity=[v_long(i,1)*cos(theta(i,1))-v_ped(:,1), v_long(i,1)*sin(theta(i,1))-v_ped(:,2)];

            relative_velocity_cluster=mean([v_long(i,1)*cos(theta(i,1))-v_ped(idx==i_obs,1), v_long(i,1)*sin(theta(i,1))-v_ped(idx==i_obs,2)], 1).';

            robot_pos=[pose(i,1); pose(i,2)];
            ped_pos=[x_ped(:,i), y_ped(:,i)];

            %Computation of tangent points
            [Q1, Q2, robot_inside_obst]=findTangentPoints_convex_hull(robot_pos, ped_pos, v_ped, relative_velocity, r_c, r_red, idx, i_obs, Na);

            %Directions of the 2 limit velocity vectors for collision cone
            v1=[Q1(1)-robot_pos(1);
                Q1(2)-robot_pos(2)];
            v2=[Q2(1)-robot_pos(1);
                Q2(2)-robot_pos(2)];
            V_12=[v1 v2];

            %the wheelchair geometric center predicted position is outside the
            %virtual box
            if robot_inside_obst==0
                alfa_12=V_12\relative_velocity_cluster;
            else
                alfa_12=[0;0];
            end
            %if the robot is in potential collision with the obstacle the constraint is computed,
            %otherwise no constraint is added
            if (robot_inside_obst==1 || (alfa_12(1)>0 && alfa_12(2)>0))
                %computation of the intersection point with the virtualbox
                %and of the constraint line coefficients
                [hx, hy, l, d_min]=constraintCoefficients_convex_hull(robot_pos, ped_pos, v_ped, relative_velocity, r_c, r_red, idx, i_obs);

                H_obs(i,j:j+1,i_obs)=[hx hy];L_obs(i,i_obs)=l;

                %safety distance computation
                dl_sl = 0;
                dl(i,i_obs)=dl_sl;
                dl_min(i,i_obs)=d_min;
                disp(dl_sl);
            end

            E_sl(i,i_obs)=-dl(i,i_obs)-dl_min(i,i_obs);

            %incrementing counter
            j=j+2;
        end
    end


    %definition of matrix A_obs for each obstacle j (last two columns are
    %columns of N zeros, related to the slack variables on acceleration u_sla)
    A_obsj=[H_obs(:,:,i_obs)*B_h E_sl zeros(N,2)];
    b_obsj=L_obs(:,i_obs)-dl(:,i_obs)-H_obs(:,:,i_obs)*A_h*csi;

    A_obs(1+(i_obs-1)*N:i_obs*N,:)=A_obsj;
    b_obs(1+(i_obs-1)*N:i_obs*N)=b_obsj;
end

%% Slack variables constraints
Asl_p=zeros(2*n_groups,2*N+n_groups+2);
bsl_p=zeros(2*n_groups,1);
Asl_a=zeros(4,2*N+n_groups+2);
bsl_a=[dsl_a;dsl_a;0;0];

Asl_p(1:n_groups,2*N+1:2*N+n_groups)=eye(n_groups);
Asl_p(n_groups+1:2*n_groups,2*N+1:2*N+n_groups)=-eye(n_groups);
bsl_p(1:n_groups,1)=1;
Asl_a(1:2,end-1:end)=eye(2);
Asl_a(3:4,end-1:end)=-eye(2);

Asl=[Asl_p;
    Asl_a];
bsl=[bsl_p;
    bsl_a];

%% Constraint matrices
Aineq=[A_vel;
    A_dvel ;
    A_obs  ;
    Asl   ];

bineq=[b_vel;
    b_dvel ;
    b_obs  ;
    bsl   ];

%% Optimization step
if count == 1
    %Initialization
    model.modelsense = 'min';
    model.sense = '<';

    model.Q = sparse(0.5*H_sl);
    model.obj = f_sl;

    model.lb = lb;
    model.ub = ub;

    model.A = sparse(Aineq);
    model.rhs = bineq;

    %Solve the problem
    params.OutputFlag =0;
    results = gurobi(model, params);
else
    %Update the QP problem
    model.Q = sparse(0.5*H_sl);
    model.obj = f_sl;

    model.lb = lb;
    model.ub = ub;

    model.A = sparse(Aineq);
    model.rhs = bineq;

    %Solve the problem
    params.OutputFlag = 0;
    results = gurobi(model, params);
end

%Extract results
status = 0;
if (strcmp(results.status,'OPTIMAL')||strcmp(results.status,'SUBOPTIMAL'))
    U_sl=results.x; U=U_sl(1:2*N);
    v_px = U(1);
    v_py = U(2);
    status(strcmp(results.status,'OPTIMAL')) = 1;
    status(strcmp(results.status,'SUBOPTIMAL')) = 2;
    opt_stat = [status; results.objval];
else
    disp('No optimal solution found\n');
    disp(results.status);
    opt_stat = [status; 0];
    v_px = 0;
    v_py = 0;
end

%Compute error on first prediction step to discriminate on replanning
pred = A_h*csi+B_h*U;
error = norm(Csi_r(1:2)-pred(1:2));

%Extract gurobi runtime
runtime = results.runtime;
end
