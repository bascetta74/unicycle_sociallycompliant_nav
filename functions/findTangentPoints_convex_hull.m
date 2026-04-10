function [Q1, Q2, robot_inside_group]=findTangentPoints_convex_hull(robot_pos, current_ped_pos, current_v_ped, current_relative_velocity, r_c, r_red, idx, i_group, Na)

tol = 1e-8;

ped_pos_cluster = current_ped_pos(idx==i_group, :);
v_ped_cluster = current_v_ped(idx==i_group, :);
relative_velocity_cluster = current_relative_velocity(idx==i_group, :);

Q1=zeros(2,1);
Q2=zeros(2,1);

PSf_transpose_cluster = zeros(2, size(ped_pos_cluster, 1));
theta_heading_cluster = zeros(size(ped_pos_cluster, 1), 1);
D = zeros(2, 2, size(ped_pos_cluster, 1));
B = zeros(2, 2, size(ped_pos_cluster, 1));
Q1_tot = zeros(3, size(ped_pos_cluster, 1));
Q2_tot = zeros(3, size(ped_pos_cluster, 1));
robot_inside_obst = zeros(size(ped_pos_cluster, 1), 1);

vector_cross = zeros(2*size(ped_pos_cluster, 1), 1);

theta = linspace(0, 2*pi, Na+1)';
theta(end) = [];

n_1 = cos(theta);
n_2 = sin(theta);

n = [n_1,n_2];

s = zeros(size(ped_pos_cluster, 1), size(n, 1));

delta = (r_c - r_red);

for i=1:size(ped_pos_cluster, 1)
    [PSf, theta_heading, ~] = personalSpaceFunction(v_ped_cluster(i,:), relative_velocity_cluster(i,:)', robot_pos, ped_pos_cluster(i, :)', r_c);
    [Q1_tot(1:2,i) , Q2_tot(1:2,i), robot_inside_obst(i)]=findTangentPoints(robot_pos,ped_pos_cluster(i, :)',PSf,theta_heading, r_red, r_c);
end

for i=1:size(ped_pos_cluster, 1)
    [PSf_transpose_cluster(:, i), theta_heading_cluster(i), ~] = personalSpaceFunction(v_ped_cluster(i,:), relative_velocity_cluster(i,:)', robot_pos, ped_pos_cluster(i, :)', r_c);

    a = PSf_transpose_cluster(2, i) + delta;
    b = PSf_transpose_cluster(1, i) + delta;

    D(:, :, i) = diag([a, b]);

    R = [cos(theta_heading_cluster(i)), -sin(theta_heading_cluster(i));
        sin(theta_heading_cluster(i)), cos(theta_heading_cluster(i))];

    B(:, :, i) = R*D(:, :, i);

    for j=1:size(n, 1)
        if dot(n(j,:), [cos(theta_heading_cluster(i)), sin(theta_heading_cluster(i))]) >= 0
            s(i,j) = n(j,:)*(ped_pos_cluster(i, :)') + norm(B(:, :, i)'*n(j,:)');
        else
            s(i,j) = n(j,:)*(ped_pos_cluster(i, :)') + b;
        end
    end


end

s_convex_hull = max(s, [], 1).';

dist = s_convex_hull - n*robot_pos;

isInsideGroup = all(dist >= -tol);

vect_Q_tot = [Q1_tot, Q2_tot] - [robot_pos; 0];

if isInsideGroup
    robot_inside_group = 1;
else
    robot_inside_group = 0;

    for k=1:2*size(ped_pos_cluster, 1)
        vector_cross(k) = atan2(vect_Q_tot(1, 1)*vect_Q_tot(2, k) - vect_Q_tot(2, 1)*vect_Q_tot(1, k), vect_Q_tot(1, 1)*vect_Q_tot(1, k) + vect_Q_tot(2, 1)*vect_Q_tot(2, k));
    end

    [~, I_min] = min(vector_cross, [], 1);
    [~, I_max] = max(vector_cross, [], 1);

    Q1 = vect_Q_tot(1:2, I_min) + robot_pos;
    Q2 = vect_Q_tot(1:2, I_max) + robot_pos;

end
end
