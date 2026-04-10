function [hx, hy, l, d_min]=constraintCoefficients_convex_hull(robot_pos, current_ped_pos, current_v_ped, current_relative_velocity, r_c, r_red, idx, i_group)

tol = 1e-8;

ped_pos_cluster = current_ped_pos(idx==i_group, :);
v_ped_cluster = current_v_ped(idx==i_group, :);
relative_velocity_cluster = current_relative_velocity(idx==i_group, :);

PSf_transpose_cluster = zeros(2, size(ped_pos_cluster, 1));
theta_heading_cluster = zeros(size(ped_pos_cluster, 1), 1);
D = zeros(2, 2, size(ped_pos_cluster, 1));
B = zeros(2, 2, size(ped_pos_cluster, 1));
m = zeros(size(ped_pos_cluster, 1), 1);
coefficients = zeros(1, 3);

delta = (r_c - r_red);

x_centroid = mean(ped_pos_cluster(:,1));
y_centroid = mean(ped_pos_cluster(:,2));

coefficients(1) = (x_centroid - robot_pos(1))/(sqrt((x_centroid - robot_pos(1))^2 + (y_centroid - robot_pos(2))^2) + 1e-6);
coefficients(2) = (y_centroid - robot_pos(2))/(sqrt((x_centroid - robot_pos(1))^2 + (y_centroid - robot_pos(2))^2) + 1e-6);

coefficients(3) = coefficients(1)*x_centroid + coefficients(2)*y_centroid;

for i=1:size(ped_pos_cluster, 1)
    [PSf_transpose_cluster(:, i), theta_heading_cluster(i), ~] = personalSpaceFunction(v_ped_cluster(i,:), relative_velocity_cluster(i,:)', robot_pos, ped_pos_cluster(i, :)', r_c);

    a = PSf_transpose_cluster(2, i) + delta;
    b = PSf_transpose_cluster(1, i) + delta;

    D(:, :, i) = diag([a, b]);

    R = [cos(theta_heading_cluster(i)), -sin(theta_heading_cluster(i));
        sin(theta_heading_cluster(i)), cos(theta_heading_cluster(i))];

    B(:, :, i) = R*D(:, :, i);

    if dot(coefficients(1:2), [cos(theta_heading_cluster(i)), sin(theta_heading_cluster(i))]) >= 0
        m(i) = coefficients(1:2)*(ped_pos_cluster(i, :)') - coefficients(3) - b;
    else
        m(i) = coefficients(1:2)*(ped_pos_cluster(i, :)') - coefficients(3) - norm(B(:, :, i)'*coefficients(1:2)');
    end

end

isHullTangentOriginal = all(m >= -tol);

hx = coefficients(1);
hy = coefficients(2);
[~, I_m] = min(m);

if isHullTangentOriginal
    l = coefficients(3);
    d_min = [hx, hy]*(ped_pos_cluster(I_m, :)') - l - r_c;
else

    l =  coefficients(3) + m(I_m);
    d_min = [hx, hy]*(ped_pos_cluster(I_m, :)') - l - r_c;
end
end
