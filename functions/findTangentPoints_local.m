function [Q1_local,Q2_local]= findTangentPoints_local(robot_pos_local, a, b)

x0 = robot_pos_local(1);
y0 = robot_pos_local(2);

R = hypot(x0/a, y0/b);
if R <= 1
    error('Punto non esterno all''ellisse.');
end
phi = atan2(y0/b, x0/a);
delta = acos(1/R);

t1 = phi + delta;
t2 = phi - delta;

Q1_local = [a*cos(t1); b*sin(t1)];
Q2_local = [a*cos(t2); b*sin(t2)];
end
